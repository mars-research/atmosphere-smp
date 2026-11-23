//! Bootloader integration.
//!
//! We are loaded by the boot manager which implements Xen PVH and Multiboot v2.

pub mod ap_start;
pub mod command_line;

use core::arch::asm;
use core::sync::atomic::{AtomicU64, AtomicU8, Ordering::Relaxed};

use astd::boot::BootInfo;
use x86::io::{inb, outb, outl, outw};

pub use command_line::get_command_line;

static mut BOOT_INFO: BootInfo = BootInfo::empty();

/// Global Tagged TLB toggle.
pub static mut PCIDE: u64 = 0;

/// The QEMU `isa-debug-exit` port (`-device isa-debug-exit,iobase=0xf4,iosize=0x04`).
const QEMU_DEBUG_EXIT_PORT: u16 = 0xf4;

pub unsafe fn init(boot_info: *const BootInfo) {
    init_tsc_hz();
    BOOT_INFO = (&*boot_info).clone();
    PCIDE = if BOOT_INFO.pcide { 1 } else { 0 };

    command_line::init(BOOT_INFO.command_line.as_str()).expect("Invalid kernel command-line");
    let cmdline = get_command_line();
    // The console keeps appending to the RAM log buffer that the loader started and mapped.
    if let Some(addr) = cmdline.ramlog {
        let attached = {
            let _console = crate::console::get_writer();
            ns16550a::ramlog::attach(addr)
        };
        assert!(attached, "ramlog={:#x} holds no RAM log header", addr);
    }
    if let Some(seconds) = cmdline.reset_after {
        let (hz, source) = tsc_hz();
        RESET_DEADLINE = core::arch::x86_64::_rdtsc() + seconds * hz;
        log::info!("Resetting the machine after {} s (TSC {} MHz from {})", seconds, hz / 1_000_000, source);
    }
}

/// The TSC value after which the timer interrupt resets the machine (`reset_after=`), or 0.
pub static mut RESET_DEADLINE: u64 = 0;

/// Where the TSC rate of `tsc_hz` comes from; the discriminant is the source code that `NR_DEBUG_TSC_HZ` returns.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TscSource {
    Assumed = 0,
    Cpuid16h = 1,
    CpuidHypervisor = 2,
    Pit = 3,
}

impl core::fmt::Display for TscSource {
    fn fmt(&self, f: &mut core::fmt::Formatter) -> core::fmt::Result {
        f.write_str(match self {
            TscSource::Assumed => "an assumed 3 GHz",
            TscSource::Cpuid16h => "CPUID 16H",
            TscSource::CpuidHypervisor => "CPUID 4000_0010H",
            TscSource::Pit => "the PIT",
        })
    }
}

/// The TSC rate and its source, set once by `init_tsc_hz` on CPU 0 before the APs start.
static TSC_HZ: AtomicU64 = AtomicU64::new(0);
static TSC_SOURCE: AtomicU8 = AtomicU8::new(TscSource::Assumed as u8);

/// The TSC rate that `init_tsc_hz` found, and its source.
pub fn tsc_hz() -> (u64, TscSource) {
    let source = match TSC_SOURCE.load(Relaxed) {
        1 => TscSource::Cpuid16h,
        2 => TscSource::CpuidHypervisor,
        3 => TscSource::Pit,
        _ => TscSource::Assumed,
    };
    (TSC_HZ.load(Relaxed), source)
}

/// Sets the TSC rate from CPUID 16H or 4000_0010H; when neither gives one, measures it against the PIT, and
/// assumes 3 GHz when the PIT does not count. User mode has no I/O privilege, hence `NR_DEBUG_TSC_HZ` reports it.
unsafe fn init_tsc_hz() {
    use core::arch::x86_64::__cpuid;
    let (hz, source) = if __cpuid(0).eax >= 0x16 && __cpuid(0x16).eax != 0 {
        (__cpuid(0x16).eax as u64 * 1_000_000, TscSource::Cpuid16h)
    } else if __cpuid(0x4000_0000).eax >= 0x4000_0010 && __cpuid(0x4000_0010).eax != 0 {
        (__cpuid(0x4000_0010).eax as u64 * 1_000, TscSource::CpuidHypervisor)
    } else if let Some(hz) = measure_tsc_hz() {
        (hz, TscSource::Pit)
    } else {
        log::warn!("TSC rate: CPUID gives none and the PIT does not count");
        (3_000_000_000, TscSource::Assumed)
    };
    TSC_HZ.store(hz, Relaxed);
    TSC_SOURCE.store(source as u8, Relaxed);
}

/// The input clock of the PIT, in Hz.
const PIT_HZ: u64 = 1_193_182;
/// PIT ticks of one measurement round, about 10 ms.
const PIT_ROUND_TICKS: u64 = PIT_HZ / 100;
const PIT_ROUNDS: usize = 5;
/// Polls of OUT2 after which a round gives up: about 1 s at the 1 µs that one ISA port read takes.
const PIT_MAX_POLLS: u64 = 1_000_000;
/// Port 0x61: bit 0 gates PIT channel 2, bit 1 enables the speaker, bit 5 reads OUT2.
const PORT_61: u16 = 0x61;
const PIT_CHANNEL_2: u16 = 0x42;
const PIT_COMMAND: u16 = 0x43;
/// Channel 2, low then high count byte, mode 0 (OUT2 rises at terminal count), binary.
const PIT_CHANNEL_2_MODE_0: u8 = 0xb0;

/// The TSC rate measured against PIT channel 2, or `None` when the PIT does not count. Each round counts
/// `PIT_ROUND_TICKS` down in mode 0, gate on and speaker off. The TSC is read before the count starts and after OUT2 is
/// seen high, so every round can only overestimate the interval, and the least of the rounds is the closest.
unsafe fn measure_tsc_hz() -> Option<u64> {
    let saved = inb(PORT_61);
    outb(PORT_61, (saved & !0x02) | 0x01);
    let (mut least, mut most) = (u64::MAX, 0);
    for _ in 0..PIT_ROUNDS {
        outb(PIT_COMMAND, PIT_CHANNEL_2_MODE_0);
        outb(PIT_CHANNEL_2, PIT_ROUND_TICKS as u8);
        let start = core::arch::x86_64::_rdtsc();
        outb(PIT_CHANNEL_2, (PIT_ROUND_TICKS >> 8) as u8);
        let mut polls = 0;
        while inb(PORT_61) & 0x20 == 0 && polls < PIT_MAX_POLLS {
            polls += 1;
        }
        let cycles = core::arch::x86_64::_rdtsc() - start;
        // OUT2 high at once or never: the PIT does not count.
        if polls == 0 || polls == PIT_MAX_POLLS {
            outb(PORT_61, saved);
            return None;
        }
        (least, most) = (least.min(cycles), most.max(cycles));
    }
    outb(PORT_61, saved);
    let hz = least * PIT_HZ / PIT_ROUND_TICKS;
    log::info!("TSC rate {} kHz from the PIT: least of {} rounds of {} ticks, {} to {} TSC cycles", hz / 1000, PIT_ROUNDS, PIT_ROUND_TICKS, least, most);
    Some(hz)
}

/// Resets the machine when `reset_after=` is set and its time has passed: called on every timer tick.
#[inline(always)]
pub fn check_reset_deadline() {
    let deadline = unsafe { RESET_DEADLINE };
    if deadline != 0 && unsafe { core::arch::x86_64::_rdtsc() } >= deadline {
        log::error!("reset_after: time is up, resetting the machine");
        unsafe { crate::power::log_frequency() };
        ns16550a::ramlog::finish(ns16550a::ramlog::REASON_TIMEOUT, 0);
        unsafe { ns16550a::ramlog::reset() }
    }
}

pub fn get_boot_info() -> &'static BootInfo {
    unsafe { &BOOT_INFO }
}

/// How much of the zeroed boot stack CPU 0 has touched: the first non-zero byte above its base
/// bounds the deepest frame. Only meaningful while CPU 0 still runs on that stack.
pub fn boot_stack_high_water() -> usize {
    let info = get_boot_info();
    let size = info.kernel_stack_size as usize;
    let stack = unsafe { core::slice::from_raw_parts(info.kernel_stack_base as *const u8, size) };
    size - stack.iter().position(|b| *b != 0).unwrap_or(size)
}

/// Returns the raw kernel command line.
pub fn get_raw_command_line() -> &'static str {
    unsafe { BOOT_INFO.command_line.as_str() }
}

/// Exits QEMU with status `(code << 1) | 1`; with an active RAM log (`ramlog=`) it records `reason` (a
/// `ns16550a::ramlog::REASON_*`) and `code` there and resets the machine instead.
pub unsafe fn exit_qemu(reason: u32, code: u32) -> ! {
    use core::sync::atomic::Ordering::Relaxed;
    use crate::platform::{REMOTE_RUNNING_TO_IDLE_IPIS, WAKE_IPIS};
    let high_water = (0..crate::cpu::present_cpus()).map(crate::cpu::kernel_stack_high_water).max().unwrap_or(0);
    log::info!(
        "exit({}): kernel stack high-water {} KiB; remote flushes {} (served {}), running-to-idle IPIs {}, wake IPIs {}",
        code, high_water / 1024, crate::cpu::remote_flushes(), crate::platform::nmi_flushes(), REMOTE_RUNNING_TO_IDLE_IPIS.load(Relaxed), WAKE_IPIS.load(Relaxed),
    );
    crate::power::log_frequency();
    if ns16550a::ramlog::base() != 0 {
        ns16550a::ramlog::finish(reason, code);
        ns16550a::ramlog::reset();
    }
    outl(QEMU_DEBUG_EXIT_PORT, code);
    spin_forever();
}

/// Shutdown the system.
///
/// On virtual platforms it's possible to set an exit code to
/// be returned by the hypervisor.
pub unsafe fn shutdown(success: bool) -> ! {
    log::info!("The system is shutting down...");

    let cmdline = get_command_line();

    // QEMU isa-debug-exit
    //
    // <https://github.com/qemu/qemu/blob/bd662023e683850c085e98c8ff8297142c2dd9f2/hw/misc/debugexit.c>
    if let Some(io_base) = cmdline.qemu_debug_exit_io_base {
        if !success {
            log::debug!(
                "Trying QEMU isa-debug-exit shutdown (IO Port {:#x})",
                io_base
            );

            // QEMU will exit with (val << 1) | 1
            outw(io_base, 0x0);
        }
    }

    // Bochs APM
    if let Some(io_base) = cmdline.bochs_apm_io_base {
        let success_marker = if success {
            "BOCHS_SUCCESS"
        } else {
            "BOCHS_FAILURE"
        };
        log::debug!(
            "Trying Bochs APM shutdown (IO Port {:#x}) - {}",
            io_base,
            success_marker
        );

        let shutdown = "Shutdown";

        for ch in shutdown.chars() {
            outb(io_base, ch as u8);
        }
    }

    // QEMU
    outw(0x604, 0x2000);

    log::info!("It is now safe to turn off your computer"); // ;)

    spin_forever();
}

/// Enters a busy loop.
pub unsafe fn spin_forever() -> ! {
    loop {
        asm!("hlt");
    }
}
