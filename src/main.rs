#![no_std]
#![no_main]

mod time;
mod task_semaphore;

use cortex_m::interrupt;
use cortex_m_semihosting::debug::{self, EXIT_FAILURE};
#[cfg(feature = "rtt")]
use defmt_rtt as _;
#[cfg(feature = "semihosting")]
use defmt_semihosting as _;
#[cfg(not(any(feature = "rtt", feature = "semihosting")))]
compile_error!("No global logger selected, enable either the rtt or semihosting feature");

use stm32f4xx_hal as _;

const WCET_THRESHOLD: u32 = 100;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    interrupt::disable();

    defmt::error!("Panic: {}", info);
    debug::exit(EXIT_FAILURE);

    loop {}
}

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI1, EXTI2],
)]
mod app {
    use crate::{
        task_semaphore::{TaskSemaphoreSignaler, TaskSemaphoreWaiter, TaskSemaphore},
        time::{Mono, Instant},
        WCET_THRESHOLD,
    };
    #[cfg(any(
        feature = "delay-until",
        feature = "isr-switch",
        feature = "signal-rtic-sync",
        feature = "task-semaphore",
    ))]
    use core::mem::MaybeUninit;
    use rtic_monotonics::{
        fugit::{
            RateExtU32 as _,
            ExtU64 as _,
        }, 
        systick::prelude::*};
    use cortex_m::peripheral::DWT;
    use stm32f4xx_hal::{
        dwt::{
            Dwt, 
            StopWatch,
        }, interrupt, pac::NVIC, rcc::RccExt
    };
    #[cfg(feature = "delay-until")]
    use stm32f4xx_hal::dwt::DwtExt;
    use rtic_sync::{
        signal::{
            SignalReader,
            SignalWriter,
        },
        make_signal,
    };

    type ProfilingDuration = fugit::NanosDurationU64;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        // ISR-Switch
        rise_interrupt_dwt: Option<&'static DWT>,
        next_time: Option<Instant>,
        
        isr_dwt: Option<&'static DWT>,
        isr_switch_activation_count: u32,
        switch_cycles: u32,
        time_ns: f32,
        wc_isr_switch: f32,
        isr_hclk_mhz: f32,

        // Delay_until
        delay_until_activation_count: u32,
        delay_interval: u32, 
        delay_until_stopwatch: Option<StopWatch<'static>>,
        delay_until_overhead: Option<ProfilingDuration>, 
        wc_delay_until_overhead: Option<ProfilingDuration>,

        // Signal rtic_sync
        signal_writer: SignalWriter<'static, ()>,
        signal_writer_dwt: Option<&'static DWT>,

        signal_reader: SignalReader<'static, ()>,
        signal_reader_dwt: Option<&'static DWT>,
        signal_reader_cycles: u32,
        signal_reader_time: f32,
        wc_signal_rtic_sync: f32,
        signal_reader_hclk_mhz: f32,
        signal_reader_activation_count: u32,

        // Task Semaphore
        task_semaphore_waiter: TaskSemaphoreWaiter<'static>,
        task_semaphore_waiter_dwt: Option<&'static DWT>,

        task_semaphore_waiter_cycles: u32,
        task_semaphore_waiter_hclk_mhz: f32,
        task_semaphore_waiter_time: f32,
        wc_task_semaphore_waiter: f32,
        task_semaphore_waiter_activation_count: u32,
        task_semaphore_signaler: TaskSemaphoreSignaler<'static>,
        task_semaphore_signaler_dwt: Option<&'static DWT>,
    }

    #[init(local = [
        #[cfg(any(
            feature = "isr-switch",
            feature = "signal-rtic-sync",
            feature = "task-semaphore",
        ))]
        dwt_storage: MaybeUninit<DWT> = MaybeUninit::uninit(),
        #[cfg(feature = "delay-until")]
        dwt_hal_storage: MaybeUninit<Dwt> = MaybeUninit::uninit(),
        #[cfg(feature = "delay-until")]
        delay_until_times: [u32; 2] = [0; 2],
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Init");

        // Extract device from context
        let peripherals = cx.device;
        #[allow(unused_mut)]
        let mut core = cx.core;

        // Clocks setup
        let rcc = peripherals.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            .pclk1(42.MHz())
            .freeze();

        // Setup monotonic timer
        Mono::start(core.SYST, clocks.sysclk().to_Hz());
        
        // HCLK setup
        let hclk_mhz = clocks.hclk().to_MHz() as f32;

        // DWT setup
        #[allow(unused_assignments, unused_mut)]
        let mut dwt_ref: Option<&'static DWT> = None;
        #[cfg(feature = "isr-switch")]
        {
            dwt_ref = Some( 
                unsafe { 
                    core.DCB.enable_trace();
                    core.DWT.enable_cycle_counter();
                    cx.local.dwt_storage.write(core.DWT);
                    cx.local.dwt_storage.assume_init_ref()
                }
            );
        }

        // Dwt HAL setup (provides stopwatch)
        #[allow(unused_variables)]
        let hal_dwt: Option<&'static Dwt>;
        #[cfg(feature = "delay-until")]
        {
            hal_dwt = Some(
                unsafe {
                    let dwt = core.DWT.constrain(core.DCB, &clocks);
                    cx.local.dwt_hal_storage.write(dwt);
                    cx.local.dwt_hal_storage.assume_init_ref()
                }
            );  
        }

        // ISR-Switch profiling setup
        #[cfg(feature = "isr-switch")] 
        rise_interrupt::spawn()
            .expect("Error spawning interrupt generator");

        // Delay_until stopwatch and profiling setup
        #[allow(unused_assignments, unused_mut)]
        let mut delay_until_stopwatch: Option<StopWatch<'static>> = None;
        #[cfg(feature = "delay-until")]
        {
            delay_until_stopwatch = Some(hal_dwt.unwrap().stopwatch(cx.local.delay_until_times));

            delay_until_profiling::spawn()
                .expect("Error spawning delay_until task");
        }

        // Signal rtic_sync setup
        let (signal_writer, signal_reader) = make_signal!(());
        #[cfg(feature = "signal-rtic-sync")]
        {
            signal_writer_task::spawn()
                .expect("Error spawning signal writer task");
            signal_reader_task::spawn()
                .expect("Error spawning signal reader task");
        }

        // Task Semaphore setup
        let (watchdog_signal_writer, _watchdog_signal_reader) = make_signal!(Instant);
        let (task_semaphore_waiter, task_semaphore_signaler) = TaskSemaphore::init(
            watchdog_signal_writer,
        );
        #[cfg(feature = "task-semaphore")]
        {
            task_seamaphore_signaler_task::spawn()
                .expect("Error spawning task semaphore signaler task");
            task_semaphore_waiter_task::spawn()
                .expect("Error spawning task semaphore waiter task");
        }

        (
            Shared {},
            Local {
                // ISR-Switch
                rise_interrupt_dwt: dwt_ref,
                next_time: None,

                isr_dwt: dwt_ref,
                isr_switch_activation_count: 0,
                switch_cycles: 0,
                time_ns: 0.0, 
                wc_isr_switch: 0.0,
                isr_hclk_mhz: hclk_mhz,
    
                // Delay_until
                delay_until_activation_count: 0,
                delay_interval: 10,
                delay_until_stopwatch, 
                delay_until_overhead: None,
                wc_delay_until_overhead: None,

                // Signal rtic_sync
                signal_writer,
                signal_writer_dwt: dwt_ref,

                signal_reader,
                signal_reader_dwt: dwt_ref,
                signal_reader_cycles: 0,
                signal_reader_time: 0.0,
                wc_signal_rtic_sync: 0.0,
                signal_reader_hclk_mhz: 0.0,
                signal_reader_activation_count: 0,

                // Task Semaphore
                task_semaphore_waiter,
                task_semaphore_waiter_dwt: dwt_ref,

                task_semaphore_waiter_cycles: 0,
                task_semaphore_waiter_hclk_mhz: hclk_mhz,
                task_semaphore_waiter_time: 0.0,
                wc_task_semaphore_waiter: 0.0,
                task_semaphore_waiter_activation_count: 0,
                task_semaphore_signaler,
                task_semaphore_signaler_dwt: dwt_ref,
            }
        )
    }

    #[task(priority = 1, local=[rise_interrupt_dwt, next_time])]
    async fn rise_interrupt(cx: rise_interrupt::Context) -> ! {
        defmt::info!("Start of isr-switch profiling.");
        unsafe { NVIC::unmask(interrupt::EXTI0) };
        loop {
            *cx.local.next_time = Some(Mono::now() + (1 as u32).secs());
            
            critical_section::with(|_cs| {
                NVIC::pend(interrupt::EXTI0);
                unsafe{ cx.local.rise_interrupt_dwt.unwrap().cyccnt.write(0) };  
            });

            Mono::delay_until(cx.local.next_time.unwrap()).await;
        }
    }
    

    #[task(binds = EXTI0, local = [isr_dwt, isr_switch_activation_count, switch_cycles, isr_hclk_mhz, time_ns, wc_isr_switch])]
    fn exti0_isr(cx: exti0_isr::Context) {        
        *cx.local.switch_cycles = cx.local.isr_dwt.unwrap().cyccnt.read();
        *cx.local.time_ns = (*cx.local.switch_cycles as f32 / *cx.local.isr_hclk_mhz) * 1000.0;
        defmt::info!("ISR switch time: {} ns (number of cycles: {})", *cx.local.time_ns as u32, *cx.local.switch_cycles);
        defmt::info!("--------------------------------------------");

        // Update the wc_isr_switch
        *cx.local.wc_isr_switch = (*cx.local.wc_isr_switch).max(*cx.local.time_ns);

        *cx.local.isr_switch_activation_count += 1;
        if *cx.local.isr_switch_activation_count == WCET_THRESHOLD {
            defmt::info!("WC ISR switch time: {} ns", *cx.local.wc_isr_switch as u32);
            defmt::panic!("End of isr-switch profiling.");
        }
    } 

    #[task(priority = 1, local =[delay_until_activation_count, delay_until_stopwatch, delay_interval, delay_until_overhead, wc_delay_until_overhead])]
    async fn delay_until_profiling(cx: delay_until_profiling::Context) -> ! {
        loop {
            if let Some(stopwatch) = cx.local.delay_until_stopwatch.as_mut() {
                stopwatch.reset();
                Mono::delay_until(Mono::now() + cx.local.delay_interval.nanos()).await;
                stopwatch.lap();

                *cx.local.delay_until_overhead = Some(
                    (stopwatch.lap_time(1).unwrap().as_nanos() - *cx.local.delay_interval as u64).nanos()
                );

                defmt::info!("Delay_until overhead: {} ns", cx.local.delay_until_overhead.unwrap().to_nanos() as u32);

                defmt::info!("--------------------------------------------");

                // Update the wc_delay_until_overhead
                *cx.local.wc_delay_until_overhead = Some(cx.local.wc_delay_until_overhead.unwrap_or((0 as u64).nanos()).max(cx.local.delay_until_overhead.unwrap()));

                *cx.local.delay_until_activation_count += 1;
                if *cx.local.delay_until_activation_count == WCET_THRESHOLD {
                    defmt::info!("WC Delay_until overhead: {} ns", cx.local.wc_delay_until_overhead.unwrap_or((0 as u64).nanos()).to_nanos() as u32);
                    defmt::panic!("End of delay until profiling.");
                }
            } else {
                defmt::panic!("Stopwatch not initialized.");
            }
        }
    }

    #[task(priority = 2, local = [signal_writer, signal_writer_dwt])]
    async fn signal_writer_task(cx: signal_writer_task::Context) -> ! {
        loop {
            critical_section::with( |_cs| {
                cx.local.signal_writer.write(());
                unsafe{ cx.local.signal_writer_dwt.unwrap().cyccnt.write(0) };
            });

            Mono::delay((1 as u32).secs()).await;
        }
    }

    #[task(priority = 1, local = [signal_reader, signal_reader_dwt, signal_reader_cycles, signal_reader_hclk_mhz, signal_reader_time, wc_signal_rtic_sync, signal_reader_activation_count])]
    async fn signal_reader_task(cx: signal_reader_task::Context) -> ! {
        loop {
            cx.local.signal_reader.wait().await;
            *cx.local.signal_reader_cycles = cx.local.signal_reader_dwt.unwrap().cyccnt.read();

            *cx.local.signal_reader_time = (*cx.local.signal_reader_cycles as f32 /  *cx.local.signal_reader_hclk_mhz) * 1000.0;
            defmt::info!("Signal RTIC sync time: {} ns (number of cycles: {})", *cx.local.signal_reader_time as u32, *cx.local.signal_reader_cycles);
            defmt::info!("---------------------------------------------------");

            // Update the wc_signal_rtic_sync
            *cx.local.wc_signal_rtic_sync = (*cx.local.wc_signal_rtic_sync).max(*cx.local.signal_reader_time);

            *cx.local.signal_reader_activation_count += 1;
            if *cx.local.signal_reader_activation_count == WCET_THRESHOLD {
                defmt::info!("WC signal RTIC sync time: {} ns", *cx.local.wc_signal_rtic_sync as u32);
                defmt::panic!("End of signal rttc_sync profiling.");
            }
        }
    }

    #[task(priority =2, local = [task_semaphore_signaler, task_semaphore_signaler_dwt])]
    async fn task_seamaphore_signaler_task(cx: task_seamaphore_signaler_task::Context) -> ! {
        loop {
            critical_section::with( |_cs| {
                cx.local.task_semaphore_signaler.signal();
                // unsafe{ cx.local.task_semaphore_signaler_dwt.unwrap().cyccnt.write(0) };
            });

            Mono::delay((1 as u32).secs()).await;
        }
    }

    #[task(priority = 1, local = [task_semaphore_waiter, task_semaphore_waiter_dwt, task_semaphore_waiter_cycles, task_semaphore_waiter_hclk_mhz, task_semaphore_waiter_time, wc_task_semaphore_waiter, task_semaphore_waiter_activation_count])]
    async fn task_semaphore_waiter_task(cx: task_semaphore_waiter_task::Context) -> ! {
        loop {
            cx.local.task_semaphore_waiter.wait().await;
            *cx.local.task_semaphore_waiter_cycles = cx.local.task_semaphore_waiter_dwt.unwrap().cyccnt.read();

            *cx.local.task_semaphore_waiter_time = (*cx.local.task_semaphore_waiter_cycles as f32 /  *cx.local.task_semaphore_waiter_hclk_mhz) * 1000.0;
            defmt::info!("Task Semaphore wait time: {} ns (number of cycles: {})", *cx.local.task_semaphore_waiter_time as u32, *cx.local.task_semaphore_waiter_cycles);
            defmt::info!("---------------------------------------------------");

            Update the wc_task_semaphore_waiter
            *cx.local.wc_task_semaphore_waiter = (*cx.local.wc_task_semaphore_waiter).max(*cx.local.task_semaphore_waiter_time);

            *cx.local.task_semaphore_waiter_activation_count += 1;
            if *cx.local.task_semaphore_waiter_activation_count == WCET_THRESHOLD {
                defmt::info!("WC task semaphore wait time: {} ns", *cx.local.wc_task_semaphore_waiter as u32);
                defmt::panic!("End of task semaphore waiter profiling.");
            }
        }
    }

}
