#![no_std]
#![no_main]

mod event_queue;
mod task_semaphore;
mod time;

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
        event_queue::{EventQueueSignaler, EventQueueWaiter, EventQueue},
        task_semaphore::{TaskSemaphoreSignaler, TaskSemaphoreWaiter, TaskSemaphore},
        time::{
            Mono, 
            Instant, 
            set_hclk_mhz,
            set_dwt_ref,
        },
        WCET_THRESHOLD,
    };
    use core::mem::MaybeUninit;
    use cortex_m::peripheral::DWT;
    use stm32f4xx_hal::{
        interrupt, 
        pac::NVIC, 
        rcc::RccExt,
    };
    use rtic_monotonics::{
        fugit::{
            RateExtU32 as _,
        }, 
        systick::prelude::*};
    use rtic_sync::{
        signal::{
            SignalReader,
            SignalWriter,
        },
        make_signal,
    };

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        // ISR-Switch
        rise_interrupt_dwt: &'static DWT,
        next_time: Option<Instant>,
        
        isr_dwt: &'static DWT,
        isr_switch_activation_count: u32,
        switch_cycles: u32,
        time_ns: f32,
        wc_isr_switch: f32,
        isr_hclk_mhz: f32,

        // Delay_until
        delay_until_dwt: &'static DWT,
        delay_until_hclk_mhz: f32,
        delay_until_activation_count: u32,
        delay_interval: u32, 
        delay_until_cycles: u32,
        delay_until_overhead: f32, 
        wc_delay_until_overhead: f32,

        // Signal rtic_sync
        signal_writer: SignalWriter<'static, ()>,
        signal_writer_dwt: &'static DWT,

        signal_reader: SignalReader<'static, ()>,
        signal_reader_dwt: &'static DWT,
        signal_reader_cycles: u32,
        signal_reader_time: f32,
        wc_signal_rtic_sync: f32,
        signal_reader_hclk_mhz: f32,
        signal_reader_activation_count: u32,

        // TaskSemaphore
        task_semaphore_waiter: TaskSemaphoreWaiter<'static>,
        task_semaphore_waiter_dwt: &'static DWT,

        task_semaphore_waiter_cycles: u32,
        task_semaphore_waiter_hclk_mhz: f32,
        task_semaphore_waiter_time: f32,
        wc_task_semaphore_waiter: f32,
        task_semaphore_waiter_activation_count: u32,
        task_semaphore_signaler: TaskSemaphoreSignaler<'static>,
        task_semaphore_signaler_dwt: &'static DWT,

        // EventQueue
        event_queue_waiter: EventQueueWaiter<'static>,
        event_queue_waiter_dwt: &'static DWT,

        event_queue_waiter_cycles: u32,
        event_queue_waiter_hclk_mhz: f32,
        event_queue_waiter_time: f32,
        wc_event_queue_waiter: f32,
        event_queue_waiter_activation_count: u32,
        event_queue_signaler: EventQueueSignaler<'static>,
        event_queue_signaler_dwt: &'static DWT,
    }

    #[init(local = [
        dwt_storage: MaybeUninit<DWT> = MaybeUninit::uninit(),
    ])]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("Init");

        // Extract device from context
        let peripherals = cx.device;
        let mut core = cx.core;

        // Clocks setup
        let rcc = peripherals.RCC.constrain();
        let clocks = rcc
            .cfgr
            .use_hse(8.MHz())
            .sysclk(168.MHz())
            .pclk1(42.MHz())
            .freeze();

        // HCLK setup
        let hclk_mhz = clocks.hclk().to_MHz() as f32;
        #[cfg(feature = "systick")]
        set_hclk_mhz(hclk_mhz);

        // DWT setup
        let dwt_ref: &'static DWT = 
            unsafe { 
                core.DCB.enable_trace();
                core.DWT.enable_cycle_counter();
                cx.local.dwt_storage.write(core.DWT);
                cx.local.dwt_storage.assume_init_ref()
            };
        #[cfg(feature = "systick")]
        set_dwt_ref(dwt_ref);

        // Setup monotonic timer
        Mono::start(core.SYST, clocks.sysclk().to_Hz());

        // ISR-Switch profiling setup
        #[cfg(feature = "isr-switch")] 
        rise_interrupt::spawn()
            .expect("Error spawning interrupt generator");

        // Delay_until profiling setup
        #[cfg(feature = "delay-until")]
        delay_until_profiling::spawn()
            .expect("Error spawning delay_until task");

        // Signal rtic_sync setup
        let (signal_writer, signal_reader) = make_signal!(());
        #[cfg(feature = "signal-rtic-sync")]
        {
            signal_writer_task::spawn()
                .expect("Error spawning signal writer task");
            signal_reader_task::spawn()
                .expect("Error spawning signal reader task");
        }

        // Fake watchdog signal for the other synchronization primitives
        let (watchdog_signal_writer, _watchdog_signal_reader) = make_signal!(Instant);

        // Task Semaphore setup
        let (task_semaphore_waiter, task_semaphore_signaler) = TaskSemaphore::init(
            watchdog_signal_writer.clone(),
        );
        #[cfg(feature = "task-semaphore")]
        {
            task_seamaphore_signaler_task::spawn()
                .expect("Error spawning task semaphore signaler task");
            task_semaphore_waiter_task::spawn()
                .expect("Error spawning task semaphore waiter task");
        }

        // Event Queue setup
        let (event_queue_waiter, event_queue_signaler) = EventQueue::init(
            watchdog_signal_writer.clone(),
        );
        #[cfg(feature = "event-queue")]
        {
            event_queue_signaler_task::spawn()
                .expect("Error spawning event queue signaler task");
            event_queue_waiter_task::spawn()
                .expect("Error spawning event queue waiter task");
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
                delay_until_dwt: dwt_ref,
                delay_until_hclk_mhz: hclk_mhz,
                delay_until_activation_count: 0,
                delay_interval: 10, 
                delay_until_cycles: 0,
                delay_until_overhead: 0.0,
                wc_delay_until_overhead: 0.0,

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

                // TaskSemaphore
                task_semaphore_waiter,
                task_semaphore_waiter_dwt: dwt_ref,

                task_semaphore_waiter_cycles: 0,
                task_semaphore_waiter_hclk_mhz: hclk_mhz,
                task_semaphore_waiter_time: 0.0,
                wc_task_semaphore_waiter: 0.0,
                task_semaphore_waiter_activation_count: 0,
                task_semaphore_signaler,
                task_semaphore_signaler_dwt: dwt_ref,

                // EventQueue
                event_queue_waiter,
                event_queue_waiter_dwt: dwt_ref,

                event_queue_waiter_cycles: 0,
                event_queue_waiter_hclk_mhz: hclk_mhz,
                event_queue_waiter_time: 0.0,
                wc_event_queue_waiter: 0.0,
                event_queue_waiter_activation_count: 0,
                event_queue_signaler,
                event_queue_signaler_dwt: dwt_ref,
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
                unsafe{ cx.local.rise_interrupt_dwt.cyccnt.write(0) };  
            });

            Mono::delay_until(cx.local.next_time.unwrap()).await;
        }
    }
    

    #[task(binds = EXTI0, local = [isr_dwt, isr_switch_activation_count, switch_cycles, isr_hclk_mhz, time_ns, wc_isr_switch])]
    fn exti0_isr(cx: exti0_isr::Context) {        
        *cx.local.switch_cycles = cx.local.isr_dwt.cyccnt.read();
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

    #[task(priority = 1, local =[delay_until_dwt, delay_until_hclk_mhz, delay_until_activation_count, delay_interval, delay_until_cycles, delay_until_overhead, wc_delay_until_overhead])]
    async fn delay_until_profiling(cx: delay_until_profiling::Context) -> ! {
        loop {
            unsafe { cx.local.delay_until_dwt.cyccnt.write(0) };
            Mono::delay_until(Mono::now() + cx.local.delay_interval.nanos()).await;
            *cx.local.delay_until_cycles = cx.local.delay_until_dwt.cyccnt.read();

            *cx.local.delay_until_overhead = 
                (*cx.local.delay_until_cycles as f32 /  *cx.local.delay_until_hclk_mhz) * 1000.0 // tot delay_until time in ns
                - (*cx.local.delay_interval as f32);                                             // - delay interval in ns = overhead 

            defmt::info!("Delay_until overhead: {} ns", *cx.local.delay_until_overhead as u32);
            defmt::info!("--------------------------------------------");

            // Update the wc_delay_until_overhead
            *cx.local.wc_delay_until_overhead = (*cx.local.wc_delay_until_overhead).max(*cx.local.delay_until_overhead);

            *cx.local.delay_until_activation_count += 1;
            if *cx.local.delay_until_activation_count == WCET_THRESHOLD {
                defmt::info!("WC Delay_until overhead: {} ns", *cx.local.wc_delay_until_overhead as u32);
                defmt::panic!("End of delay until profiling.");
            }            
        }
    }

    #[task(priority = 2, local = [signal_writer, signal_writer_dwt])]
    async fn signal_writer_task(cx: signal_writer_task::Context) -> ! {
        loop {
            critical_section::with( |_cs| {
                cx.local.signal_writer.write(());
                unsafe{ cx.local.signal_writer_dwt.cyccnt.write(0) };
            });

            Mono::delay((1 as u32).secs()).await;
        }
    }

    #[task(priority = 1, local = [signal_reader, signal_reader_dwt, signal_reader_cycles, signal_reader_hclk_mhz, signal_reader_time, wc_signal_rtic_sync, signal_reader_activation_count])]
    async fn signal_reader_task(cx: signal_reader_task::Context) -> ! {
        loop {
            cx.local.signal_reader.wait().await;
            *cx.local.signal_reader_cycles = cx.local.signal_reader_dwt.cyccnt.read();

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
                unsafe{ cx.local.task_semaphore_signaler_dwt.cyccnt.write(0) };
            });

            Mono::delay((1 as u32).secs()).await;
        }
    }

    #[task(priority = 1, local = [task_semaphore_waiter, task_semaphore_waiter_dwt, task_semaphore_waiter_cycles, task_semaphore_waiter_hclk_mhz, task_semaphore_waiter_time, wc_task_semaphore_waiter, task_semaphore_waiter_activation_count])]
    async fn task_semaphore_waiter_task(cx: task_semaphore_waiter_task::Context) -> ! {
        loop {
            cx.local.task_semaphore_waiter.wait().await;
            *cx.local.task_semaphore_waiter_cycles = cx.local.task_semaphore_waiter_dwt.cyccnt.read();

            *cx.local.task_semaphore_waiter_time = (*cx.local.task_semaphore_waiter_cycles as f32 /  *cx.local.task_semaphore_waiter_hclk_mhz) * 1000.0;
            defmt::info!("Task Semaphore wait time: {} ns (number of cycles: {})", *cx.local.task_semaphore_waiter_time as u32, *cx.local.task_semaphore_waiter_cycles);
            defmt::info!("---------------------------------------------------");

            // Update the wc_task_semaphore_waiter
            *cx.local.wc_task_semaphore_waiter = (*cx.local.wc_task_semaphore_waiter).max(*cx.local.task_semaphore_waiter_time);

            *cx.local.task_semaphore_waiter_activation_count += 1;
            if *cx.local.task_semaphore_waiter_activation_count == WCET_THRESHOLD {
                defmt::info!("WC task semaphore wait time: {} ns", *cx.local.wc_task_semaphore_waiter as u32);
                defmt::panic!("End of task semaphore waiter profiling.");
            }
        }
    }

    #[task(priority =2, local = [event_queue_signaler, event_queue_signaler_dwt])]
    async fn event_queue_signaler_task(cx: event_queue_signaler_task::Context) -> ! {
        loop {
            critical_section::with( |_cs| {
                cx.local.event_queue_signaler.signal(());
                unsafe{ cx.local.event_queue_signaler_dwt.cyccnt.write(0) };
            });

            Mono::delay((1 as u32).secs()).await;
        }
    }

    #[task(priority = 1, local = [event_queue_waiter, event_queue_waiter_dwt, event_queue_waiter_cycles, event_queue_waiter_hclk_mhz, event_queue_waiter_time, wc_event_queue_waiter, event_queue_waiter_activation_count])]
    async fn event_queue_waiter_task(cx: event_queue_waiter_task::Context) -> ! {
        loop {
            cx.local.event_queue_waiter.wait().await;
            *cx.local.event_queue_waiter_cycles = cx.local.event_queue_waiter_dwt.cyccnt.read();

            *cx.local.event_queue_waiter_time = (*cx.local.event_queue_waiter_cycles as f32 /  *cx.local.event_queue_waiter_hclk_mhz) * 1000.0;
            defmt::info!("Event Queue wait time: {} ns (number of cycles: {})", *cx.local.event_queue_waiter_time as u32, *cx.local.event_queue_waiter_cycles);
            defmt::info!("---------------------------------------------------");

            // Update the wc_event_queue_waiter
            *cx.local.wc_event_queue_waiter = (*cx.local.wc_event_queue_waiter).max(*cx.local.event_queue_waiter_time);

            *cx.local.event_queue_waiter_activation_count += 1;
            if *cx.local.event_queue_waiter_activation_count == WCET_THRESHOLD {
                defmt::info!("WC event queue wait time: {} ns", *cx.local.wc_event_queue_waiter as u32);
                defmt::panic!("End of event queue waiter profiling.");
            }
        }
    }
}
