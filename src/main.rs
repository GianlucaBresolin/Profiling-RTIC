#![no_std]
#![no_main]

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

const WCET_THRESHOLD: u32 = 10;

#[panic_handler]
fn panic(info: &core::panic::PanicInfo) -> ! {
    interrupt::disable();

    defmt::error!("Panic: {}", info);
    debug::exit(EXIT_FAILURE);

    loop {}
}

#[rtic::app(
    device = stm32f4xx_hal::pac,
    dispatchers = [EXTI1],
)]
mod app {
    use crate::{
        time::{Mono, Instant},
        WCET_THRESHOLD,
    };
    #[cfg(any(
        feature = "delay-until",
        feature = "isr-switch",
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
    }

    #[init(local = [
        #[cfg(feature = "isr-switch")]
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
        // Delay_until stopwatch setup
        #[allow(unused_assignments, unused_mut)]
        let mut delay_until_stopwatch: Option<StopWatch<'static>> = None;
        #[cfg(feature = "delay-until")]
        {
            delay_until_stopwatch = Some(hal_dwt.unwrap().stopwatch(cx.local.delay_until_times));
        }

        // ISR-Switch profiling setup
        #[cfg(feature = "isr-switch")] 
        rise_interrupt::spawn()
            .expect("Error spawning interrupt generator");
        
        // Delay_until profiling setup
        #[cfg(feature = "delay-until")]
        delay_until_profiling::spawn()
            .expect("Error spawning delay_until task");

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
            }
        )
    }

    #[task(priority = 1, local=[rise_interrupt_dwt, next_time])]
    async fn rise_interrupt(cx: rise_interrupt::Context) {
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
    async fn delay_until_profiling(cx: delay_until_profiling::Context) {
        if let Some(stopwatch  ) = cx.local.delay_until_stopwatch.as_mut() {
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
