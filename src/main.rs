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
    dispatchers = [EXTI1],)]
mod app {
    use crate::{
        time::{Instant, Mono},
        WCET_THRESHOLD,
    };
    use core::mem::MaybeUninit;
    use rtic_monotonics::{fugit::RateExtU32 as _, systick::prelude::*};
    use cortex_m::peripheral::DWT;

    use stm32f4xx_hal::{interrupt, pac::NVIC, rcc::RccExt};

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        dwt_rise_interrupt: &'static DWT,
        activation_count: u32,
        next_time: Option<Instant>,
        
        dwt_isr: &'static DWT,
        switch_cycles: u32,
        time_ns: Option<f32>,
        wc_isr_switch: Option<f32>,
        hclk_mhz: f32,
    }

    #[init(local = [
        dwt_wrapper: MaybeUninit<DWT> = MaybeUninit::uninit(),
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

        // Setup monotonic timer
        Mono::start(core.SYST, clocks.sysclk().to_Hz());
        
        rise_interrupt::spawn()
            .expect("Error spawning interrupt generator");
        
        let hclk_mhz = clocks.hclk().to_MHz() as f32;
        let dwt_ref = unsafe { 
            core.DCB.enable_trace();
            core.DWT.enable_cycle_counter();
            cx.local.dwt_wrapper.write(core.DWT);
            cx.local.dwt_wrapper.assume_init_ref()
        };

        (
            Shared {},
            Local {
                dwt_rise_interrupt: dwt_ref, 
                activation_count: 0,
                next_time: None,

                dwt_isr : dwt_ref,
                switch_cycles: 0,
                time_ns: None, 
                wc_isr_switch: None,
                hclk_mhz,
            },
        )
    }

    #[task(priority = 1, local=[dwt_rise_interrupt, activation_count, next_time])]
    async fn rise_interrupt(cx: rise_interrupt::Context) {
        defmt::info!("Start of isr-switch profiling.");
        unsafe { NVIC::unmask(interrupt::EXTI0) };
        loop {
            *cx.local.next_time = Some(Mono::now() + 1.secs());
            *cx.local.activation_count += 1;
            
            critical_section::with(|_cs| {
                NVIC::pend(interrupt::EXTI0);
                unsafe{ cx.local.dwt_rise_interrupt.cyccnt.write(0) };  
            });
            if *cx.local.activation_count == WCET_THRESHOLD {
                defmt::panic!("End of isr-switch profiling.");
            }
            Mono::delay_until(cx.local.next_time.unwrap()).await;
        }
    }
    

    #[task(binds = EXTI0, local = [dwt_isr, switch_cycles, hclk_mhz, time_ns, wc_isr_switch])]
    fn exti0_isr(cx: exti0_isr::Context) {        
        *cx.local.switch_cycles = cx.local.dwt_isr.cyccnt.read();
        *cx.local.time_ns = Some((*cx.local.switch_cycles as f32 / *cx.local.hclk_mhz) * 1000.0);
        defmt::info!("ISR switch time: {} ns (number of cycles: {})", cx.local.time_ns.unwrap() as u32, *cx.local.switch_cycles);

        // Update the wc_isr_switch
        *cx.local.wc_isr_switch = Some(cx.local.wc_isr_switch.unwrap_or(0.0).max(cx.local.time_ns.unwrap()));
        defmt::info!("WC ISR switch time: {} ns", cx.local.wc_isr_switch.unwrap() as u32);

        defmt::info!("--------------------------------------------");
    } 
}
