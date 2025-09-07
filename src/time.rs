use rtic_monotonics::Monotonic;
use cortex_m::peripheral::DWT;

static mut HCLK_MHZ: f32 = 0.0;

pub fn set_hclk_mhz(hclk_mhz: f32) {
    unsafe {
        HCLK_MHZ = hclk_mhz;
    }
}

fn get_hclk_mhz() -> f32 {
    unsafe { HCLK_MHZ }
}

static mut DWT_REF: Option<&'static DWT> = None;

pub fn set_dwt_ref(dwt_ref: &'static DWT) {
    unsafe {
        DWT_REF = Some(dwt_ref);
    }
}

fn get_dwt_ref() -> &'static DWT {
    unsafe { DWT_REF.expect("DWT reference not set") }
}


#[cfg(not(feature = "systick"))]
rtic_monotonics::systick_monotonic!(Mono, 1_000);
#[cfg(feature = "systick")]
profiled_rtic_monotonics::systick_monotonic!(Mono, 1_000, get_hclk_mhz(), get_dwt_ref());

// defmt timestamp
defmt::timestamp!("{=u32:ms}", Mono::now().duration_since_epoch().to_millis());

#[cfg(not(feature = "systick"))]
pub type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

#[cfg(feature = "systick")]
pub type Instant = <Mono as profiled_rtic_monotonics::Monotonic>::Instant;
