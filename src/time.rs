use rtic_monotonics::Monotonic;

// Timer interrupt setup and timer type creation
// stm32_tim2_monotonic!(Mono, 1_000);

#[cfg(not(feature = "systick"))]
rtic_monotonics::systick_monotonic!(Mono, 1_000);
#[cfg(feature = "systick")]
profiled_rtic_monotonics::systick_monotonic!(Mono, 1_000);

// defmt timestamp
defmt::timestamp!("{=u32:ms}", Mono::now().duration_since_epoch().to_millis());

#[cfg(not(feature = "systick"))]
pub type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

#[cfg(feature = "systick")]
pub type Instant = <Mono as profiled_rtic_monotonics::Monotonic>::Instant;
