# Profiling-RTIC

This project is a simple example of how to measure the context switch time of an interrupt service routine (ISR) in a Real-Time Interrupt-driven Concurrency (RTIC) application running on an STM32F4 microcontroller. The context switch time is measured using the Data Watchpoint and Trace (DWT) unit available in ARM Cortex-M processors.


First setup the proper environment variables depending on whether you want to run it on QEMU or on the actual board. It is only required to set the `CARGO_TARGET_MODE` variable in `.cargo/config.toml` to either `board` or `qemu`.

To compile the project with debug config:
```
cargo build --features semihosting|rtt
```

To run the project:
```
cargo run --features semihosting|rtt
```

The runner is set up to either launch a QEMU instance that prints to the host via semihosting, with `defmt-print` decoding and printing defmt logs; or to use `probe-rs` to flash and run the executable on the board.

Real hardware is required to make the DWT cycle counter work (otherwise, QEMU will always return 0).