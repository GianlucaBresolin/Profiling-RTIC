# Profiling-RTIC

This repository provides a way to measure different types of overhead in a Real-Time Interrupt-driven Concurrency (RTIC) application running on an STM32F4 microcontroller.

The measurements are performed using the Data Watchpoint and Trace (DWT) unit available in ARM Cortex-M processors, which allows for precise cycle counting.

First setup the proper environment variables depending on whether you want to run it on QEMU or on the actual board. It is only required to set the `CARGO_TARGET_MODE` variable in `.cargo/config.toml` to either `board` or `qemu`.

To compile the project with debug config:
```
cargo build --features <semihosting|rtt>,<OVERHEAD>
```

To run the project:
```
cargo run --features <semihosting|rtt>,<OVERHEAD>
```

Where `<OVERHEAD>` can be *just one* of the following:

| Overhead           | Description | 
|--------------------|-----------|
| `systick`          | The overhead of the `SysTick` interrupt handler (system timer overhead) defined in `rtic_monotonics`, which is responsible for activating all timed events whose expiration time has already passed. |
| `isr-switch`       | The context switch time of an interrupt service routine (ISR), without taking into account the execution time of the ISR itself. |
| `delay-until`      | The overhead of the `delay_until` function provided by the `rtic_monotonics` timer, used to delay task execution until an absolute time. |
| `signal-rtic-sync` | The overhead of waiting on a `rtic_sync` crate's signal mechanism, which provides a way for tasks to synchronize with each other. |
| `task-semaphore`   | The overhead of waiting on a `TaskSemaphore`. | 
| `event-queue`      | The overhead of waiting on an `EventQueue`. |
| `spawn-overhead`   | The (best) overhead of spawning a task. |
| `context-switch`   | The context switch time between two tasks (this value also includes the spawn overhead of the preempting task). |

The runner is set up to either launch a QEMU instance that prints to the host via semihosting, with `defmt-print` decoding and printing defmt logs; or to use `probe-rs` to flash and run the executable on the board.

Real hardware is required to make the DWT cycle counter work (otherwise, QEMU will always return 0).