# rocket-os

This crate provides the core "Operating System" abstractions for the `rocket_vR` firmware, specifically tailored for the RP2350 (Cortex-M33) dual-core architecture.

## Requirements

- **Architecture**: ARM Cortex-M with Event Register support (e.g., RP2350, RP2040).
- **Linker Configuration**: Requires a `.uninit` section in the linker script to support persistent panic records across reboots.
- **Hardware**: Uses RP-specific peripherals (`SIO`, `FIFO`, `CPUID`) for dual-core synchronization and crash detection.
- **Critical Section**: Requires a `critical-section` implementation (e.g., `embassy-rp`) for atomic operations.
- **Global Blackboard**: Depends on `rocket-core` for shared data structures and health types.

## System Engineering Requirements

Determining the success of the `rocket-os` layer involves verifying these core functional and performance requirements:

1.  **Deterministic Idle Monitoring**: The executor MUST accurately sample CPU idle cycles to provide utilization metrics with at least 0.1% resolution.
2.  **Dual-Core Isolation**: Wakeup signals MUST be targeted; an event on Core 1 MUST NOT cause a spurious wakeup/poll cycle on Core 0 if no tasks are pending.
3.  **Panic Persistence**: On a system crash (Panic or HardFault), the OS MUST preserve the failure location (file/line) and message in RAM that survives a warm reset.
4.  **Hardware-Level Alarms**: Crash detection MUST NOT rely on high-level software; it MUST use hardware primitives (SIO FIFO) to ensure core-to-core signaling even if the memory bus is congested.
5.  **Stack Safety**: The OS MUST provide a mechanism to "paint" and "scan" stack memory to detect high-watermarks for both cores during runtime.
6.  **Zero-Noise Idle**: At 0% load, the instrumented executor MUST NOT exceed 1% apparent CPU utilization due to internal overhead or spurious wakeups.

## Known Limitations

### Task-Level Over-Reporting
Individual task utilization (via `TrackedFuture`) currently includes time spent in high-priority interrupts that fire during a task's poll cycle.
- **Impact**: Task-level stats are slightly overstated when interrupts are frequent.
- **Accuracy**: Core-level totals (Core 0/1 Total, HP, BG) are unaffected as they use a separate differential calculation to subtract interrupt time.
- **Mitigation**: Future update to `TrackedFuture` will subtract `INTERRUPT_ACTIVE_TICKS` from individual poll cycles.

## Features

### 1. Instrumented Embassy Executor
A custom Embassy executor that solves the "spurious wakeup" and "ping-pong" problems unique to dual-core ARM Cortex-M systems.

```mermaid
sequenceDiagram
    participant P as __pender (Any Core)
    participant E as Instrumented Executor (Core 0/1)
    participant HW as ARM Event Register

    Note over P, E: Task Wakeup Triggered
    P->>E: sets wake = true
    P->>HW: SEV (Broadcast to ALL cores)
    HW-->>E: WFE returns (if sleeping)

    loop Run Loop
        E->>E: swap(wake, false)
        alt wake was true
            E->>E: inner.poll()
        end
        E->>E: 1st WFE (Drain stale event)
        E->>E: 2nd WFE (Real Sleep)
    end
```

- **Wake-Gate Logic**: Uses a software gate to filter out spurious wakeups caused by atomic operations in the run-queue.
- **Dual-Core Efficiency**: Replaces the broadcast `SEV` (Send Event) with a double `WFE` (Wait For Event) drain pattern, preventing Core 1 from accidentally waking Core 0 and vice versa.
- **Utilization Tracking**: Automatically tracks `idle_ticks` and `poll_count` per core for accurate CPU load monitoring.

### 2. Robust Panic Handling
A high-reliability panic and hard-fault handler designed for cross-core visibility.

```mermaid
sequenceDiagram
    participant C1 as Crashing Core
    participant RAM as Shared RAM (.uninit)
    participant FIFO as SIO FIFO (Hardware)
    participant C0 as Monitoring Core

    C1->>RAM: Write PanicRecord (File, Line, Msg)
    C1->>RAM: Set STATE_SIGNATURE_PARTIAL
    C1->>FIFO: Write STATE_SIGNATURE_PARTIAL
    FIFO-->>C0: Hardware interrupt / Flag set
    C0->>FIFO: Read Sentinel
    C0->>RAM: Extract CrashReport
    C0->>C0: Log to console (defmt)
    C1->>C1: loop {} (Hang)
```

- **Bidirectional Monitoring**: Core 1 monitors Core 0, and Core 0 monitors Core 1.
- **Hardware Signaling**: Uses the RP2350 SIO FIFO as a hardware-level sentinel for immediate crash detection.
- **Post-Mortem Logs**: Stores panic information (file, line, message) in `.uninit` RAM, allowing the system to report crash details after an automatic reboot.
- **`addr2line` Integration**: Includes instructions for resolving stack traces using the project binary.

### 3. System Health Monitoring
Background tasks for real-time diagnostics.

- **CPU Utilization**: Calculates total load, interrupt-mode load, and per-task load in tenths of a percent (0.1% resolution).
- **Stack Watermarking**: Monitors the high-watermark of the stack on both cores to prevent overflows.
- **Heartbeat Monitoring**: Tracks the liveness of critical subsystems (IMU, GPS, Radio).

## Usage

### Setting up the Executor

In your `main.rs`, define the executors for both cores:

```rust
static METRICS_C0: ExecutorMetrics = ExecutorMetrics::new();
static mut EXECUTOR_C0: InstrumentedExecutor = InstrumentedExecutor::new(&METRICS_C0);

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Core 0 logic...
    unsafe { EXECUTOR_C0.run(|spawner| { ... }) };
}
```

### Enabling Verbose Stats
Build with the `verbose-utilization` feature to see second-by-second reports in the console:

```powershell
cargo run-pico2 --features verbose-utilization
```
