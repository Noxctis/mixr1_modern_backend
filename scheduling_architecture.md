# MIXR-1 Scheduling Architecture

The daemon uses a periodic 10 ms control loop. The loop attempts to run with `SCHED_FIFO` priority 90 when launched with sufficient privileges. Timing is scheduled against an absolute `next_wake` deadline so small execution variations do not continuously shift the nominal loop schedule.

```mermaid
flowchart TD
    A[Daemon starts] --> B{Test mode?}
    B -- Yes --> C[Configure test options]
    B -- No --> D[Request SCHED_FIFO priority 90]
    C --> E{"--fifo requested?"}
    E -- Yes --> F[Request SCHED_FIFO priority 90]
    E -- No --> G[Keep normal scheduling policy]
    F --> H[Start pigpio connection]
    G --> H
    D --> I[Start telemetry server]
    H --> J[Configure encoder callbacks and motor PWM]
    J --> K[Initialize kinematics and PI controller]
    K --> L[Set initial PWM or target]
    L --> M["Set absolute next_wake = now"]

    I --> N[Wait for dashboard client]
    N --> O["Set initial next_wake = now"]
    O --> P[Periodic 10 ms loop]

    M --> Q[Periodic 10 ms test loop]

    P --> R["next_wake += 10 ms"]
    Q --> S["next_wake += 10 ms"]
    R --> T["sleep_until(next_wake)"]
    S --> U["sleep_until(next_wake)"]
    T --> V[Measure actual wake time and loop period]
    U --> W["Measure actual wake time, period, and lateness"]

    V --> X[Check Simulink status periodically]
    X --> Y[Receive target RPM command]
    Y --> Z[Read encoder count]
    Z --> AA[Calculate delta ticks and raw RPM]
    AA --> AB[EMA filtering and optional LCD average]
    AB --> AC{"Target RPM > 0?"}
    AC -- Yes --> AD["PI compute: error, integral, scheduled Kp/Ki"]
    AD --> AE["Clamp PWM to 0..4095"]
    AC -- No --> AF["Set PWM = 0"]
    AE --> AG[Write hardware PWM]
    AF --> AG
    AG --> AH[Send telemetry and update LCD]
    AH --> P

    W --> AI{"PWM sweep?"}
    AI -- Yes --> AJ["Select 0..100% PWM step"]
    AI -- No --> AK[Keep selected PWM]
    AJ --> AL[Read encoder count]
    AK --> AL
    AL --> AM[Calculate raw and filtered RPM]
    AM --> AN{"PI enabled?"}
    AN -- Yes --> AO[Use stepped RPM target]
    AO --> AP[PI compute and write hardware PWM]
    AN -- No --> AQ[Write fixed PWM]
    AP --> AR[Write CSV timing and RPM sample]
    AQ --> AR
    AR --> Q

    subgraph CallbackThreads[ pigpio callback context ]
        CA[Encoder A/B edge] --> CB[Quadrature state update]
        CB --> CC[Atomic encoder count]
        CX[Index rising edge] --> CY[Atomic revolution count]
    end

    CA -. asynchronous callback .-> CB
    CX -. asynchronous callback .-> CY
    CC -. read by .-> Z
    CY -. read by .-> AH
    CC -. read by .-> AL
```

## Scheduling Roles

| Component | Scheduling role | Data exchanged |
|---|---|---|
| Main daemon thread | Runs dashboard control loop every 10 ms | Target RPM, PWM, RPM telemetry |
| Test loop | Runs the same 10 ms cadence and logs timing | PWM/target steps, raw RPM, loop period, lateness |
| Encoder callbacks | Asynchronously process quadrature edges | Atomic encoder tick count |
| PI controller | Executes inside the main/test loop | RPM error and PWM command |
| pigpio hardware PWM | Generates the motor waveform independently | PWM duty cycle |
| Network and LCD work | Executes in the normal loop after control output | Dashboard/LCD updates |

## FIFO Comparison

The scheduling experiment compares the same test in two conditions:

- `--no-fifo`: normal Linux scheduling
- `--fifo`: main test thread requests `SCHED_FIFO` priority 90

The CSV timing fields are:

- `loop_period_us`: time between actual loop iterations
- `late_us`: time after the intended absolute wake deadline
- `raw_rpm`: encoder-derived RPM before display filtering

A useful presentation comparison is the standard deviation of `loop_period_us`, the maximum `late_us`, the count of late cycles, and the raw RPM standard deviation at each identical PWM level.

## Commands

Open-loop PWM characterization:

```bash
sudo ./mixr1_daemon --test --sweep --no-fifo --no-pi \
  --duration=10 --target=0 --csv=open_loop_no_fifo.csv

sudo ./mixr1_daemon --test --sweep --fifo --no-pi \
  --duration=10 --target=0 --csv=open_loop_fifo.csv
```

PI setpoint characterization:

```bash
sudo ./mixr1_daemon --test --sweep --no-fifo --pi \
  --target=2500 --duration=10 --csv=pi_no_fifo.csv

sudo ./mixr1_daemon --test --sweep --fifo --pi \
  --target=2500 --duration=10 --csv=pi_fifo.csv
```
