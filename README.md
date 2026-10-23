# Arduino UNO Q: ROS 2 maximum-throughput test

An Arduino App Lab app derived from [the BMI088 integration project](https://github.com/mauringo/bosch-sensortec-ros2-unoq). It replaces sensor acquisition with dummy data to measure the throughput of the same **STM32 → RouterBridge → Python → ROS 2 Jazzy → subscriber** path. No external sensor or wiring is needed.

The MCU returns the same eight-number MessagePack array shape as `read_imu_fast` in the BMI088 app. ROS publishes standard `sensor_msgs/msg/Imu`, with orientation unavailable, to `/benchmark/imu`. The first dummy acceleration axis carries a host publication sequence number for loss detection. **These are test values; do not use this topic as real IMU data.**

## Why this repository exists

The [original BMI088 repository](https://github.com/mauringo/bosch-sensortec-ros2-unoq) demonstrated a working sensor-to-ROS 2 pipeline, with approximately 65–68 Hz received against a 100 Hz target in its recorded example. Before using that approach for navigation or balancing, we need to understand where the time goes.

This companion project removes sensor reads and keeps the original compact RPC response shape. It compares the complete MCU/RouterBridge/ROS path with a ROS-only baseline, measures delivery at an independent subscriber, and preserves results for review. Its purpose is to distinguish sensor work from communication overhead and guide the division of work between UNO Q's STM32 and Linux processors.

The engineering question is: **can this implementation deliver timely IMU data, and which tasks should stay on the MCU?** Throughput is one part of that answer. Timestamp accuracy, jitter, calibration and closed-loop behavior require separate tests.

## Recorded hardware results

The [recorded run and assessment](benchmarks/20261023T202925.325545Z/analysis.md) include the supplied raw log, 66 parsed stages and an explanation of the implications for navigation and balancing.

| Measurement | Result across three repeats |
| --- | --- |
| Highest passing tested Bridge target | **75 Hz**, received 74.9–75.0 Hz, reported loss 0.00% |
| Bridge unthrottled saturation | **93.5–93.9 Hz** received |
| Highest passing tested ROS-only target | **400 Hz**, received 399.9–400.0 Hz, reported loss 0.00% |
| ROS-only at 800 Hz offered | 481.7–487.7 Hz received, 37.79–38.54% reported loss |

These are observations of this software path on one setup. Rates between 75 and 100 Hz (Bridge) and 400 and 800 Hz (ROS-only) were not tested. Rounded console values do not include RPC percentiles, exact counts or acquisition latency.

**Engineering assessment:** UNO Q's split architecture is promising when the STM32 handles acquisition and fast feedback while ROS handles navigation and supervision. This request/reply implementation has not demonstrated the bounded timing needed for a balancing loop through Linux. Dummy-message delivery also cannot establish IMU measurement quality. See the [full analysis and proposed next tests](benchmarks/20261023T202925.325545Z/analysis.md).

## Run in Arduino App Lab

1. Download and import [UNO_Q_ROS2_SPEED_TEST.zip](https://github.com/mauringo/uno-q-ros2-imu-benchmark/raw/refs/heads/main/UNO_Q_ROS2_SPEED_TEST.zip), or create an app and copy `app.yaml`, `sketch/`, `python/`, `bricks/`, `results/` and `UNO_Q_SPEED_APP.marker` into its directory. Connect your **UNO Q**.
2. Stop other sensor/ROS apps using RouterBridge, then **Deploy/Run**. The first run builds the ROS Jazzy container and needs internet access and storage.
3. The Python pane prints the log command. Run this in the **UNO Q Linux board shell**:

```bash
find "$HOME/ArduinoApps" -name UNO_Q_SPEED_APP.marker -execdir docker compose -f .cache/app-compose.yaml logs -f ros2 \; -quit
```

The finite test runs automatically. Defaults: targets **10, 25, 50, 75, 100, 150, 200, 400, 800, 1600 Hz**, followed by **unthrottled**, with **three repeats** for each of the Bridge and ROS-only modes. Each stage warms up for 1 s, measures for 10 s, then drains for 1 s. Allow roughly **14 minutes after startup**. The ROS Brick exits when finished; the idle App Lab Python entry point keeps the app open.

Results persist under `results/<UTC run ID>/` in the deployed app, outside the container:

- `stages.csv`: one row per completed stage, saved immediately.
- `summary.json`: settings, platform information, stage measurements, completion status, and the best sustainable tested target for each mode.

Use the deployed app directory containing the marker when reading results. Running from the Mac source directory does not read files on the board. With multiple app copies, the log command selects the first match.

## Run a shorter test or refine the ceiling

From the **deployed app directory on UNO Q**, after the automatic benchmark has finished:

```bash
docker compose -f .cache/app-compose.yaml run --rm --no-deps ros2 bash -c 'source /opt/ros/jazzy/setup.bash && exec python3 -u /app/benchmark.py --rates 50,100,200,0 --duration 5 --repeats 1 --modes bridge'
```

To refine a ceiling between 50 and 100 Hz, for example:

```bash
docker compose -f .cache/app-compose.yaml run --rm --no-deps ros2 bash -c 'source /opt/ros/jazzy/setup.bash && exec python3 -u /app/benchmark.py --rates 60,65,70,75,80,85,90,95,100,0 --duration 30 --repeats 3 --modes bridge'
```

Do not run two benchmarks at once: they share the topic and MCU. Set the same `ROS_DOMAIN_ID` for publisher and receiver; choose an unused domain in `brick_compose.yaml` if other ROS applications are present.

Other options: `--modes ros-only`, `--reliable`, `--depth 100`, `--rpc-timeout 0.5`, `--warmup 1`, `--drain 1`, `--rate-fraction 0.95`, `--max-loss 0.01`, `--output /results`. Use `--help` for all options. To change automatic defaults, edit the command in `brick_compose.yaml`, then Stop + Deploy/Run. Source edits also require Stop + Deploy/Run; Dockerfile changes require rebuilding the image.

## Interpret the result

| Field | Meaning |
| --- | --- |
| `target_hz` | Requested pacing; **0 means no deliberate sleep** |
| `sent_hz` | Successful ROS publications divided by measurement duration plus any final blocking-call overrun |
| `received_hz` | Unique subscriber callbacks inside the fixed measurement window, divided by its duration |
| `received` | Unique messages delivered, including the drain interval |
| `loss_fraction` | Successful publications not received by the drain deadline, divided by publications |
| `rpc_p50_ms`, `rpc_p95_ms` | Observed RPC + packet-validation latency; absent for ROS-only |
| `errors` | Failed Bridge reads, validation or ROS publications during measurement |
| `overrun_ms` | Time a final blocking operation continued beyond the measurement window |
| `passed` | Paced stage met both rate thresholds, loss threshold, and zero errors |

A paced stage passes when sent **and received** Hz reach at least **95%** of target, loss is at most **1%**, and there are no errors. The summary reports the highest target that passes **every repeat**. No passing target produces `null`. Unthrottled rows report observed saturation and never earn a paced-target pass. Inspect their spread across repeats and confirm a nearby paced rate for longer runs.

The separate subscriber process uses the same host monotonic clock as the publisher. Old warmup/stage messages are filtered by publication sequence. Late drain deliveries reduce loss but do not inflate in-window throughput. Duplicates and reordering are counted. Missed pacing deadlines are skipped rather than followed by catch-up bursts. Message IDs start at zero for each run and increment only after successful publish calls. No extra sequence topic or custom ROS interface is required.

Bridge mode makes **one synchronous RPC per message**, matching the BMI088 app's request/reply architecture. It removes I²C, temperature, LED matrix work and console polling. ROS-only skips the MCU and Bridge while retaining the message, QoS and independent receiver. Compare these two modes to investigate the bottleneck. A slow Bridge stage does not, by itself, establish the MCU's raw generation speed.

Default QoS is **best effort, keep last, depth 100**, on both ends. Your BMI088 app uses sensor-data QoS with a smaller queue; use `--depth 5` for that comparison. Use `--reliable` for a separate reliability/backpressure test. QoS, middleware, board load, receiver scheduling, temperature, and container versions can change measured throughput.

This tests delivery to a **local subscriber on UNO Q**, including Python and DDS overhead. It does not measure external-network throughput, sample acquisition latency, micro-ROS, batching, or the theoretical hardware limit. ROS timestamps are Linux publication timestamps. The dummy payload has the same array shape and numeric types as the source, but MessagePack byte counts can differ with values. The recorded hardware run above measures this implementation; it does not establish a theoretical hardware maximum.

## Implementation and validation

- `sketch/sketch.ino`: eight-float dummy RPC, served with `Bridge.provide_safe`.
- `bricks/ros2_jazzy/app/app_utils/bridge.py`: persistent MessagePack socket client reused from the BMI088 app.
- `bricks/ros2_jazzy/app/benchmark.py`: finite sweep, independent ROS subscriber, durable results.
- `bricks/ros2_jazzy/app/metrics.py`: payload validation and receiver accounting.
- `python/main.py`: idle App Lab console, with no competing RPC traffic.

Run host-side accounting tests:

```bash
python3 -m unittest discover -s tests -v
python3 -m compileall -q python bricks/ros2_jazzy/app
```

These checks do not substitute for compiling against the UNO Q board core or running DDS/RouterBridge on hardware. The app targets `arduino:zephyr:unoq` and uses the RouterBridge library from the installed Arduino platform/library environment. Versions are not pinned; record your App Lab, board core and RouterBridge versions with reported results. The container follows the ROS Jazzy base image tag.

[Arduino RouterBridge API](https://github.com/arduino-libraries/Arduino_RouterBridge) documents safe callbacks and their loop-thread dispatch. [UNO Q manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/) describes the board and App Lab environment. Original project and reused client are MIT licensed; see `LICENSE`.
