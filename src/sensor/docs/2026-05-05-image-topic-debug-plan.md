# 2026-05-05 Image Topic Debug Plan

## Goal

Find why `/camera/image` and sometimes other sensor topics are only intermittently
usable after `ros2 launch sensor launch.py`.

Assumptions for this debug session:

- Camera, LiDAR, trigger wiring, and basic hardware are treated as already
  verified.
- `gst-launch` can receive camera frames in at least one known-good condition.
- RViz is not the root cause because rosbag recording also showed missing image
  data.
- Topic existence is not enough. We must distinguish:
  - ROS graph endpoint exists.
  - `publisher_->publish()` is actually called.
  - subscriber actually receives `sensor_msgs/Image`.
  - received image pixels are valid and changing.

## Current Evidence

From the previous run:

- `ros2 launch sensor launch.py` started LiDAR and then image node.
- `see3cam_trigger` opened HID successfully.
- Camera was set to `TRIGGER` mode successfully.
- GStreamer pipeline reached `PLAYING`.
- `/camera/image` appeared in ROS graph with one publisher.
- A BEST_EFFORT image subscriber received `0` frames for 15 seconds.
- `/tmp/pub_trace_camera.bin` was `0 bytes`.
- `see3cam_trigger` printed `CAMERA FAIL -- stamped_frames=0`.

Interpretation:

- `create_publisher()` happened, so the topic endpoint exists.
- `on_new_sample()` did not successfully receive any GStreamer sample.
- Therefore actual `publisher_->publish()` was not reached in that run.

## Main Hypotheses

### H1: Trigger-mode frame source is not delivering buffers to `v4l2src`

Evidence pointing here:

- HID mode changes to `TRIGGER`.
- GStreamer is `PLAYING`, but no `appsink` sample arrives.
- `stamped_frames=0` means no image callback reached stamp/publish path.

How to test:

- Run the exact ROS pipeline outside ROS after setting HID trigger mode.
- Save actual PNG frames, not just measure fps.
- Compare MASTER/free-run mode vs TRIGGER mode.

Expected result if true:

- MASTER/free-run produces PNG files.
- TRIGGER mode produces no PNG files or stalls unless external trigger is active
  and accepted by the camera.

### H2: ROS publisher exists but QoS/subscriber mismatch hides frames

Evidence against this in the previous run:

- Test subscriber used BEST_EFFORT to match publisher QoS.
- `pub_trace_camera.bin` was empty, so publish was not reached.

Still test because intermittent sensor topics may include other topics:

- Use `ros2 topic info -v` for `/camera/image`, `/livox/lidar`, `/livox/imu`.
- Subscriber probes must use matching QoS:
  - sensor topics: BEST_EFFORT, VOLATILE.
  - avoid Reliable subscribers for BEST_EFFORT publishers.

Expected result if true:

- Publisher logs show frames published, but mismatched subscriber receives none.
- `pub_trace_camera.bin` has samples while probe receives none.

### H3: Queue/buffer starvation or backpressure in image node

Relevant code path:

- GStreamer appsink queue: `max-buffers=4 drop=true`.
- Internal message pool: `kPoolSize = 4`.
- `acquire_slot()` can drop ready frames if publisher thread lags.

Evidence against this in previous run:

- `pub_trace_camera.bin` was empty, so it did not reach queue handoff.

Still test because failure may be intermittent:

- Add counters for:
  - `new_sample_enter`
  - `pull_sample_null`
  - `caps_missing`
  - `map_fail`
  - `slot_null`
  - `handoff_count`
  - `publish_count`
  - `dropped_count`
  - `publish_throw_count`
- Log these once per second even when no frames arrive.

Expected result if true:

- `new_sample_enter` increases.
- `handoff_count` increases.
- `publish_count` lags or `dropped_count` rises.
- Subscriber may see irregular images but not zero frames from startup.

### H4: GStreamer bus error occurs but current logs miss the state transition

Relevant code:

- `on_bus_message()` only handles `GST_MESSAGE_ERROR`.
- It immediately calls `NULL -> PLAYING` on the same pipeline.

Risk:

- Repeated error recovery may create a loop.
- Warnings, EOS, state changes, and stream status are not logged.

How to test:

- Log bus messages:
  - `ERROR`
  - `WARNING`
  - `EOS`
  - `STATE_CHANGED` for pipeline and `v4l2src`
  - `STREAM_STATUS`
  - `ASYNC_DONE`
- On error, record exact timestamp and frame counters.

Expected result if true:

- Topic exists.
- Frames stop after a bus warning/error/state transition.
- Restart behavior correlates with missing images.

### H5: LiDAR/launch gating intermittently starts image node in a bad order

Launch currently starts image node after both strings appear:

- `ALUX SYNC ... OK`
- `ALUX Lidar ... OK`

Risk:

- The LiDAR writer may be ready for stamp, but camera trigger source may not be
  ready.
- External trigger may not be active or stable when image pipeline starts.

How to test:

- Log launch time markers:
  - LiDAR process start.
  - `ALUX SYNC ... OK`.
  - `ALUX Lidar ... OK`.
  - image process start.
  - HID trigger set complete.
  - GStreamer PLAYING.
  - first `new-sample`.
  - first `publish`.
  - first subscriber PNG saved.

Expected result if true:

- Bad runs differ in ordering or delay before first sample.
- Good runs have a repeatable first-sample window.

## Required Logging Changes

### 1. Per-second camera health line

Add a wall timer in `see3cam_trigger` that prints one compact line per second:

```text
CAM_HEALTH sec=12 gst_state=PLAYING samples=0 pulled=0 published=0 ready_q=0 free_q=4 drops=0 partial=0 map_fail=0 slot_null=0 gst_errors=0 last_sample_age_ms=-1
```

Why:

- Current logs only print when frames arrive.
- If no frame arrives, silence hides the failure layer.

### 2. Bus message logging

Log all relevant GStreamer bus messages with source name:

```text
GST_BUS type=STATE_CHANGED src=v4l2src0 old=READY new=PAUSED pending=PLAYING
GST_BUS type=WARNING src=v4l2src0 msg=...
GST_BUS type=ERROR src=v4l2src0 msg=... dbg=...
GST_BUS type=EOS src=pipeline0
```

Why:

- Need to know whether `v4l2src` never produces buffers or fails later.

### 3. Appsink callback counters

At the top of `on_new_sample()`:

- increment `new_sample_enter`.
- record monotonic timestamp.

At each early return:

- increment a reason-specific counter.

Why:

- This separates no-buffer, bad-caps, map failure, pool exhaustion, and publish
  failure.

### 4. Actual image probe output

Keep a separate subscriber probe that writes:

- `/tmp/camera_image_probe/frame_000.png`
- `/tmp/camera_image_probe/frame_001.png`
- `/tmp/camera_image_probe/report.txt`

Report fields:

- received frame count.
- encoding, width, height, step.
- min, max, mean, std, entropy.
- frame-to-frame mean absolute difference.
- percent of pixels changed.
- judgement: empty, flat, repeated, valid-changing.

Why:

- `hz` and message size cannot detect black frames, frozen frames, bad stride, or
  unchanged image content.

## Tomorrow Execution Order

### Step 1: Start with a clean logging directory

Create a run id:

```bash
export RUN_ID=$(date +%Y%m%d_%H%M%S)
export RUN_DIR=/tmp/sensor_debug_$RUN_ID
mkdir -p "$RUN_DIR"
```

Capture environment:

```bash
env | sort > "$RUN_DIR/env.txt"
ls -l /dev/24cug /dev/24cug_hid /dev/video* /dev/hidraw* > "$RUN_DIR/devices.txt" 2>&1
v4l2-ctl -d /dev/24cug --all > "$RUN_DIR/v4l2_all_before.txt" 2>&1
```

### Step 2: Launch stack and save full output

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_dep_ws2/install/setup.bash
ROS_DOMAIN_ID=30 ros2 launch sensor launch.py 2>&1 | tee "$RUN_DIR/launch.log"
```

Keep this terminal open.

### Step 3: In another terminal, record graph and QoS snapshots

Run every 5 seconds during the first minute:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_dep_ws2/install/setup.bash
ROS_DOMAIN_ID=30 ros2 topic list -t > "$RUN_DIR/topic_list_t.txt"
ROS_DOMAIN_ID=30 ros2 topic info /camera/image -v > "$RUN_DIR/camera_image_info.txt" 2>&1
ROS_DOMAIN_ID=30 ros2 topic info /livox/lidar -v > "$RUN_DIR/lidar_info.txt" 2>&1
ROS_DOMAIN_ID=30 ros2 topic info /livox/imu -v > "$RUN_DIR/imu_info.txt" 2>&1
```

Do not rely on `hz` as the primary evidence.

### Step 4: Run actual image probe

Use a BEST_EFFORT subscriber. It must save PNG frames and report pixel stats.

Success criteria:

- At least 10 PNG files saved.
- `std` not near zero.
- entropy is reasonable.
- frame-to-frame difference is nonzero.

Failure categories:

- `0 frames`: no actual publish or QoS mismatch.
- PNGs all black/flat: image payload exists but pixels invalid.
- PNGs identical: stale/repeated frame.
- malformed layout: encoding/step/data size bug.

### Step 5: Run ROS-independent GStreamer image save test

Test exact pipeline in current camera mode:

```bash
gst-launch-1.0 -e \
  v4l2src device=/dev/24cug num-buffers=20 \
  ! video/x-raw,format=UYVY,width=1280,height=720,framerate=60/1 \
  ! nvvidconv \
  ! video/x-raw,format=BGRx,width=640,height=360 \
  ! videoconvert \
  ! pngenc \
  ! multifilesink location="$RUN_DIR/gst_frame_%03d.png"
```

Interpretation:

- If this saves PNGs while ROS gets zero frames, ROS node/pipeline setup differs.
- If this also saves zero PNGs in trigger mode, the issue is before ROS publish:
  trigger-mode buffer delivery into V4L2/GStreamer.

### Step 6: Compare MASTER vs TRIGGER mode

Use HID control to switch modes and repeat Step 5:

- MASTER/free-run mode.
- TRIGGER mode.

Record:

- mode query result.
- number of PNGs produced.
- whether frame content changes.

Expected useful split:

- MASTER works, TRIGGER fails: trigger delivery/mode interaction.
- Both work, ROS fails: ROS node pipeline/error handling.
- Both fail: device/caps/current camera state issue despite earlier hardware
  checks.

### Step 7: Bag only after image probe sees frames

If image probe receives valid frames:

```bash
ROS_DOMAIN_ID=30 ros2 bag record /camera/image /livox/lidar /livox/imu -o "$RUN_DIR/bag"
```

Then replay and run the same PNG probe against bag playback.

Why:

- Bagging before confirming actual pixel validity can preserve the wrong thing:
  topic metadata or empty intervals.

## Decision Table

| Observation | Meaning | Next Action |
|---|---|---|
| `/camera/image` exists, image probe gets 0 frames, `pub_trace_camera.bin` 0B | publisher endpoint only; no actual image publish | inspect appsink/GStreamer source |
| image probe gets frames, PNGs black/flat | publish works; pixel payload invalid | inspect caps, stride, conversion |
| image probe gets identical PNGs | stale/repeated frame | inspect queue, buffer reuse, trigger |
| `new_sample_enter` increases but `published` does not | internal queue/publish path bug | inspect pool and publish thread |
| `published` increases but probe gets 0 | QoS/domain/subscriber mismatch | inspect QoS and ROS_DOMAIN_ID |
| GStreamer standalone works in MASTER but not TRIGGER | trigger-mode capture issue | inspect trigger signal/mode timing |
| GStreamer standalone works but ROS node does not | node pipeline or bus handling issue | instrument/rebuild `see3cam_trigger` |

## Files To Preserve After Each Run

Copy these into `$RUN_DIR`:

- launch output.
- `.ros/log/see3cam_trigger_*.log`.
- `.ros/log/livox*.log` if present.
- `/tmp/pub_trace_camera.bin`.
- `/tmp/pub_trace_lidar.bin`.
- `/tmp/pub_trace_imu.bin`.
- `/tmp/camera_image_probe/*`.
- GStreamer standalone PNGs.
- `v4l2-ctl --all` before and after.
- `ros2 topic info -v` snapshots.

## First Code Patch To Make Tomorrow

Patch only instrumentation first. Do not change recovery behavior yet.

Add:

- per-second `CAM_HEALTH` timer.
- GStreamer bus logging for warning/EOS/state changes.
- reason-specific counters in `on_new_sample()`.
- explicit `publish_count_`.
- queue sizes in health log.

Do not yet add:

- automatic restart policy changes.
- extra validation gates.
- clamps/retries that hide the root cause.

After one instrumented run, decide whether to patch recovery logic.

