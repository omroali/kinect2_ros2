# Recording & Reconstruction Pipeline

This document covers the full data lifecycle for long-duration multi-sensor
recordings: how to record efficiently, how to reconstruct PointCloud2 data
offline, and how to load frames for ML inference.

---

## Overview

The pipeline is split into three independent phases:

```
┌─────────────────────────────────────────────────────────────────────┐
│  RECORD (live, with hardware)                                       │
│                                                                     │
│  kinect2_bridge                                                     │
│    colour → H.265 .mp4 + timestamp .csv   (colour_video_recorder)  │
│    depth  → .mcap bag  (compressed TIFF/PNG, lossless)             │
│    info   → .mcap bag  (camera_info, tiny)                         │
└─────────────────────────────────────────────────────────────────────┘
                          │
                          ▼
┌─────────────────────────────────────────────────────────────────────┐
│  POST-PROCESS (offline, no ROS needed)                              │
│                                                                     │
│  sync_camera_frames.py                                              │
│    timestamp CSVs → synced_frames.csv   (cross-camera alignment)   │
└─────────────────────────────────────────────────────────────────────┘
                          │
              ┌───────────┴───────────┐
              ▼                       ▼
┌─────────────────────┐   ┌───────────────────────────────────────────┐
│  ML INFERENCE       │   │  POINTCLOUD RECONSTRUCTION                │
│  (no ROS needed)    │   │  (ROS, no hardware needed)                │
│                     │   │                                           │
│  synced_frames.csv  │   │  bag play --clock                         │
│  + .mp4 files       │   │  + video_to_image_publisher               │
│  → cv2 frames       │   │  + image_transport republish (depth)      │
│  → your ML model    │   │  + depth_image_proc → PointCloud2         │
└─────────────────────┘   └───────────────────────────────────────────┘
```

---

## Storage estimates (4 sensors, 100 hours, QHD 960×540 @ 30fps)

| Stream | Format | Per sensor | 4 sensors × 100 hrs |
|---|---|---|---|
| Colour | H.265 CRF 28 | ~150 KB/s | **~210 GB** |
| Depth | TIFF compressed bag | ~420 KB/s | **~580 GB** |
| Camera info | bag | negligible | negligible |
| **Total** | | | **~790 GB** |

Compare to raw: **~63 TB**.  H.265 + compressed depth is approximately **80× smaller**.

> **Why not H.265 for depth?**  Standard video codecs target 8-bit YUV.
> Depth is `uint16` millimetre values — forcing it through H.265 silently
> truncates to 8-bit, destroying geometry irreversibly.  TIFF/PNG are
> lossless for 16-bit data and already built into kinect2_bridge.

---

## Dependencies

```bash
# ffmpeg (encoding/decoding)
sudo apt install ffmpeg

# Python packages (ML inference only — not needed for ROS pipeline)
pip install opencv-python pandas numpy

# ROS packages (reconstruction only)
sudo apt install ros-$ROS_DISTRO-depth-image-proc \
                 ros-$ROS_DISTRO-image-transport-plugins
```

---

## Phase 1 — Recording

Run two processes in parallel: `colour_video_recorder` for colour streams and
`ros2 bag record` for depth + camera info.

### Step 1a — Start the bridge

```bash
ros2 launch kinect2_bridge kinect2_single.launch.py namespace:=kinect2_1
# repeat for kinect2_2, kinect2_3, kinect2_4
```

### Step 1b — Start the colour video recorder

Records all four colour streams into H.265 `.mp4` files and companion
timestamp `.csv` files.

```bash
ros2 run kinect2_bridge colour_video_recorder.py \
    --ros-args \
    -p topics:="[/kinect2_1/qhd/image_color_rect,
                  /kinect2_2/qhd/image_color_rect,
                  /kinect2_3/qhd/image_color_rect,
                  /kinect2_4/qhd/image_color_rect]" \
    -p output_dir:="/data/session_001" \
    -p encoder:=libx265 \
    -p crf:=28 \
    -p fps:=30.0
```

**Hardware GPU encoding (recommended for 4 cameras):**

| GPU | encoder param |
|---|---|
| Nvidia | `encoder:=hevc_nvenc` |
| Intel iGPU | `encoder:=hevc_qsv` |
| AMD | `encoder:=hevc_amf` |

**CRF quality guide:**

| CRF | Quality | Approx size |
|---|---|---|
| 18 | Visually lossless | ~500 KB/s |
| 28 | Good (default) | ~150 KB/s |
| 35 | Small, soft | ~60 KB/s |

**Output files per camera:**
```
/data/session_001/
  kinect2_1_qhd_image_color_rect.mp4   ← H.265 colour video
  kinect2_1_qhd_image_color_rect.csv   ← per-frame ROS timestamps
  kinect2_2_qhd_image_color_rect.mp4
  kinect2_2_qhd_image_color_rect.csv
  ...
```

**CSV format:**
```
frame_idx, ros_timestamp_ns
0,         1705312800123456789
1,         1705312800156789012
...
```

### Step 1c — Record depth and camera info (bag)

```bash
ros2 bag record -o /data/session_001/depth \
    /kinect2_1/qhd/image_depth_rect/compressed \
    /kinect2_1/qhd/camera_info \
    /kinect2_2/qhd/image_depth_rect/compressed \
    /kinect2_2/qhd/camera_info \
    /kinect2_3/qhd/image_depth_rect/compressed \
    /kinect2_3/qhd/camera_info \
    /kinect2_4/qhd/image_depth_rect/compressed \
    /kinect2_4/qhd/camera_info
```

> The `/compressed` depth topics are TIFF-encoded by default (lossless 16-bit).
> Use `use_png:=true` on the bridge if you prefer PNG — both are lossless.

### Session layout after recording

```
/data/session_001/
  kinect2_1_qhd_image_color_rect.mp4
  kinect2_1_qhd_image_color_rect.csv
  kinect2_2_qhd_image_color_rect.mp4
  kinect2_2_qhd_image_color_rect.csv
  kinect2_3_qhd_image_color_rect.mp4
  kinect2_3_qhd_image_color_rect.csv
  kinect2_4_qhd_image_color_rect.mp4
  kinect2_4_qhd_image_color_rect.csv
  depth/                              ← ros2 bag directory
    depth_0.mcap
    metadata.yaml
```

---

## Phase 2 — Cross-camera frame synchronisation

The Kinect v2 has no hardware sync between units.  Each sensor's frames are
timestamped independently by USB arrival time, which can differ by up to ~33ms
between sensors.  `sync_camera_frames.py` aligns frames from all cameras by
nearest timestamp and produces a single merged index CSV.

**Run once per session, no ROS needed:**

```bash
python3 /path/to/kinect2_bridge/scripts/sync_camera_frames.py \
    --cameras \
        /data/session_001/kinect2_1_qhd_image_color_rect.csv \
        /data/session_001/kinect2_2_qhd_image_color_rect.csv \
        /data/session_001/kinect2_3_qhd_image_color_rect.csv \
        /data/session_001/kinect2_4_qhd_image_color_rect.csv \
    --output /data/session_001/synced_frames.csv \
    --tolerance_ms 33
```

**Example output:**
```
Loaded 4 cameras:
  cam_0: kinect2_1_qhd_image_color_rect.csv  (1080000 frames)
  cam_1: kinect2_2_qhd_image_color_rect.csv  (1079843 frames)
  cam_2: kinect2_3_qhd_image_color_rect.csv  (1080102 frames)
  cam_3: kinect2_4_qhd_image_color_rect.csv  (1079991 frames)

Sync complete:
  tolerance   : 33.0 ms
  matched     : 1079201  (99.9%)
  dropped     : 799      (0.1%)

Worst-case offsets per camera (ms):
  cam_0: (reference)
  cam_1: max 16.23 ms
  cam_2: max 14.87 ms
  cam_3: max 18.41 ms
```

**Synced CSV format:**
```
sync_idx, cam_0_frame_idx, cam_0_ts_ns,          cam_0_offset_ms, cam_1_frame_idx, ...
0,        0,               1705312800123456789,   0.0,             0,               ...
1,        1,               1705312800156789012,   0.0,             1,               ...
```

> `cam_0` is always the reference (first `--cameras` entry, offset = 0).
> `cam_N_offset_ms` tells you how far off each match was — useful for
> filtering frames where timing was particularly poor.

---

## Phase 3a — ML Inference

No ROS required.  Load frames directly from the `.mp4` files using the synced
index.

```python
import pandas as pd
import cv2

SESSION = "/data/session_001"

# Load the synced frame index built in Phase 2
sync = pd.read_csv(f"{SESSION}/synced_frames.csv")

# Open one VideoCapture per camera
caps = [
    cv2.VideoCapture(f"{SESSION}/kinect2_{i+1}_qhd_image_color_rect.mp4")
    for i in range(4)
]

for _, row in sync.iterrows():

    frames = []
    for i, cap in enumerate(caps):
        # Seek to the matched frame for this camera
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(row[f"cam_{i}_frame_idx"]))
        ok, frame = cap.read()   # BGR uint8, shape (540, 960, 3)
        if ok:
            frames.append(frame)

    # frames[0..3] are the best time-matched set across all 4 cameras
    # Pass to your model, e.g.:
    #   results = model.predict(frames)

for cap in caps:
    cap.release()
```

**Tips:**
- `cap.set(cv2.CAP_PROP_POS_FRAMES, n)` is cheap for H.265 of mostly-static
  scenes because P-frames only store motion deltas — seeking is fast.
- If you only need sequential playback (no random access), remove the
  `cap.set()` call and just call `cap.read()` — it will be faster.
- Use `row["cam_N_offset_ms"]` to skip or flag frames where cross-camera
  timing was poor, if your model requires tight sync.

---

## Phase 3b — PointCloud2 Reconstruction

Reconstructs `sensor_msgs/PointCloud2` from the depth bag + colour videos.
Requires ROS but not the physical sensor hardware.

The sync mechanism relies on the bag's `/clock` topic.  `ros2 bag play --clock`
drives sim time forward at the original rate.  `video_to_image_publisher`
subscribes to `/clock` and publishes each colour frame exactly when sim time
reaches that frame's original timestamp — so `depth_image_proc`'s approximate
time synchroniser sees matching timestamps on colour and depth and pairs them
correctly.

### Single camera

Open four terminals:

**Terminal 1 — replay depth bag:**
```bash
ros2 bag play /data/session_001/depth \
    --clock \
    --topics \
        /kinect2_1/qhd/image_depth_rect/compressed \
        /kinect2_1/qhd/camera_info
```

**Terminal 2 — republish colour video into ROS:**
```bash
ros2 run kinect2_bridge video_to_image_publisher.py \
    --ros-args \
    -p video:="/data/session_001/kinect2_1_qhd_image_color_rect.mp4" \
    -p timestamps:="/data/session_001/kinect2_1_qhd_image_color_rect.csv" \
    -p topic:="/kinect2_1/qhd/image_color_rect" \
    -p use_sim_time:=true
```

**Terminal 3 — decompress depth:**
```bash
ros2 run image_transport republish compressed raw \
    --ros-args \
    --remap in/compressed:=/kinect2_1/qhd/image_depth_rect/compressed \
    --remap out:=/kinect2_1/qhd/image_depth_rect \
    -p use_sim_time:=true
```

**Terminal 4 — reconstruct pointcloud:**
```bash
ros2 run depth_image_proc point_cloud_xyzrgb_node \
    --ros-args \
    --remap rgb/camera_info:=/kinect2_1/qhd/camera_info \
    --remap rgb/image_rect_color:=/kinect2_1/qhd/image_color_rect \
    --remap depth_registered/image_rect:=/kinect2_1/qhd/image_depth_rect \
    --remap points:=/kinect2_1/qhd/points \
    -p use_sim_time:=true
```

The pointcloud is published on `/kinect2_1/qhd/points` as
`sensor_msgs/PointCloud2`.

### All four cameras

**Terminal 1 — depth bag (all cameras):**
```bash
ros2 bag play /data/session_001/depth \
    --clock \
    --topics \
        /kinect2_1/qhd/image_depth_rect/compressed /kinect2_1/qhd/camera_info \
        /kinect2_2/qhd/image_depth_rect/compressed /kinect2_2/qhd/camera_info \
        /kinect2_3/qhd/image_depth_rect/compressed /kinect2_3/qhd/camera_info \
        /kinect2_4/qhd/image_depth_rect/compressed /kinect2_4/qhd/camera_info
```

**Terminal 2 — colour video republisher (all cameras, one node):**
```bash
ros2 run kinect2_bridge video_to_image_publisher.py \
    --ros-args \
    -p videos:="[/data/session_001/kinect2_1_qhd_image_color_rect.mp4,
                  /data/session_001/kinect2_2_qhd_image_color_rect.mp4,
                  /data/session_001/kinect2_3_qhd_image_color_rect.mp4,
                  /data/session_001/kinect2_4_qhd_image_color_rect.mp4]" \
    -p timestamp_csvs:="[/data/session_001/kinect2_1_qhd_image_color_rect.csv,
                          /data/session_001/kinect2_2_qhd_image_color_rect.csv,
                          /data/session_001/kinect2_3_qhd_image_color_rect.csv,
                          /data/session_001/kinect2_4_qhd_image_color_rect.csv]" \
    -p topics:="[/kinect2_1/qhd/image_color_rect,
                  /kinect2_2/qhd/image_color_rect,
                  /kinect2_3/qhd/image_color_rect,
                  /kinect2_4/qhd/image_color_rect]" \
    -p use_sim_time:=true
```

**Terminal 3 — decompress depth (repeat for each camera):**
```bash
for ns in kinect2_1 kinect2_2 kinect2_3 kinect2_4; do
    ros2 run image_transport republish compressed raw \
        --ros-args \
        --remap in/compressed:=/${ns}/qhd/image_depth_rect/compressed \
        --remap out:=/${ns}/qhd/image_depth_rect \
        -p use_sim_time:=true &
done
```

**Terminal 4 — pointcloud nodes (repeat for each camera):**
```bash
for ns in kinect2_1 kinect2_2 kinect2_3 kinect2_4; do
    ros2 run depth_image_proc point_cloud_xyzrgb_node \
        --ros-args \
        --remap rgb/camera_info:=/${ns}/qhd/camera_info \
        --remap rgb/image_rect_color:=/${ns}/qhd/image_color_rect \
        --remap depth_registered/image_rect:=/${ns}/qhd/image_depth_rect \
        --remap points:=/${ns}/qhd/points \
        -p use_sim_time:=true &
done
```

Pointclouds are published on:
- `/kinect2_1/qhd/points`
- `/kinect2_2/qhd/points`
- `/kinect2_3/qhd/points`
- `/kinect2_4/qhd/points`

### Visualise in RViz

```bash
ros2 launch kinect2_bridge kinect_viz.launch.py
```

Add a `PointCloud2` display for each `/kinectN/qhd/points` topic.

---

## Scripts and launch files reference

| File | Phase | Purpose |
|---|---|---|
| `scripts/colour_video_recorder.py` | Record | Subscribes to colour topics, encodes H.265 + writes timestamp CSVs |
| `scripts/sync_camera_frames.py` | Post-process | Aligns frames across cameras by nearest timestamp, no ROS needed |
| `scripts/video_to_image_publisher.py` | Reconstruct | Replays H.265 videos back into ROS locked to `/clock` |
| `launch/pointcloud_from_bag.launch.py` | Reconstruct | Convenience launcher for single-camera bag + pointcloud node |

---

## Timestamp and sync notes

### Within a single sensor (depth ↔ colour)
`kinect2_bridge` pairs each depth and colour frame with a **shared timestamp**
(see `createHeader()` in `kinect2_bridge.cpp`).  The first stream to arrive
sets the timestamp; the second stream steals it.  Both messages carry the same
`header.stamp`, which is what `depth_image_proc` requires.  This is handled
automatically — no action needed.

> The timestamp is the ROS wall-clock **arrival time** from the USB driver,
> not a hardware capture timestamp.  It is consistent within a sensor pair
> but drifts by ~1–5 ms from true capture time.  This is acceptable for ML
> and pointcloud work.

### Across multiple sensors
There is no hardware sync between Kinect v2 units.  Each sensor's timestamps
are independent.  `sync_camera_frames.py` aligns them by nearest timestamp
within a configurable tolerance (default 33 ms = one frame at 30 fps).
Expected worst-case drift between sensors is ~15–20 ms in practice.

### Colour video ↔ depth bag
`colour_video_recorder.py` writes `msg.header.stamp` (the same shared
timestamp set by the bridge) into the CSV.  `video_to_image_publisher.py`
restores this timestamp onto the republished `Image` message.
`depth_image_proc` therefore sees identical `header.stamp` values on both
inputs and pairs them correctly.
