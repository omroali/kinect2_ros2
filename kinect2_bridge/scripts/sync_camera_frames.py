#!/usr/bin/env python3
"""
Cross-Camera Frame Synchroniser
=================================
Post-processing utility that aligns frames from multiple cameras by
nearest timestamp, producing a single merged CSV index where each row
represents a set of frames (one per camera) that are as close in time
as possible.

This is intentionally a simple offline tool — run it once after recording
before starting ML inference.  It does not require ROS to be running.

Background
----------
The Kinect v2 has no hardware sync between units.  Each sensor's depth and
colour frames are paired by kinect2_bridge (see createHeader() in
kinect2_bridge.cpp), but two different Kinects capturing at the same instant
will have independent USB-arrival timestamps that drift by up to ~33ms (one
frame period at 30fps).

This script takes the per-camera timestamp CSVs written by
colour_video_recorder.py and finds the best frame match across all cameras
within a configurable tolerance window.  Unmatched frames (where no other
camera has a frame within the tolerance) are dropped.

Usage
-----
# Two cameras, default 33ms tolerance
python3 sync_camera_frames.py \
    --cameras kinect2_1_qhd_image_color_rect.csv \
              kinect2_2_qhd_image_color_rect.csv \
    --output  synced_frames.csv

# Four cameras, tighter 20ms tolerance
python3 sync_camera_frames.py \
    --cameras kinect2_1_qhd_image_color_rect.csv \
              kinect2_2_qhd_image_color_rect.csv \
              kinect2_3_qhd_image_color_rect.csv \
              kinect2_4_qhd_image_color_rect.csv \
    --output  synced_frames.csv \
    --tolerance_ms 20

Output CSV columns
------------------
  sync_idx          Row index (0-based) in the synced output
  cam_N_frame_idx   Frame index in camera N's .mp4 video
  cam_N_ts_ns       ROS timestamp of that frame in nanoseconds
  cam_N_offset_ms   Offset in ms from the reference camera (cam_0 = 0.0)

  cam_0 is the reference camera (first in --cameras list).
  All other cameras are matched to cam_0's frame timestamps.

Loading in ML code
------------------
  import pandas as pd
  import cv2

  sync  = pd.read_csv('synced_frames.csv')
  caps  = [cv2.VideoCapture(f'kinect2_{i+1}_qhd_image_color_rect.mp4')
           for i in range(4)]

  for _, row in sync.iterrows():
      frames = []
      for i, cap in enumerate(caps):
          cap.set(cv2.CAP_PROP_POS_FRAMES, int(row[f'cam_{i}_frame_idx']))
          ok, frame = cap.read()
          if ok:
              frames.append(frame)
      # frames[0..3] are now the best-matched set across all 4 cameras
      # pass to your ML model
"""

import argparse
import csv
import os
import sys


# ── CSV loader ────────────────────────────────────────────────────────────── #

def load_timestamps(csv_path: str) -> list[tuple[int, int]]:
    """
    Load a timestamp CSV produced by colour_video_recorder.py.
    Returns a list of (frame_idx, ros_timestamp_ns) tuples sorted by timestamp.
    """
    if not os.path.exists(csv_path):
        raise FileNotFoundError(f"Timestamp CSV not found: {csv_path}")

    rows = []
    with open(csv_path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append((int(row['frame_idx']), int(row['ros_timestamp_ns'])))

    # Should already be sorted, but guarantee it
    rows.sort(key=lambda r: r[1])
    return rows


# ── Nearest-neighbour matcher ─────────────────────────────────────────────── #

def find_nearest(timestamps: list[tuple[int, int]], target_ns: int) -> tuple[int, int] | None:
    """
    Binary search for the frame in `timestamps` whose timestamp is closest
    to `target_ns`.  Returns (frame_idx, ts_ns) or None if the list is empty.
    """
    if not timestamps:
        return None

    lo, hi = 0, len(timestamps) - 1
    while lo < hi:
        mid = (lo + hi) // 2
        if timestamps[mid][1] < target_ns:
            lo = mid + 1
        else:
            hi = mid

    # lo is the insertion point — check both neighbours
    best = lo
    if lo > 0 and abs(timestamps[lo - 1][1] - target_ns) < abs(timestamps[lo][1] - target_ns):
        best = lo - 1

    return timestamps[best]


# ── Main sync logic ───────────────────────────────────────────────────────── #

def sync_cameras(
    csv_paths: list[str],
    output_path: str,
    tolerance_ns: int,
) -> None:
    """
    Align frames across all cameras by nearest timestamp.

    Algorithm
    ---------
    cam_0 is the reference.  For every frame in cam_0 we search all other
    cameras for the nearest frame within `tolerance_ns`.  If every camera
    has a match the row is written to the output; otherwise it is dropped.

    This is O(N * M) where N = frames in cam_0, M = number of cameras,
    and each search is O(log F) via binary search.
    """

    camera_data = [load_timestamps(p) for p in csv_paths]
    n_cameras   = len(camera_data)
    reference   = camera_data[0]

    print(f"Loaded {n_cameras} cameras:")
    for i, (path, data) in enumerate(zip(csv_paths, camera_data)):
        print(f"  cam_{i}: {os.path.basename(path)}  ({len(data)} frames)")

    # ── Build output header ───────────────────────────────────────────────── #
    fieldnames = ['sync_idx']
    for i in range(n_cameras):
        fieldnames += [f'cam_{i}_frame_idx', f'cam_{i}_ts_ns', f'cam_{i}_offset_ms']

    matched      = 0
    dropped      = 0
    max_offsets  = [0.0] * n_cameras   # track worst-case offsets for reporting

    with open(output_path, 'w', newline='') as out_f:
        writer = csv.DictWriter(out_f, fieldnames=fieldnames)
        writer.writeheader()

        for ref_frame_idx, ref_ts_ns in reference:

            row = {'sync_idx': matched}

            # Reference camera is always offset 0
            row[f'cam_0_frame_idx'] = ref_frame_idx
            row[f'cam_0_ts_ns']     = ref_ts_ns
            row[f'cam_0_offset_ms'] = 0.0

            all_matched = True

            for cam_i in range(1, n_cameras):
                result = find_nearest(camera_data[cam_i], ref_ts_ns)

                if result is None:
                    all_matched = False
                    break

                match_frame_idx, match_ts_ns = result
                offset_ns = abs(match_ts_ns - ref_ts_ns)

                if offset_ns > tolerance_ns:
                    # No frame close enough on this camera — drop the row
                    all_matched = False
                    break

                offset_ms = (match_ts_ns - ref_ts_ns) / 1_000_000.0
                max_offsets[cam_i] = max(max_offsets[cam_i], abs(offset_ms))

                row[f'cam_{cam_i}_frame_idx'] = match_frame_idx
                row[f'cam_{cam_i}_ts_ns']     = match_ts_ns
                row[f'cam_{cam_i}_offset_ms'] = round(offset_ms, 3)

            if all_matched:
                writer.writerow(row)
                matched += 1
            else:
                dropped += 1

    # ── Report ────────────────────────────────────────────────────────────── #
    total      = matched + dropped
    drop_pct   = 100.0 * dropped / total if total > 0 else 0.0
    tolerance_ms = tolerance_ns / 1_000_000.0

    print(f"\nSync complete:")
    print(f"  tolerance   : {tolerance_ms:.1f} ms")
    print(f"  ref frames  : {total}")
    print(f"  matched     : {matched}  ({100 - drop_pct:.1f}%)")
    print(f"  dropped     : {dropped}  ({drop_pct:.1f}%)")
    print(f"  output      : {output_path}")
    print(f"\nWorst-case offsets per camera (ms):")
    for i in range(n_cameras):
        label = "(reference)" if i == 0 else f"max {max_offsets[i]:.2f} ms"
        print(f"  cam_{i}: {label}")

    if drop_pct > 20.0:
        print(
            f"\nWARNING: {drop_pct:.1f}% of frames dropped. "
            "Consider raising --tolerance_ms or check that all cameras "
            "were running at the same time."
        )


# ── CLI ───────────────────────────────────────────────────────────────────── #

def main():
    parser = argparse.ArgumentParser(
        description="Align frames from multiple cameras by nearest timestamp.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        '--cameras',
        nargs='+',
        required=True,
        metavar='CSV',
        help="Timestamp CSV files from colour_video_recorder.py, "
             "one per camera. First file is the reference camera.",
    )
    parser.add_argument(
        '--output',
        default='synced_frames.csv',
        metavar='CSV',
        help="Output merged CSV path. Default: synced_frames.csv",
    )
    parser.add_argument(
        '--tolerance_ms',
        type=float,
        default=33.0,
        metavar='MS',
        help="Maximum allowed timestamp difference between cameras in "
             "milliseconds. Frames outside this window are dropped. "
             "Default: 33ms (one frame at 30fps).",
    )

    args = parser.parse_args()

    if len(args.cameras) < 2:
        print("ERROR: at least 2 camera CSVs are required.")
        sys.exit(1)

    tolerance_ns = int(args.tolerance_ms * 1_000_000)

    sync_cameras(
        csv_paths    = args.cameras,
        output_path  = args.output,
        tolerance_ns = tolerance_ns,
    )


if __name__ == '__main__':
    main()
