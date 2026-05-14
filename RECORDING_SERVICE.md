# Kinect Recording Service

Use this when the Kinect/RealSense topics are already running and you only want to start or stop recording remotely.

## Launch the service manager

```bash
ros2 launch kinect2_bridge kinect_recording_service.launch.py
```

This starts a node that exposes:

- `start_recording` (`std_srvs/srv/Trigger`)
- `stop_recording` (`std_srvs/srv/Trigger`)

## Start a recording

```bash
ros2 service call /start_recording std_srvs/srv/Trigger "{}"
```

This creates a timestamped session folder under:

```bash
~/data/<participant_id>/session_YYYYMMDD_HHMMSS
```

It records:

- color topics via `colour_video_recorder.py`
- depth topics via `ros2 bag record`
- `camera_info`
- `tf` and `tf_static`

## Stop a recording

```bash
ros2 service call /stop_recording std_srvs/srv/Trigger "{}"
```

The manager shuts down the recorder processes cleanly and finalizes the files.

## Configuration

Edit `kinect_recording_service.launch.py` to set:

- `UUID` for the participant folder
- `OUTPUT_ROOT`
- `VIDEO_ENCODER`
- `VIDEO_CRF`
- `VIDEO_FPS`
- `BAG_STORAGE`

Toggle recording per camera in `kinect2_bridge/config/multi_camera_config.yaml`
with `record: true/false`.

## Notes

- Use the service launch if you want remote control from another machine or container.
- Use `kinect_recording.launch.py` if you want recording to start immediately.
