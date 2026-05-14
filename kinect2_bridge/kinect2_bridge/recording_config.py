"""Shared camera recording config helpers."""

from __future__ import annotations

import os
from typing import Any

import yaml


def load_multi_camera_config(config_path: str) -> tuple[str, dict[str, dict[str, Any]]]:
    if not os.path.isfile(config_path):
        raise FileNotFoundError(f"Camera config not found: {config_path}")

    with open(config_path, "r") as f:
        cfg = yaml.safe_load(f) or {}

    cameras = cfg.get("cameras", {})
    if not isinstance(cameras, dict):
        cameras = {}

    return str(cfg.get("world_frame", "map")), cameras


def _num(value: Any, default: float = 0.0) -> str:
    try:
        return str(float(value))
    except (TypeError, ValueError):
        return str(float(default))


def _normalize_camera(namespace: str, cam: dict[str, Any]) -> dict[str, Any]:
    pos = cam.get("position", {}) if isinstance(cam.get("position", {}), dict) else {}
    ori = cam.get("orientation", {}) if isinstance(cam.get("orientation", {}), dict) else {}

    entry = dict(cam)
    entry["namespace"] = str(namespace)
    entry["serial"] = str(cam.get("serial", ""))
    entry["enabled"] = bool(cam.get("enabled", True))
    entry["record"] = bool(cam.get("record", False))
    entry["frame"] = str(cam.get("frame", f"{namespace}_link"))
    entry["x"] = _num(cam.get("x", pos.get("x", 0.0)))
    entry["y"] = _num(cam.get("y", pos.get("y", 0.0)))
    entry["z"] = _num(cam.get("z", pos.get("z", 0.0)))
    entry["roll"] = _num(cam.get("roll", ori.get("roll", 0.0)))
    entry["pitch"] = _num(cam.get("pitch", ori.get("pitch", 0.0)))
    entry["yaw"] = _num(cam.get("yaw", ori.get("yaw", 0.0)))
    return entry


def selected_cameras(config_path: str, flag_name: str) -> tuple[str, list[dict[str, Any]]]:
    world_frame, cameras = load_multi_camera_config(config_path)
    selected: list[dict[str, Any]] = []

    for namespace, cam in cameras.items():
        if not isinstance(cam, dict):
            continue
        if not bool(cam.get(flag_name, False)):
            continue

        selected.append(_normalize_camera(namespace, cam))

    return world_frame, selected


def color_topics(cameras: list[dict[str, Any]]) -> list[str]:
    return [f"/{cam['namespace']}/qhd/image_color_rect" for cam in cameras]


def bag_topics_for_camera(namespace: str) -> list[str]:
    return [
        "/tf",
        "/tf_static",
        f"/{namespace}/qhd/image_depth_rect/compressed",
        f"/{namespace}/sd/image_ir_rect/compressed",
        f"/{namespace}/qhd/camera_info",
    ]
