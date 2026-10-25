#!/usr/bin/env python3
"""Local-only ROS 2 vision dashboard for the Jetson NoMachine desktop."""

from __future__ import annotations

import json
import logging
import math
import os
from collections import defaultdict, deque
from copy import deepcopy
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import subprocess
import threading
import time
from typing import Any

import cv2
import numpy as np
import rclpy
from apriltag_msgs.msg import AprilTagDetectionArray
from px4_msgs.msg import OffboardControlMode, TargetRelativePose, VehicleStatus
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32MultiArray
from tf2_ros import Buffer, TransformException, TransformListener


ROOT = Path(__file__).resolve().parent
HOST = "127.0.0.1"
PORT = 8765
SERVICE = "apriltag-stack.service"
TARGET_FRAMES = {0: "target_body_frd_0", 1: "target_body_frd", 2: "target_body_frd_2"}
SOURCE_RATE_NAMES = ("image", "camera_info", "detections", "pose", "heartbeat", "px4", "tf")


def command(*args: str, timeout: float = 2.0) -> str:
    try:
        result = subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)
        return result.stdout.strip()
    except (OSError, subprocess.TimeoutExpired):
        return ""


def stamp_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def finite(value: float) -> float | None:
    number = float(value)
    return number if math.isfinite(number) else None


class VisionMonitor(Node):
    def __init__(self) -> None:
        super().__init__(
            "vision_dashboard_monitor",
            cli_args=["--ros-args", "-r", "/tf:=/vision_dashboard/tf"],
            use_global_arguments=False,
        )
        self.lock = threading.Lock()
        self.received: dict[str, float] = {}
        self.samples: dict[str, deque[float]] = defaultdict(lambda: deque(maxlen=600))
        self.data: dict[str, Any] = {}
        self.preview_jpeg: bytes | None = None
        self.preview_at = 0.0
        self.system: dict[str, Any] = {}

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, "/vision_dashboard/image_preview", self.on_image, sensor_qos
        )
        self.create_subscription(
            Float32MultiArray, "/vision_dashboard/source_rates", self.on_source_rates, sensor_qos
        )
        self.create_subscription(
            CameraInfo, "/vision_dashboard/camera_info", self.on_camera_info, sensor_qos
        )
        self.create_subscription(
            AprilTagDetectionArray, "/vision_dashboard/detections", self.on_detections, sensor_qos
        )
        self.create_subscription(
            TargetRelativePose, "/vision_dashboard/target_relative_pose", self.on_pose, sensor_qos
        )
        self.create_subscription(
            OffboardControlMode, "/vision_dashboard/offboard_control_mode", self.on_heartbeat, sensor_qos
        )
        self.create_subscription(
            VehicleStatus, "/vision_dashboard/vehicle_status", self.on_vehicle_status, sensor_qos
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=False)
        self.create_timer(0.5, self.refresh_tf)

    def record(self, name: str, payload: dict[str, Any], now: float | None = None) -> None:
        received_at = time.monotonic() if now is None else now
        with self.lock:
            self.received[name] = received_at
            self.samples[name].append(received_at)
            self.data[name] = payload

    def on_image(self, msg: Image) -> None:
        now = time.monotonic()
        with self.lock:
            self.received["image"] = now
            self.samples["image"].append(now)
            self.preview_at = now

        payload = {
            "width": int(msg.width),
            "height": int(msg.height),
            "encoding": msg.encoding,
            "frame_id": msg.header.frame_id,
            "stamp_ns": stamp_ns(msg.header.stamp),
        }
        with self.lock:
            self.data["image"] = payload

        try:
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if msg.encoding == "mono8":
                frame = raw.reshape(msg.height, msg.step)[:, : msg.width]
            elif msg.encoding in ("bgr8", "rgb8"):
                frame = raw.reshape(msg.height, msg.step)[:, : msg.width * 3]
                frame = frame.reshape(msg.height, msg.width, 3)
                if msg.encoding == "rgb8":
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                return
            width = min(int(msg.width), 640)
            height = max(1, round(int(msg.height) * width / int(msg.width)))
            thumbnail = cv2.resize(frame, (width, height), interpolation=cv2.INTER_AREA)
            encoded, jpeg = cv2.imencode(".jpg", thumbnail, [cv2.IMWRITE_JPEG_QUALITY, 72])
            if encoded:
                with self.lock:
                    self.preview_jpeg = jpeg.tobytes()
        except (ValueError, cv2.error) as exc:
            self.get_logger().warning(f"Could not encode camera preview: {exc}")

    def on_camera_info(self, msg: CameraInfo) -> None:
        self.record("camera_info", {
            "width": int(msg.width),
            "height": int(msg.height),
            "fx": finite(msg.k[0]),
            "fy": finite(msg.k[4]),
            "cx": finite(msg.k[2]),
            "cy": finite(msg.k[5]),
            "distortion_model": msg.distortion_model,
            "frame_id": msg.header.frame_id,
            "stamp_ns": stamp_ns(msg.header.stamp),
        })

    def on_source_rates(self, msg: Float32MultiArray) -> None:
        rates = {
            name: finite(msg.data[index])
            for index, name in enumerate(SOURCE_RATE_NAMES)
            if index < len(msg.data)
        }
        self.record("source_rates", rates)

    def on_detections(self, msg: AprilTagDetectionArray) -> None:
        detections = []
        for item in msg.detections[:12]:
            detections.append({
                "id": int(item.id),
                "family": item.family,
                "hamming": int(item.hamming),
                "decision_margin": finite(item.decision_margin),
                "centre": [finite(item.centre.x), finite(item.centre.y)],
                "corners": [[finite(point.x), finite(point.y)] for point in item.corners],
            })
        self.record("detections", {
            "count": len(msg.detections),
            "items": detections,
            "frame_id": msg.header.frame_id,
            "stamp_ns": stamp_ns(msg.header.stamp),
        })

    def on_pose(self, msg: TargetRelativePose) -> None:
        self.record("pose", {
            "target_id": int(msg.target_id),
            "position": [finite(value) for value in msg.position],
            "q_wxyz": [finite(value) for value in msg.q],
            "position_valid": bool(msg.position_valid),
            "orientation_valid": bool(msg.orientation_valid),
            "timestamp_us": int(msg.timestamp),
            "timestamp_sample_us": int(msg.timestamp_sample),
        })

    def on_heartbeat(self, msg: OffboardControlMode) -> None:
        self.record("heartbeat", {
            "position_ready": bool(msg.position),
            "timestamp_us": int(msg.timestamp),
        })

    def on_vehicle_status(self, msg: VehicleStatus) -> None:
        names = {1: "未解锁", 2: "已解锁"}
        modes = {2: "POSCTL", 14: "OFFBOARD", 15: "STAB"}
        self.record("px4", {
            "system_id": int(msg.system_id),
            "arming_state": int(msg.arming_state),
            "arming_name": names.get(int(msg.arming_state), f"状态 {msg.arming_state}"),
            "nav_state": int(msg.nav_state),
            "mode_name": modes.get(int(msg.nav_state), f"模式 {msg.nav_state}"),
            "failsafe": bool(msg.failsafe),
            "gcs_connection_lost": bool(msg.gcs_connection_lost),
            "timestamp_us": int(msg.timestamp),
        })

    def refresh_tf(self) -> None:
        with self.lock:
            target_id = self.data.get("pose", {}).get("target_id", 1)
        target = TARGET_FRAMES.get(target_id, TARGET_FRAMES[1])
        tag = f"tag{target_id}" if target_id in TARGET_FRAMES else "tag1"
        edges = [
            ("机体 → 相机", "body_frd", "camera_link", True),
            ("相机 → 标签", "camera_link", tag, False),
            ("标签 → 目标机体", tag, target, True),
            ("端到端查询（前三条组合）", target, "body_frd", False),
        ]
        result = []
        now_ns = time.time_ns()
        for label, parent, child, static in edges:
            edge: dict[str, Any] = {"label": label, "parent": parent, "child": child}
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, Time())
                sample_ns = stamp_ns(transform.header.stamp)
                age_ms = None if sample_ns == 0 else round((now_ns - sample_ns) / 1e6, 1)
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                edge.update({
                    "available": True,
                    "fresh": bool(static or (age_ms is not None and 0 <= age_ms <= 200)),
                    "age_ms": age_ms,
                    "translation": [finite(translation.x), finite(translation.y), finite(translation.z)],
                    "q_xyzw": [finite(rotation.x), finite(rotation.y), finite(rotation.z), finite(rotation.w)],
                })
            except TransformException as exc:
                edge.update({"available": False, "fresh": False, "error": str(exc)[:150]})
            result.append(edge)
        with self.lock:
            self.data["tf"] = {"target_id": target_id, "edges": result}
            self.received["tf"] = time.monotonic()

    def update_system(self, value: dict[str, Any]) -> None:
        with self.lock:
            self.system = value

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self.lock:
            data = deepcopy(self.data)
            received = dict(self.received)
            samples = {name: list(entries) for name, entries in self.samples.items()}
            system = dict(self.system)
            preview = self.preview_jpeg is not None
        streams = {}
        for name in ("image", "camera_info", "detections", "pose", "heartbeat", "px4"):
            entries = [item for item in samples.get(name, []) if item >= now - 5]
            rate = (len(entries) - 1) / (entries[-1] - entries[0]) if len(entries) >= 2 else 0.0
            streams[name] = {
                "age_s": round(now - received[name], 2) if name in received else None,
                "rate_hz": round(rate, 1),
            }
        # The C++ relay continuously consumes all production topics and reports
        # their true source rates. Python receives lower-rate diagnostic copies.
        source_rates = data.get("source_rates", {})
        for name in ("camera_info", "detections", "pose", "heartbeat", "px4"):
            if source_rates.get(name) is not None:
                streams[name]["rate_hz"] = round(source_rates[name], 1)
        streams["image"] = {
            "age_s": round(now - received["source_rates"], 2) if "source_rates" in received else None,
            "rate_hz": round(source_rates.get("image", 0.0), 1),
        }
        streams["image"]["preview_age_s"] = (
            round(now - received["image"], 2) if "image" in received else None
        )
        return {
            "server_time_us": time.time_ns() // 1000,
            "streams": streams,
            "data": data,
            "system": system,
            "preview_available": preview,
        }

    def frame(self) -> bytes | None:
        with self.lock:
            return self.preview_jpeg


class DashboardServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, monitor: VisionMonitor) -> None:
        super().__init__((HOST, PORT), DashboardHandler)
        self.monitor = monitor
        self.csrf_token = secrets.token_urlsafe(32)
        self.restart_lock = threading.Lock()
        self.restart_job: dict[str, Any] = {"state": "idle"}

    def start_restart(self, force: bool) -> bool:
        with self.restart_lock:
            if self.restart_job["state"] == "running":
                return False
            self.restart_job = {
                "state": "running",
                "force": force,
                "started_at_us": time.time_ns() // 1000,
                "output": "正在执行安全检查并重启视觉链路…",
            }
        threading.Thread(target=self._run_restart, args=(force,), daemon=True).start()
        return True

    def _run_restart(self, force: bool) -> None:
        lines: list[str] = []
        code = 1
        state = "failed"

        def progress(message: str) -> None:
            lines.append(message)
            with self.restart_lock:
                self.restart_job["output"] = "\n".join(lines)

        try:
            snapshot = self.monitor.snapshot()
            px4 = snapshot["data"].get("px4")
            px4_stream = snapshot["streams"]["px4"]
            px4_fresh = bool(
                px4 and px4_stream["age_s"] is not None and px4_stream["age_s"] <= 1.0
                and abs(snapshot["server_time_us"] - px4["timestamp_us"]) <= 2_000_000
            )
            if px4_fresh:
                if px4["arming_state"] != VehicleStatus.ARMING_STATE_STANDBY:
                    progress(f"拒绝重启：PX4 arming_state={px4['arming_state']}，飞控不是未解锁状态。")
                    code = 2
                    return
                progress("安全检查通过：PX4 回传新鲜，飞控未解锁。")
            elif not force:
                progress("拒绝重启：无法取得新鲜 PX4 状态。请先确认飞机已落地且未解锁。")
                code = 2
                return
            else:
                progress("PX4 状态不可用；已按页面确认执行离线重启。")

            progress("正在重启 apriltag-stack.service…")
            result = subprocess.run(
                ["sudo", "-n", "/usr/bin/systemctl", "restart", SERVICE],
                capture_output=True, text=True, timeout=30, check=False,
            )
            if result.returncode != 0:
                progress(f"服务重启失败：{(result.stderr or result.stdout).strip()}")
                code = result.returncode
                return

            restarted_at = time.monotonic()
            required = ["camera_info", "detections", "pose", "heartbeat"]
            if px4_fresh:
                required.append("px4")
            progress("服务已重启，等待新的图像、检测、位姿、心跳和 PX4 回传…")
            deadline = restarted_at + 45
            missing: list[str] = required
            while time.monotonic() < deadline:
                now = time.monotonic()
                with self.monitor.lock:
                    received = dict(self.monitor.received)
                missing = [
                    name for name in required
                    if received.get(name, 0) <= restarted_at or now - received[name] > 2.0
                ]
                if not missing and command("systemctl", "is-active", SERVICE) == "active":
                    latest = self.monitor.snapshot()
                    pose = latest["data"].get("pose", {})
                    progress(
                        "链路恢复：相机、AprilTag、位姿桥、Offboard 心跳"
                        + ("、PX4 回传" if px4_fresh else "")
                        + "均已收到新样本。"
                    )
                    progress(
                        f"目标 ID={pose.get('target_id', '—')}，位置有效={pose.get('position_valid', False)}，"
                        f"姿态有效={pose.get('orientation_valid', False)}。"
                    )
                    if not px4_fresh:
                        progress("PX4 重启前未连接；需连接飞控后再确认 DDS/uORB 链路。")
                    state = "success"
                    code = 0
                    return
                time.sleep(0.5)
            progress(f"等待恢复超时：{', '.join(missing) if missing else '服务未保持 active'}。")
            logs = command("journalctl", "-u", SERVICE, "-n", "12", "--no-pager", timeout=3)
            if logs:
                progress("最近服务日志：\n" + logs[-6000:])
        except (OSError, subprocess.TimeoutExpired) as exc:
            progress(f"重启失败：{exc}")
        except Exception:
            logging.exception("Unexpected dashboard restart failure")
            progress("重启检查发生内部错误，请查看监控服务日志。")
        finally:
            with self.restart_lock:
                self.restart_job = {
                    "state": state,
                    "force": force,
                    "started_at_us": self.restart_job["started_at_us"],
                    "finished_at_us": time.time_ns() // 1000,
                    "exit_code": code,
                    "output": "\n".join(lines),
                }


class DashboardHandler(BaseHTTPRequestHandler):
    server: DashboardServer

    def log_message(self, format_string: str, *args: Any) -> None:
        logging.info("%s %s", self.address_string(), format_string % args)

    def allowed_host(self) -> bool:
        return self.headers.get("Host") in (f"{HOST}:{PORT}", f"localhost:{PORT}")

    def send_bytes(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Content-Security-Policy", "default-src 'self'; img-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; object-src 'none'; frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(body)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def send_json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_bytes(status, body, "application/json; charset=utf-8")

    def do_GET(self) -> None:
        if not self.allowed_host():
            self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid host"})
            return
        path = self.path.split("?", 1)[0]
        if path == "/api/health":
            self.send_json(HTTPStatus.OK, {"app": "vision-dashboard"})
        elif path == "/api/bootstrap":
            self.send_json(HTTPStatus.OK, {"csrf_token": self.server.csrf_token})
        elif path == "/api/status":
            snapshot = self.server.monitor.snapshot()
            with self.server.restart_lock:
                snapshot["restart"] = dict(self.server.restart_job)
            self.send_json(HTTPStatus.OK, snapshot)
        elif path == "/api/frame.jpg":
            frame = self.server.monitor.frame()
            if frame is None:
                self.send_json(HTTPStatus.SERVICE_UNAVAILABLE, {"error": "no camera frame"})
            else:
                self.send_bytes(HTTPStatus.OK, frame, "image/jpeg")
        elif path == "/api/logs":
            lines = command("journalctl", "-u", SERVICE, "-n", "35", "--no-pager", "--output=short-iso", timeout=3)
            self.send_json(HTTPStatus.OK, {"lines": lines[-14000:]})
        elif path in ("/", "/index.html", "/app.css", "/app.js"):
            filename = "index.html" if path == "/" else path.lstrip("/")
            content_type = {
                "index.html": "text/html; charset=utf-8",
                "app.css": "text/css; charset=utf-8",
                "app.js": "text/javascript; charset=utf-8",
            }[filename]
            self.send_bytes(HTTPStatus.OK, (ROOT / filename).read_bytes(), content_type)
        else:
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})

    def do_POST(self) -> None:
        if not self.allowed_host():
            self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid host"})
            return
        if self.path != "/api/restart":
            self.send_json(HTTPStatus.NOT_FOUND, {"error": "not found"})
            return
        origin = self.headers.get("Origin")
        if origin and origin not in (f"http://{HOST}:{PORT}", f"http://localhost:{PORT}"):
            self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid origin"})
            return
        if not secrets.compare_digest(self.headers.get("X-CSRF-Token", ""), self.server.csrf_token):
            self.send_json(HTTPStatus.FORBIDDEN, {"error": "invalid token"})
            return
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if length < 0 or length > 1024:
                raise ValueError("body too large")
            options = json.loads(self.rfile.read(length)) if length else {}
            if not isinstance(options, dict) or set(options) - {"force"} or not isinstance(options.get("force", False), bool):
                raise ValueError("invalid request")
        except (ValueError, json.JSONDecodeError):
            self.send_json(HTTPStatus.BAD_REQUEST, {"error": "invalid request"})
            return
        force = options.get("force", False)
        if not self.server.start_restart(force):
            self.send_json(HTTPStatus.CONFLICT, {"error": "视觉链路正在重启"})
            return
        self.send_json(HTTPStatus.ACCEPTED, {"message": "正在执行重启；结果会直接显示在页面内。"})


def poll_system(monitor: VisionMonitor, stop: threading.Event) -> None:
    while not stop.is_set():
        properties = command(
            "systemctl", "show", SERVICE, "-p", "ActiveState", "-p", "SubState",
            "-p", "NRestarts", "-p", "ExecMainStartTimestamp", timeout=2,
        )
        system = dict(line.split("=", 1) for line in properties.splitlines() if "=" in line)
        system["enabled"] = command("systemctl", "is-enabled", SERVICE, timeout=2)
        system["agent_running"] = bool(command("pgrep", "-x", "MicroXRCEAgent", timeout=1))
        system["restart_permission"] = bool(command(
            "sudo", "-n", "-l", "/usr/bin/systemctl", "restart", SERVICE, timeout=2,
        ))
        try:
            system["temperature_c"] = round(int(Path("/sys/class/thermal/thermal_zone0/temp").read_text().strip()) / 1000, 1)
        except (OSError, ValueError):
            system["temperature_c"] = None
        monitor.update_system(system)
        stop.wait(2)


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    os.environ.setdefault("ROS_DOMAIN_ID", "99")
    rclpy.init()
    monitor = VisionMonitor()
    executor = SingleThreadedExecutor()
    executor.add_node(monitor)
    stop = threading.Event()
    system_thread = threading.Thread(target=poll_system, args=(monitor, stop), name="system-poll", daemon=True)
    system_thread.start()
    try:
        with DashboardServer(monitor) as server:
            logging.info("Vision dashboard listening at http://%s:%d", HOST, PORT)
            http_thread = threading.Thread(target=server.serve_forever, name="http-server", daemon=True)
            http_thread.start()
            try:
                executor.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            finally:
                server.shutdown()
                http_thread.join(timeout=3)
    finally:
        stop.set()
        executor.shutdown()
        monitor.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
