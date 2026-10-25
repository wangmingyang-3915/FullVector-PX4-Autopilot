"""Safety checks for the dashboard restart worker; no service is restarted."""

import threading
import time
import unittest
from types import SimpleNamespace
from unittest.mock import patch

import server


class FakeMonitor:
    def __init__(self, arming_state=1, connected=True):
        self.arming_state = arming_state
        self.connected = connected
        self.lock = threading.Lock()
        self.received = {}

    def snapshot(self):
        now = time.time_ns() // 1000
        return {
            "server_time_us": now,
            "data": {
                "px4": {"arming_state": self.arming_state, "timestamp_us": now} if self.connected else None,
                "pose": {"target_id": 1, "position_valid": True, "orientation_valid": True},
            },
            "streams": {"px4": {"age_s": 0.01 if self.connected else None}},
        }


def worker(monitor):
    dashboard = server.DashboardServer.__new__(server.DashboardServer)
    dashboard.monitor = monitor
    dashboard.restart_lock = threading.Lock()
    dashboard.restart_job = {"state": "running", "started_at_us": time.time_ns() // 1000}
    return dashboard


class RestartTests(unittest.TestCase):
    def test_armed_vehicle_is_never_restarted(self):
        dashboard = worker(FakeMonitor(arming_state=2))
        with patch.object(server.subprocess, "run", side_effect=AssertionError("must not restart")):
            dashboard._run_restart(force=True)
        self.assertEqual(dashboard.restart_job["state"], "failed")
        self.assertEqual(dashboard.restart_job["exit_code"], 2)
        self.assertIn("拒绝重启", dashboard.restart_job["output"])

    def test_unknown_vehicle_requires_explicit_force(self):
        dashboard = worker(FakeMonitor(connected=False))
        with patch.object(server.subprocess, "run", side_effect=AssertionError("must not restart")):
            dashboard._run_restart(force=False)
        self.assertEqual(dashboard.restart_job["state"], "failed")
        self.assertEqual(dashboard.restart_job["exit_code"], 2)

    def test_disarmed_vehicle_checks_new_streams(self):
        monitor = FakeMonitor()
        dashboard = worker(monitor)

        def fake_restart(args, **_kwargs):
            self.assertEqual(args, ["sudo", "-n", "/usr/bin/systemctl", "restart", "apriltag-stack.service"])
            with monitor.lock:
                monitor.received = {
                    name: time.monotonic() + 1.0
                    for name in ("camera_info", "detections", "pose", "heartbeat", "px4")
                }
            return SimpleNamespace(returncode=0, stdout="", stderr="")

        with patch.object(server.subprocess, "run", side_effect=fake_restart), \
                patch.object(server, "command", return_value="active"):
            dashboard._run_restart(force=False)
        self.assertEqual(dashboard.restart_job["state"], "success")
        self.assertEqual(dashboard.restart_job["exit_code"], 0)
        self.assertIn("均已收到新样本", dashboard.restart_job["output"])


if __name__ == "__main__":
    unittest.main()
