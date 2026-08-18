#!/usr/bin/env python3
"""End-to-end probe for hand_console angle ACK and tracking perf fields."""
from __future__ import annotations

import json
import select
import subprocess
import sys
import time
import unittest
from pathlib import Path


SIM = Path(__file__).resolve().parents[1]


class HandConsoleAckTest(unittest.TestCase):
    def test_mock_angle_ack_carries_correlation_and_tracking(self) -> None:
        process = subprocess.Popen(
            [sys.executable, str(SIM / "hand_console.py"), "--mock", "--hz", "30"],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        self.addCleanup(self._stop_process, process)
        ready = self._read_until(process, lambda row: row.get("type") == "ready")
        self.assertTrue(ready.get("mock"))

        enqueued_ns = time.perf_counter_ns()
        command = {
            "cmd": "angles",
            "rad": [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            "perf_id": 42,
            "_perf": {
                "id": 42,
                "ack_token": "test-token",
                "source": "mimic",
                "enqueued_ns": enqueued_ns,
            },
        }
        assert process.stdin is not None
        process.stdin.write(json.dumps(command) + "\n")
        process.stdin.flush()

        ack = self._read_until(
            process,
            lambda row: row.get("type") == "ack" and row.get("cmd") == "angles",
        )
        perf = ack["perf"]
        self.assertEqual(perf["id"], 42)
        self.assertEqual(perf["ack_token"], "test-token")
        self.assertEqual(perf["source"], "mimic")
        self.assertGreaterEqual(perf["stdin_queue_ms"], 0)
        self.assertGreaterEqual(perf["serial_ms"], 0)

        state = self._read_until(
            process,
            lambda row: (row.get("tracking_perf") or {}).get("id") == 42,
        )
        self.assertIn("mean_err_rad", state["tracking_perf"])
        self.assertIn("settled_ms", state["tracking_perf"])

    @staticmethod
    def _read_until(process, predicate, timeout: float = 3.0) -> dict:
        assert process.stdout is not None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            readable, _, _ = select.select([process.stdout], [], [], 0.1)
            if not readable:
                if process.poll() is not None:
                    break
                continue
            line = process.stdout.readline()
            if not line:
                break
            row = json.loads(line)
            if predicate(row):
                return row
        stderr = process.stderr.read() if process.poll() is not None else ""
        raise AssertionError(f"expected console event not received; stderr={stderr!r}")

    @staticmethod
    def _stop_process(process) -> None:
        try:
            if process.poll() is None:
                assert process.stdin is not None
                process.stdin.write('{"cmd":"quit"}\n')
                process.stdin.flush()
                process.wait(timeout=3)
        except Exception:  # noqa: BLE001
            if process.poll() is None:
                process.terminate()
                process.wait(timeout=3)
        finally:
            for stream in (process.stdin, process.stdout, process.stderr):
                if stream is not None:
                    stream.close()


if __name__ == "__main__":
    unittest.main()
