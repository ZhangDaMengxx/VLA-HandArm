from __future__ import annotations

import threading

from hardware_lease import HardwareLease


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def test_acquire_heartbeat_and_busy_owner():
    clock = Clock()
    lease = HardwareLease(ttl_s=8, clock=clock)
    first = lease.acquire("tab-a")
    assert first.ok and first.owner == "tab-a"
    assert lease.acquire("tab-b").reason == "owner_busy"
    renewed = lease.heartbeat("tab-a")
    assert renewed.ok and renewed.expires_at == 8.0
    assert lease.heartbeat("tab-b").reason == "not_owner"


def test_takeover_replaces_owner_and_reports_previous_owner():
    lease = HardwareLease(ttl_s=8)
    assert lease.acquire("tab-a").ok
    result = lease.acquire("tab-b", takeover=True)
    assert result.ok
    assert result.owner == "tab-b"
    assert result.previous_owner == "tab-a"
    assert lease.heartbeat("tab-a").reason == "not_owner"
    assert lease.heartbeat("tab-b").ok


def test_expiry_can_be_claimed_once_and_new_owner_can_acquire():
    clock = Clock()
    lease = HardwareLease(ttl_s=2, clock=clock)
    lease.acquire("tab-a")
    clock.now = 2.0
    assert lease.expired() == "tab-a"
    assert lease.expired() is None
    assert lease.acquire("tab-b").ok


def test_late_heartbeat_does_not_hide_expiry_from_watchdog():
    clock = Clock()
    lease = HardwareLease(ttl_s=2, clock=clock)
    lease.acquire("tab-a")
    clock.now = 2.0
    assert lease.heartbeat("tab-a").reason == "lease_expired"
    assert lease.expired() == "tab-a"


def test_release_is_idempotent_only_for_owner():
    lease = HardwareLease(ttl_s=8)
    assert lease.release("missing").ok
    assert lease.acquire("tab-a").ok
    assert lease.release("tab-b").reason == "not_owner"
    assert lease.release("tab-a").ok
    assert lease.release("tab-a").ok


def test_concurrent_release_does_not_corrupt_state():
    lease = HardwareLease(ttl_s=8)
    lease.acquire("tab-a")
    results = []

    def release():
        results.append(lease.release("tab-a").ok)

    threads = [threading.Thread(target=release) for _ in range(8)]
    for thread in threads:
        thread.start()
    for thread in threads:
        thread.join()
    assert all(results)
    assert lease.owner is None
