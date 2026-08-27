from ros_driver_state import DeviceHealth, DeviceState


def test_connect_failure_uses_capped_exponential_backoff():
    health = DeviceHealth("arm", retry_initial_s=1.0, retry_max_s=4.0)

    assert health.begin_connect(10.0)
    health.mark_fault("offline", 10.0)
    assert health.state is DeviceState.FAULT
    assert health.next_retry_at == 11.0
    assert not health.begin_connect(10.9)

    assert health.begin_connect(11.0)
    health.mark_fault("offline", 11.0)
    assert health.next_retry_at == 13.0

    assert health.begin_connect(13.0)
    health.mark_fault("offline", 13.0)
    assert health.next_retry_at == 17.0

    assert health.begin_connect(17.0)
    health.mark_fault("offline", 17.0)
    assert health.next_retry_at == 21.0


def test_success_resets_backoff_and_read_watchdog():
    health = DeviceHealth("hand", retry_initial_s=2.0, read_failure_limit=3)
    health.begin_connect(1.0)
    health.mark_fault("missing", 1.0)
    health.begin_connect(3.0)
    health.mark_ready(3.0)

    assert health.ready
    assert health.mark_read_failure("timeout") is False
    assert health.mark_read_failure("timeout") is False
    assert health.mark_read_failure("timeout") is True

    health.mark_ready(4.0)
    health.mark_read_failure("one lost frame")
    health.mark_read_success(4.1)
    assert health.read_failures == 0
    assert health.last_error is None

    health.mark_fault("unplugged", 5.0)
    assert health.next_retry_at == 7.0


def test_snapshot_reports_age_and_retry_delay():
    health = DeviceHealth("arm", retry_initial_s=1.5)
    health.begin_connect(2.0)
    health.mark_ready(2.0)
    assert health.snapshot(2.25)["last_success_age_s"] == 0.25

    health.mark_fault("usbip detached", 3.0)
    snap = health.snapshot(3.5)
    assert snap["state"] == "FAULT"
    assert snap["retry_in_s"] == 1.0
    assert snap["last_error"] == "usbip detached"


def test_shutdown_disables_reconnect():
    health = DeviceHealth("arm")
    health.mark_disconnected("shutdown")
    assert not health.retry_due(1e12)
    assert health.snapshot(100.0)["state"] == "DISCONNECTED"
