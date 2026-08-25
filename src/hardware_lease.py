"""Server-side ownership lease for browser-controlled hardware sessions."""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass


@dataclass(frozen=True)
class LeaseResult:
    ok: bool
    reason: str | None = None
    owner: str | None = None
    expires_at: float | None = None
    previous_owner: str | None = None


class HardwareLease:
    """Small thread-safe lease registry.

    The lease is deliberately independent from a WebSocket: a browser can have
    several sockets and can disappear without a close frame. Callers poll
    ``expired`` from their server task and perform hardware cleanup themselves.
    """

    def __init__(self, ttl_s: float = 8.0, clock=time.monotonic) -> None:
        if ttl_s <= 0:
            raise ValueError("ttl_s must be positive")
        self.ttl_s = float(ttl_s)
        self._clock = clock
        self._lock = threading.Lock()
        self._owner: str | None = None
        self._expires_at: float | None = None

    @property
    def owner(self) -> str | None:
        with self._lock:
            return self._owner

    @property
    def expires_at(self) -> float | None:
        with self._lock:
            return self._expires_at

    def acquire(self, owner: str, *, takeover: bool = False) -> LeaseResult:
        owner = str(owner or "").strip()
        if not owner:
            return LeaseResult(False, "owner_required")
        with self._lock:
            expired = self._is_expired_locked()
            if expired and not takeover:
                return LeaseResult(False, "lease_expired", self._owner,
                                   self._expires_at)
            if self._owner not in (None, owner) and not takeover:
                return LeaseResult(False, "owner_busy", self._owner, self._expires_at)
            previous_owner = self._owner if takeover and (
                expired or self._owner not in (None, owner)
            ) else None
            self._owner = owner
            self._expires_at = self._clock() + self.ttl_s
            return LeaseResult(True, owner=owner, expires_at=self._expires_at,
                               previous_owner=previous_owner)

    def heartbeat(self, owner: str) -> LeaseResult:
        owner = str(owner or "").strip()
        with self._lock:
            if self._is_expired_locked():
                return LeaseResult(False, "lease_expired", self._owner,
                                   self._expires_at)
            if not owner or owner != self._owner:
                return LeaseResult(False, "not_owner", self._owner, self._expires_at)
            self._expires_at = self._clock() + self.ttl_s
            return LeaseResult(True, owner=owner, expires_at=self._expires_at)

    def release(self, owner: str | None) -> LeaseResult:
        owner = str(owner or "").strip()
        with self._lock:
            if self._owner is None:
                return LeaseResult(True, owner=owner or None)
            if owner != self._owner:
                return LeaseResult(False, "not_owner", self._owner, self._expires_at)
            self._owner = None
            self._expires_at = None
            return LeaseResult(True, owner=owner)

    def expired(self) -> str | None:
        """Atomically claim and clear an expired owner, if any."""
        with self._lock:
            if self._owner is None or self._expires_at is None:
                return None
            if self._clock() < self._expires_at:
                return None
            owner = self._owner
            self._owner = None
            self._expires_at = None
            return owner

    def _is_expired_locked(self) -> bool:
        return bool(self._owner is not None and self._expires_at is not None
                    and self._clock() >= self._expires_at)
