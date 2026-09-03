#!/usr/bin/env python3

import math


INITIALIZING = 0
HEALTHY = 1
DEGRADED = 2
RELOCALIZING = 3
LOST = 4

REASON_NONE = 0
REASON_QUALITY = 1 << 0
REASON_AMBIGUOUS = 1 << 1
REASON_DEGENERATE = 1 << 2
REASON_STALE_INPUT = 1 << 3
REASON_INTERNAL = 1 << 4


class LocalizationHealthTracker:
    """Pure fail-closed state for fixed-map correction ownership."""

    def __init__(self, max_consecutive_rejections):
        if (not isinstance(max_consecutive_rejections, int) or
                isinstance(max_consecutive_rejections, bool) or
                max_consecutive_rejections <= 0):
            raise ValueError('max_consecutive_rejections must be a positive integer')
        self.max_consecutive_rejections = max_consecutive_rejections
        self.sequence = 0
        self.state = INITIALIZING
        self.reason_mask = REASON_NONE
        self.consecutive_rejections = 0
        self.last_accepted_stamp_ns = 0
        self.last_fitness = math.nan
        self.last_rmse = math.nan

    def accepted(self, stamp_ns, fitness, rmse):
        fitness = float(fitness)
        rmse = float(rmse)
        if (not isinstance(stamp_ns, int) or stamp_ns <= 0 or
                not math.isfinite(fitness) or not math.isfinite(rmse)):
            self.rejected(REASON_INTERNAL)
            return False
        self.sequence += 1
        self.last_fitness = fitness
        self.last_rmse = rmse
        if self.state not in (INITIALIZING, HEALTHY):
            return False
        self.state = HEALTHY
        self.reason_mask = REASON_NONE
        self.consecutive_rejections = 0
        self.last_accepted_stamp_ns = stamp_ns
        return True

    def rejected(self, reason_mask):
        self.sequence += 1
        self.reason_mask |= int(reason_mask)
        self.consecutive_rejections += 1
        if (self.state == HEALTHY and
                self.consecutive_rejections >= self.max_consecutive_rejections):
            self.state = DEGRADED
        return self.state

    def reinitialize(self):
        self.sequence += 1
        self.state = INITIALIZING
        self.reason_mask = REASON_NONE
        self.consecutive_rejections = 0
        self.last_accepted_stamp_ns = 0
        self.last_fitness = math.nan
        self.last_rmse = math.nan


def map_output_allowed(require_health, health_state, last_receipt_seconds,
                       now_seconds, timeout_seconds, expected_map_id='',
                       health_map_id=''):
    """Return whether map-frame TF/localization may be emitted."""
    if not require_health:
        return True
    if (not math.isfinite(timeout_seconds) or timeout_seconds <= 0.0 or
            not math.isfinite(now_seconds)):
        raise ValueError('health timeout and current time must be finite and positive')
    if health_state != HEALTHY or last_receipt_seconds is None:
        return False
    if expected_map_id and health_map_id != expected_map_id:
        return False
    if not math.isfinite(last_receipt_seconds):
        return False
    age = now_seconds - last_receipt_seconds
    return 0.0 <= age < timeout_seconds
