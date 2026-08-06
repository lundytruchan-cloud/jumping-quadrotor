# Copyright 2026 Larry
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Hybrid-locomotion phase state machine (paper drop/jump convention)."""

from enum import IntEnum


class LegPhase(IntEnum):
    AERIAL = 0
    LANDING = 1
    SUPPORT = 2
    TAKEOFF = 3


class PhaseStateMachine:
    """Deterministic AERIAL/LANDING/SUPPORT/TAKEOFF phase switching."""

    def __init__(self, foot_threshold=0.03, compression_eps=0.002):
        self.foot_threshold = foot_threshold
        self.compression_eps = compression_eps
        self.phase = LegPhase.AERIAL
        self._prev_foot_z = None

    def update(self, t, foot_z, q, q_dot):
        """Advance the state machine with one high-rate sample."""
        del t, q_dot  # reserved for diagnostics / future hysteresis tuning
        if self._prev_foot_z is None:
            self._prev_foot_z = foot_z
            return self.phase

        prev = self._prev_foot_z
        down_cross = prev > self.foot_threshold >= foot_z
        up_cross = prev <= self.foot_threshold < foot_z
        compressed = q < -self.compression_eps

        if self.phase == LegPhase.AERIAL:
            if down_cross:
                self.phase = (
                    LegPhase.SUPPORT if compressed else LegPhase.LANDING)
        elif self.phase == LegPhase.LANDING:
            if compressed:
                self.phase = LegPhase.SUPPORT
            elif up_cross:
                self.phase = LegPhase.AERIAL
        elif self.phase == LegPhase.SUPPORT:
            if not compressed:
                self.phase = LegPhase.TAKEOFF
        elif self.phase == LegPhase.TAKEOFF:
            if up_cross or foot_z > self.foot_threshold:
                self.phase = LegPhase.AERIAL

        self._prev_foot_z = foot_z
        return self.phase
