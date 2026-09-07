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

"""Unit tests for the aerial/landing/support/takeoff phase state machine."""

from quadcopter_description.phase_machine import LegPhase, PhaseStateMachine


def test_starts_in_aerial():
    sm = PhaseStateMachine()
    assert sm.phase == LegPhase.AERIAL
    assert sm.update(0.0, 0.50, 0.0, 0.0) == LegPhase.AERIAL


def test_landing_to_support():
    sm = PhaseStateMachine()
    sm.update(0.00, 0.50, 0.0, 0.0)
    # Foot crosses the 3 cm threshold downwards -> LANDING.
    assert sm.update(0.31, 0.02, 0.0, -3.0) == LegPhase.LANDING
    # Leg starts compressing -> SUPPORT.
    assert sm.update(0.311, 0.00, -0.005, -2.0) == LegPhase.SUPPORT


def test_support_to_takeoff_to_aerial():
    sm = PhaseStateMachine()
    sm.update(0.00, 0.50, 0.0, 0.0)
    sm.update(0.31, 0.00, 0.0, -3.0)
    sm.update(0.312, 0.00, -0.006, -1.0)
    # Leg returns to rest length -> TAKEOFF.
    assert sm.update(0.344, 0.00, 0.0, 3.0) == LegPhase.TAKEOFF
    # Foot rises above the threshold -> AERIAL.
    assert sm.update(0.36, 0.05, 0.0, 3.0) == LegPhase.AERIAL


def test_full_cycle_sequence():
    sm = PhaseStateMachine()
    samples = [
        (0.00, 0.50, 0.000, 0.0),
        (0.30, 0.02, 0.000, -3.0),
        (0.31, 0.00, -0.006, -2.0),
        (0.34, 0.00, -0.001, 2.0),
        (0.35, 0.00, 0.000, 3.0),
        (0.37, 0.05, 0.000, 3.0),
    ]
    phases = [sm.update(*sample) for sample in samples]
    assert phases == [
        LegPhase.AERIAL,
        LegPhase.LANDING,
        LegPhase.SUPPORT,
        LegPhase.TAKEOFF,
        LegPhase.TAKEOFF,
        LegPhase.AERIAL,
    ]


def test_support_holds_with_foot_near_threshold():
    sm = PhaseStateMachine()
    sm.update(0.00, 0.50, 0.0, 0.0)
    sm.update(0.30, 0.02, 0.0, -3.0)
    sm.update(0.31, 0.00, -0.006, -2.0)
    # Foot height jitters around the threshold during support.
    for t, z, q, qd in [
        (0.32, 0.02, -0.004, -0.5),
        (0.33, 0.04, -0.003, 0.0),
        (0.34, 0.01, -0.0025, 1.0),
    ]:
        assert sm.update(t, z, q, qd) == LegPhase.SUPPORT
    # Only returning to the rest length ends support.
    assert sm.update(0.35, 0.01, 0.0, 2.0) == LegPhase.TAKEOFF


def test_phase_names():
    assert LegPhase.AERIAL.name == 'AERIAL'
    assert LegPhase.LANDING.name == 'LANDING'
    assert LegPhase.SUPPORT.name == 'SUPPORT'
    assert LegPhase.TAKEOFF.name == 'TAKEOFF'
