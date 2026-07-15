from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))

from scan_tf_pose_publisher import PoseSample, TfVelocityEstimator  # noqa: E402


def _estimator(**overrides) -> TfVelocityEstimator:
    options = {
        "smoothing_alpha": 1.0,
        "max_linear_speed": 2.0,
        "max_angular_speed": 3.0,
        "reset_gap_sec": 0.5,
    }
    options.update(overrides)
    return TfVelocityEstimator(**options)


def test_velocity_estimator_uses_consecutive_tf_samples():
    estimator = _estimator()
    assert estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, 0.0)) == (
        0.0,
        0.0,
        0.0,
        0.0,
    )

    velocity = estimator.update(PoseSample(1.2, 0.1, -0.04, 0.02, 0.2))

    assert velocity == pytest.approx((0.5, -0.2, 0.1, 1.0))


def test_velocity_estimator_handles_yaw_wraparound():
    estimator = _estimator()
    estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, math.pi - 0.1))

    velocity = estimator.update(PoseSample(1.2, 0.0, 0.0, 0.0, -math.pi + 0.1))

    assert velocity[3] == pytest.approx(1.0)


def test_velocity_estimator_rejects_tf_jump_and_stale_gap():
    estimator = _estimator()
    estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, 0.0))
    assert estimator.update(PoseSample(1.1, 1.0, 0.0, 0.0, 0.0)) == (
        0.0,
        0.0,
        0.0,
        0.0,
    )
    assert estimator.update(PoseSample(2.0, 1.0, 0.0, 0.0, 0.0)) == (
        0.0,
        0.0,
        0.0,
        0.0,
    )


def test_velocity_estimator_ignores_duplicate_tf_stamp():
    estimator = _estimator()
    estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, 0.0))
    expected = estimator.update(PoseSample(1.1, 0.1, 0.0, 0.0, 0.0))

    assert estimator.update(PoseSample(1.1, 99.0, 0.0, 0.0, 0.0)) == expected


def test_velocity_estimator_suppresses_linear_motion_while_execution_is_frozen():
    estimator = _estimator(smoothing_alpha=0.5)
    estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, 0.0))
    estimator.update(PoseSample(1.1, 0.02, 0.0, 0.0, 0.1))

    velocity = estimator.update(
        PoseSample(1.2, 0.06, -0.03, 0.0, 0.2),
        suppress_linear=True,
    )

    assert velocity[:3] == (0.0, 0.0, 0.0)
    assert velocity[3] == pytest.approx(0.75)


def test_velocity_estimator_does_not_accumulate_frozen_translation():
    estimator = _estimator()
    estimator.update(PoseSample(1.0, 0.0, 0.0, 0.0, 0.0))
    estimator.update(
        PoseSample(1.1, 0.04, -0.02, 0.0, 0.1),
        suppress_linear=True,
    )

    velocity = estimator.update(PoseSample(1.2, 0.05, -0.02, 0.0, 0.1))

    assert velocity == pytest.approx((0.1, 0.0, 0.0, 0.0))
