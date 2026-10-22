"""Regression tests for stale service results and failure/clear semantics."""
import math

import pytest

from topmap_stage_demo.interactive_state import (
    DEFAULT_GOAL,
    DEFAULT_START,
    ReplanCoordinator,
    quaternion_from_yaw,
    sanitize_planar_orientation,
)


def test_defaults_are_the_existing_safe_demo_endpoints():
    coordinator = ReplanCoordinator()
    assert coordinator.start == pytest.approx(DEFAULT_START)
    assert coordinator.goal == pytest.approx(DEFAULT_GOAL)
    assert coordinator.status == "waiting"
    assert coordinator.last_path is None


def test_endpoint_change_discards_an_inflight_old_result():
    coordinator = ReplanCoordinator()
    stale_token = coordinator.begin_plan("default")
    coordinator.set_goal((28.0, 2.5, 2.0))
    assert not coordinator.accept_path(stale_token, [(2, 3.5, 0), (28.0, 2.5, 2.0)])
    assert coordinator.last_path is None

    fresh_token = coordinator.begin_plan("changed")
    assert coordinator.accept_path(fresh_token, [(2, 3.5, 0), (28.0, 2.5, 2.0)])
    assert coordinator.last_path == [(2.0, 3.5, 0.0), (28.0, 2.5, 2.0)]
    assert coordinator.status == "success"


def test_stale_failure_cannot_clear_a_newer_success():
    coordinator = ReplanCoordinator()
    old = coordinator.begin_plan("old")
    coordinator.set_start((3.0, 2.0, 0.0))
    new = coordinator.begin_plan("new")
    assert coordinator.accept_path(new, [(3.0, 2.0, 0.0), (28.0, 3.5, 2.0)])

    assert not coordinator.fail(old, "old request failed")
    assert coordinator.last_path is not None
    assert coordinator.status == "success"


def test_latest_failure_clears_the_previous_success_path():
    coordinator = ReplanCoordinator()
    token = coordinator.begin_plan("first")
    assert coordinator.accept_path(token, [(2, 3.5, 0), (28, 3.5, 2)])
    token = coordinator.begin_plan("second")
    assert coordinator.fail(token, "planner service refused the start")
    assert coordinator.last_path is None
    assert coordinator.status == "failed"
    assert "refused" in coordinator.status_text


def test_invalid_click_invalidates_inflight_and_requests_clear():
    coordinator = ReplanCoordinator()
    token = coordinator.begin_plan("in flight")
    before = coordinator.clear_serial
    revision = coordinator.invalidate("click is outside the map")
    assert revision > 0
    assert coordinator.clear_serial > before
    assert not coordinator.accept_path(token, [(0, 0, 0), (1, 1, 1)])
    assert coordinator.status == "invalid"
    assert coordinator.last_path is None
    assert "outside" in coordinator.status_text


def test_planar_yaw_is_preserved_but_quaternion_is_finite_and_normalized():
    for yaw in (-math.pi, -0.7, 0.0, 1.25, math.pi):
        quaternion = quaternion_from_yaw(yaw)
        assert all(math.isfinite(value) for value in quaternion)
        assert math.sqrt(sum(value * value for value in quaternion)) == pytest.approx(
            1.0, abs=1e-12
        )

    quaternion = quaternion_from_yaw(1.25)
    recovered = sanitize_planar_orientation(quaternion)
    assert recovered == pytest.approx(1.25, abs=1e-9)

    for bad in (
        (math.nan, 0.0, 0.0, 1.0),
        (0.0, math.inf, 0.0, 1.0),
        (0.0, 0.0, 0.0, 0.0),
    ):
        with pytest.raises(ValueError, match="orientation"):
            sanitize_planar_orientation(bad)
