"""Contract tests for the interactive bridge's published messages."""
import math

import pytest
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

from topmap_stage_demo import interactive as bridge


PATH = [(2.0, 3.5, 0.0), (10.0, 3.5, 0.0), (28.0, 3.5, 2.0)]


def test_final_path_markers_are_the_service_points_with_render_only_lift():
    array = bridge.build_path_markers(PATH)
    assert len(array.markers) == 1
    core = array.markers[0]
    assert core.type == Marker.LINE_STRIP
    assert len(core.points) == len(PATH)
    for point, expected in zip(core.points, PATH):
        assert point.x == pytest.approx(expected[0], abs=1e-12)
        assert point.y == pytest.approx(expected[1], abs=1e-12)
        assert point.z == pytest.approx(expected[2] + bridge.PATH_Z_LIFT, abs=1e-12)
    assert 0.0 < bridge.PATH_Z_LIFT <= 0.005
    assert core.scale.x >= 0.12


def test_nav_msgs_path_is_exactly_the_service_path_without_lift():
    message = bridge.build_path_message(PATH)
    assert isinstance(message, Path)
    assert message.header.frame_id == "map"
    assert len(message.poses) == len(PATH)
    for pose, expected in zip(message.poses, PATH):
        assert pose.pose.position.x == pytest.approx(expected[0], abs=1e-12)
        assert pose.pose.position.y == pytest.approx(expected[1], abs=1e-12)
        assert pose.pose.position.z == pytest.approx(expected[2], abs=1e-12)
        orientation = pose.pose.orientation
        assert (orientation.x, orientation.y, orientation.z, orientation.w) == (0.0, 0.0, 0.0, 1.0)


def test_failure_clear_uses_deleteall_and_an_empty_path():
    cleared = bridge.build_cleared_path_markers()
    assert len(cleared.markers) == 1
    assert cleared.markers[0].action == Marker.DELETEALL
    assert cleared.markers[0].header.frame_id == "map"
    empty = bridge.build_path_message([])
    assert len(empty.poses) == 0
    assert empty.header.frame_id == "map"


def test_status_marker_is_visible_and_explains_invalid_click():
    marker = bridge.build_status_marker("invalid", "INVALID CLICK: outside the mesh")
    assert marker.type == Marker.TEXT_VIEW_FACING
    assert marker.header.frame_id == "map"
    assert marker.action == Marker.ADD
    assert "INVALID" in marker.text
    assert marker.text.isascii()
    assert all(math.isfinite(v) for v in (marker.pose.position.x,
                                         marker.pose.position.y,
                                         marker.pose.position.z))
    orientation = marker.pose.orientation
    assert all(math.isfinite(v) for v in (orientation.x, orientation.y, orientation.z, orientation.w))


def test_start_marker_preserves_planar_yaw_with_finite_quaternion():
    array = bridge.build_start_markers((2.0, 3.5, 0.0), 1.25)
    spheres = [m for m in array.markers if m.type == Marker.SPHERE]
    arrows = [m for m in array.markers if m.type == Marker.ARROW]
    assert len(spheres) == 1 and len(arrows) == 1
    arrow = arrows[0]
    quaternion = arrow.pose.orientation
    assert all(math.isfinite(v) for v in (quaternion.x, quaternion.y, quaternion.z, quaternion.w))
    assert math.sqrt(
        quaternion.x ** 2 + quaternion.y ** 2 + quaternion.z ** 2 + quaternion.w ** 2
    ) == pytest.approx(1.0, abs=1e-12)
    assert quaternion.z == pytest.approx(math.sin(1.25 / 2.0), abs=1e-12)
    assert quaternion.w == pytest.approx(math.cos(1.25 / 2.0), abs=1e-12)


def test_goal_marker_and_invalid_marker_clear_shape():
    goal = bridge.build_goal_markers((28.0, 3.5, 2.0))
    spheres = [m for m in goal.markers if m.type == Marker.SPHERE]
    assert len(spheres) == 1
    assert spheres[0].pose.position.z == pytest.approx(2.0 + bridge.ENDPOINT_Z_LIFT)
