"""Exercise actual ROS input types and a mesh containing skipped vertical faces."""
import pytest
from geometry_msgs.msg import Quaternion
from builtin_interfaces.msg import Time
from concurrent.futures import Future
from types import SimpleNamespace
from topmap_msgs.srv import PlanPath
from unittest.mock import Mock
from visualization_msgs.msg import Marker

from topmap_stage_demo.interactive import InteractivePlannerNode
from topmap_stage_demo.interactive_state import ReplanCoordinator, sanitize_planar_orientation
from topmap_stage_demo.surface_projection import SurfaceMesh


def test_rviz_ros_quaternion_is_accepted():
    # PoseWithCovarianceStamped supplies this message, not an iterable tuple.
    orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    assert sanitize_planar_orientation(orientation) == pytest.approx(0.0)


def test_vertical_face_before_clickable_face_preserves_triangle_identity():
    mesh = SurfaceMesh(
        [[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0],
         [2.0, 0.0, 2.0], [3.0, 0.0, 2.0], [2.0, 1.0, 2.0]],
        [[0, 1, 2], [3, 4, 5]],
    )
    point = mesh.project(2.2, 0.2)
    assert point.as_tuple() == pytest.approx((2.2, 0.2, 2.0))
    assert point.triangle_id == 1


def test_invalidating_path_also_clears_previous_topology_sequence():
    bridge = object.__new__(InteractivePlannerNode)
    bridge._stamp = lambda: Time()
    bridge._publishers = {key: Mock() for key in ('path', 'nav_path', 'topology')}
    bridge._publish_clear()
    published = bridge._publishers['topology'].publish
    assert published.called
    assert published.call_args.args[0].markers[0].action == Marker.DELETEALL


def test_late_response_cannot_release_a_newer_inflight_request():
    bridge = object.__new__(InteractivePlannerNode)
    bridge._coord = ReplanCoordinator(revision=2)
    bridge._inflight_token = 2
    bridge._node = SimpleNamespace(get_logger=lambda: Mock())
    future = Future()
    future.set_result(PlanPath.Response(success=False, error_message='old request failed'))
    bridge._on_plan_done(1, future)
    assert bridge._inflight_token == 2
