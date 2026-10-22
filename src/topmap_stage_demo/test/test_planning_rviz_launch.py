"""Parse the planning RViz view and assert the one-command interactive contract."""
import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.normpath(os.path.join(HERE, ".."))
RVIZ = os.path.join(PKG, "rviz", "planning.rviz")
LAUNCH = os.path.join(PKG, "launch", "planning.launch.py")
REPLAY = os.path.join(PKG, "launch", "replay.launch.py")
STAGES_LAUNCH = os.path.join(PKG, "launch", "stages.launch.py")
PLANNING_JSON = os.path.join(PKG, "data", "planning.json")
ARCHIVE = os.path.join(PKG, "data", "stages.topmap.bin")


def _load():
    with open(RVIZ, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def _collect_topics(displays, topics=None):
    if topics is None:
        topics = {}
    for display in displays:
        if "Displays" in display:
            _collect_topics(display["Displays"], topics)
        elif "Topic" in display:
            topics[display["Topic"]["Value"]] = display
    return topics


def _tools(config):
    return {tool["Class"]: tool for tool in config["Visualization Manager"]["Tools"]}


def test_planning_rviz_is_a_clean_interactive_view():
    config = _load()
    manager = config["Visualization Manager"]
    assert manager["Global Options"]["Fixed Frame"] == "map"
    topics = _collect_topics(manager["Displays"])

    # Required default layers, including the visible interactive status.
    for topic in (
        "/topmap_stage_demo/source_cloud",
        "/topmap_stage_demo/stage3_final_regions",
        "/topmap_stage_demo/stage4_portal_chords",
        "/topmap_stage_demo/planning_start",
        "/topmap_stage_demo/planning_goal",
        "/topmap_stage_demo/final_path",
        "/topmap_stage_demo/planning_status",
    ):
        assert topic in topics, topic
        assert topics[topic]["Enabled"] is True, topic

    # The optional topology sequence is present but off by default.
    assert topics["/topmap_stage_demo/planned_topology_sequence"]["Enabled"] is False
    # nav_msgs/Path mirror is available for inspection but the bright Marker is
    # the primary visual.
    assert topics["/topmap_stage_demo/planned_path"]["Enabled"] is False
    # The render-only surface projection is optional and must be off by default;
    # it is not the planner final path.
    trace_display = topics["/topmap_stage_demo/surface_trace_reference"]
    assert trace_display["Enabled"] is False
    assert "render-only" in trace_display["Name"] or "参考线" in trace_display["Name"]

    # Overlapping / dense layers from the four-stage view are not part of this
    # default planning screenshot.
    for hidden in (
        "/topmap_stage_demo/stage1_triangle_mesh",
        "/topmap_stage_demo/stage1_triangle_edges",
        "/topmap_stage_demo/stage2_grown_regions",
    ):
        assert hidden not in topics or topics[hidden]["Enabled"] is False

    for topic, display in topics.items():
        if topic == "/clicked_point":
            continue
        inner = display["Topic"]
        assert inner["Depth"] == 1
        assert inner["Reliability Policy"] == "Reliable"
        assert inner["Durability Policy"] == "Transient Local"


def test_rviz_toolbar_has_standard_start_and_goal_tools():
    tools = _tools(_load())
    assert "rviz_default_plugins/SetInitialPose" in tools
    assert tools["rviz_default_plugins/SetInitialPose"]["Topic"]["Value"] == "/topmap_stage_demo/initialpose"
    assert "rviz_default_plugins/SetGoal" in tools
    assert tools["rviz_default_plugins/SetGoal"]["Topic"]["Value"] == "/topmap_stage_demo/goal_pose"
    for tool in tools.values():
        if "Topic" in tool:
            assert tool["Topic"]["Reliability Policy"] == "Reliable"
            assert tool["Topic"]["Durability Policy"] == "Volatile"


def test_planning_camera_frames_the_whole_ramp_scene():
    config = _load()
    view = config["Visualization Manager"]["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/Orbit"
    assert view["Distance"] >= 30
    assert view["Focal Point"]["X"] == pytest.approx(15.2, abs=1.0)
    assert view["Focal Point"]["Y"] == pytest.approx(3.5, abs=1.0)
    assert view["Focal Point"]["Z"] == pytest.approx(0.8, abs=0.8)
    assert config["Window Geometry"]["Width"] >= 1200
    assert config["Window Geometry"]["Height"] >= 800


def test_one_command_launch_defaults_to_interactive_and_keeps_replay_opt_in():
    for path in (RVIZ, LAUNCH, REPLAY, STAGES_LAUNCH, PLANNING_JSON, ARCHIVE):
        assert os.path.isfile(path), path
    with open(LAUNCH, "r", encoding="utf-8") as handle:
        text = handle.read()
    # Interactive is the default and is the only branch starting the backend.
    assert 'default_value="true"' in text
    assert "planning_publisher" in text
    assert "UnlessCondition(interactive)" in text
    assert "topmap_ros" in text
    assert "planner_node" in text
    assert "interactive_planner" in text
    assert "stages.topmap.bin" in text
    assert "/topmap_stage_demo/plan_path" in text
    assert '"--service", service_name' in text
    assert "planning.json" in text
    assert "planning.rviz" in text
    assert "stages.json" in text
    assert "scene_h_ground_original_ramp.pcd" in text

    with open(REPLAY, "r", encoding="utf-8") as handle:
        replay_text = handle.read()
    assert "planning_publisher" in replay_text
    assert "planner_node" not in replay_text
    assert "interactive_planner" not in replay_text

    with open(os.path.join(PKG, "setup.py"), "r", encoding="utf-8") as handle:
        setup_text = handle.read()
    assert "planning_publisher = topmap_stage_demo.planning:main" in setup_text
    assert "interactive_planner = topmap_stage_demo.interactive:main" in setup_text
    assert "glob('data/*')" in setup_text


def test_planning_resources_are_installed_with_the_package():
    try:
        share = get_package_share_directory("topmap_stage_demo")
    except Exception:
        pytest.skip("package not installed yet")
    for relative in (
        "launch/planning.launch.py",
        "launch/replay.launch.py",
        "rviz/planning.rviz",
        "data/planning.json",
        "data/stages.json",
        "data/stages.topmap.bin",
        "data/scene_h_ground_original_ramp.pcd",
    ):
        assert os.path.isfile(os.path.join(share, relative)), relative
