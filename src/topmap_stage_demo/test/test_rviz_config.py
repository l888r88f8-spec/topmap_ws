"""Parse the RViz layer configuration and the launch file contract."""
import os

import pytest
import yaml
from ament_index_python.packages import get_package_share_directory

HERE = os.path.dirname(os.path.abspath(__file__))
RVIZ = os.path.normpath(os.path.join(HERE, "..", "rviz", "stages.rviz"))

GROUP_NAMES = [
    "原始点云",
    "三角化(表面+网格边)",
    "区域合并(初始3区域)",
    "结构细分(最终5区域)",
    "拓扑图(links/nodes)",
    "通道(red chords/nodes, 可选 actual chain)",
    "标签(可选, 默认关闭)",
]


def _load():
    with open(RVIZ, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def test_rviz_config_parses_with_expected_groups_and_topics():
    config = _load()
    manager = config["Visualization Manager"]
    names = [display.get("Name") for display in manager["Displays"]]
    for name in GROUP_NAMES:
        assert name in names, name
    assert manager["Global Options"]["Fixed Frame"] == "map"

    topics = {}

    def collect(displays):
        for display in displays:
            if "Displays" in display:
                collect(display["Displays"])
            elif "Topic" in display:
                topics[display["Topic"]["Value"]] = display

    collect(manager["Displays"])
    assert topics["/topmap_stage_demo/source_cloud"]["Enabled"] is True
    for topic, display in topics.items():
        if topic == "/clicked_point":
            continue
        inner = display["Topic"]
        assert inner["Depth"] == 1
        assert inner["Reliability Policy"] == "Reliable"
        assert inner["Durability Policy"] == "Transient Local"
    # optional clutter layers off, everything else on
    assert topics["/topmap_stage_demo/stage4_boundary_chains"]["Enabled"] is False
    assert topics["/topmap_stage_demo/region_labels"]["Enabled"] is False
    assert topics["/topmap_stage_demo/portal_labels"]["Enabled"] is False


def test_rviz_camera_covers_single_scene():
    config = _load()
    view = config["Visualization Manager"]["Views"]["Current"]
    assert view["Class"] == "rviz_default_plugins/Orbit"
    assert view["Distance"] == pytest.approx(36)
    assert view["Pitch"] == pytest.approx(1.2)
    assert view["Yaw"] == pytest.approx(4.71238898)
    assert view["Focal Point"]["X"] == pytest.approx(15.2)
    assert view["Focal Point"]["Y"] == pytest.approx(3.5)
    assert view["Focal Point"]["Z"] == pytest.approx(0.8)
    geometry = config["Window Geometry"]
    assert geometry["Hide Right Dock"] is True
    assert geometry["Width"] == 1500
    assert geometry["Height"] == 900


def test_package_installed_paths():
    try:
        share = get_package_share_directory("topmap_stage_demo")
    except Exception:
        pytest.skip("package not installed yet")
    for relative in ("launch/stages.launch.py", "rviz/stages.rviz",
                     "data/stages.json", "data/scene_h_ground_original_ramp.pcd"):
        assert os.path.isfile(os.path.join(share, relative)), relative
