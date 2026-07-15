import json
import sys
from pathlib import Path


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "script"))

from waypoint_navigator_from_json import WaypointNavigator  # noqa: E402


class _Logger:
    def info(self, _message):
        pass

    def error(self, _message):
        pass

    def warning(self, _message):
        pass


class _NavigatorHarness:
    load_waypoints = WaypointNavigator.load_waypoints
    reload_waypoints = WaypointNavigator.reload_waypoints
    nav_start_callback = WaypointNavigator.nav_start_callback

    def __init__(self, task_file: Path):
        self.waypoints_file = str(task_file)
        self.frame_id = "map"
        self.waypoints = []
        self.current_index = 9
        self.navigating = False
        self.retry_timer = None
        self.publish_count = 0
        self._logger = _Logger()

    def get_logger(self):
        return self._logger

    def publish_current_waypoint(self):
        self.publish_count += 1


class _BoolMessage:
    def __init__(self, data: bool):
        self.data = data


def _write_task(path: Path, name: str, x: float) -> None:
    path.write_text(
        json.dumps(
            {
                "frame_id": "map",
                "steps": [
                    {
                        "type": "navigate_waypoint",
                        "waypoint_name": name,
                        "x": x,
                        "y": 2.0,
                        "z": 0.3,
                        "yaw": 0.4,
                        "frame_id": "map",
                    }
                ],
            }
        ),
        encoding="utf-8",
    )


def test_nav_start_reloads_latest_runtime_task(tmp_path):
    task_file = tmp_path / "current_task.json"
    _write_task(task_file, "first", 1.0)
    navigator = _NavigatorHarness(task_file)
    assert navigator.reload_waypoints() is True
    assert navigator.waypoints[0]["name"] == "first"

    _write_task(task_file, "second", 8.0)
    navigator.nav_start_callback(_BoolMessage(True))

    assert navigator.navigating is True
    assert navigator.current_index == 0
    assert navigator.waypoints[0]["name"] == "second"
    assert navigator.waypoints[0]["x"] == 8.0
    assert navigator.publish_count == 1


def test_nav_start_rejects_missing_runtime_task(tmp_path):
    navigator = _NavigatorHarness(tmp_path / "missing.json")

    navigator.nav_start_callback(_BoolMessage(True))

    assert navigator.navigating is False
    assert navigator.publish_count == 0
