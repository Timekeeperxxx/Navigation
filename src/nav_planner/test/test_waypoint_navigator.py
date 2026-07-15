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


class _CapturePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _NavigatorHarness:
    load_waypoints = WaypointNavigator.load_waypoints
    reload_waypoints = WaypointNavigator.reload_waypoints
    nav_start_callback = WaypointNavigator.nav_start_callback
    waypoint_reached_callback = WaypointNavigator.waypoint_reached_callback
    publish_waypoint_context = WaypointNavigator.publish_waypoint_context

    def __init__(self, task_file: Path):
        self.waypoints_file = str(task_file)
        self.frame_id = "map"
        self.waypoints = []
        self.task_id = None
        self.task_name = None
        self.current_index = 9
        self.navigating = False
        self.retry_timer = None
        self.publish_count = 0
        self.task_statuses = []
        self.waypoint_context_pub = _CapturePublisher()
        self._logger = _Logger()

    def get_logger(self):
        return self._logger

    def publish_current_waypoint(self):
        self.publish_count += 1
        if self.navigating and self.current_index < len(self.waypoints):
            self.publish_waypoint_context(
                active=True,
                waypoint_index=self.current_index,
            )

    def publish_task_status(
        self,
        status,
        message,
        *,
        task_complete,
        waypoint_index=None,
    ):
        self.task_statuses.append(
            {
                "status": status,
                "message": message,
                "task_complete": task_complete,
                "waypoint_index": waypoint_index,
            }
        )


class _BoolMessage:
    def __init__(self, data: bool):
        self.data = data


def _write_task(path: Path, name: str, x: float) -> None:
    path.write_text(
        json.dumps(
            {
                "task_id": "task-1",
                "task_name": "巡检任务",
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
    assert navigator.task_statuses[-1]["status"] == "failed"
    assert navigator.task_statuses[-1]["task_complete"] is True


def test_intermediate_waypoint_keeps_task_running(tmp_path):
    task_file = tmp_path / "current_task.json"
    _write_task(task_file, "first", 1.0)
    payload = json.loads(task_file.read_text(encoding="utf-8"))
    payload["steps"].append(
        {
            "type": "navigate_waypoint",
            "waypoint_id": "wp-2",
            "waypoint_name": "second",
            "x": 8.0,
            "y": 2.0,
            "z": 0.3,
            "yaw": 0.4,
            "frame_id": "map",
        }
    )
    task_file.write_text(json.dumps(payload), encoding="utf-8")

    navigator = _NavigatorHarness(task_file)
    navigator.nav_start_callback(_BoolMessage(True))
    navigator.waypoint_reached_callback(_BoolMessage(True))

    assert navigator.navigating is True
    assert navigator.current_index == 1
    assert navigator.publish_count == 2
    assert navigator.task_statuses[-1]["status"] == "moving"
    assert navigator.task_statuses[-1]["task_complete"] is False
    first_context = json.loads(navigator.waypoint_context_pub.messages[0].data)
    next_context = json.loads(navigator.waypoint_context_pub.messages[-1].data)
    assert first_context["waypoint_index"] == 0
    assert first_context["is_final"] is False
    assert next_context["waypoint_index"] == 1
    assert next_context["is_final"] is True


def test_final_waypoint_publishes_task_complete(tmp_path):
    task_file = tmp_path / "current_task.json"
    _write_task(task_file, "only", 1.0)
    navigator = _NavigatorHarness(task_file)
    navigator.nav_start_callback(_BoolMessage(True))

    navigator.waypoint_reached_callback(_BoolMessage(True))

    assert navigator.navigating is False
    assert navigator.current_index == 1
    assert navigator.task_statuses[-1]["status"] == "reached"
    assert navigator.task_statuses[-1]["task_complete"] is True
    context = json.loads(navigator.waypoint_context_pub.messages[-1].data)
    assert context["active"] is False
