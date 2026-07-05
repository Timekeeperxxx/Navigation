from __future__ import annotations

import argparse
import json
import signal
import sys
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

CURRENT_DIR = Path(__file__).resolve().parent
if str(CURRENT_DIR) not in sys.path:
    sys.path.insert(0, str(CURRENT_DIR))

from process_manager import ProcessManager  # noqa: E402
from ros_bridge import RosBridge  # noqa: E402


class TestAppState:
    """HTTP 层共享状态。"""

    def __init__(self, workspace_dir: Path) -> None:
        self.workspace_dir = workspace_dir.resolve()
        self.frontend_dir = self.workspace_dir / "tools" / "test_app" / "frontend"
        self.process_manager = ProcessManager(self.workspace_dir)
        self.ros_bridge = RosBridge()

    def start(self) -> None:
        self.ros_bridge.start()

    def stop(self) -> None:
        self.ros_bridge.stop()

    def snapshot(self) -> dict[str, Any]:
        return {
            "ok": True,
            "timestamp": time.time(),
            "workspace_dir": str(self.workspace_dir),
            "ros": self.ros_bridge.snapshot(),
            "runtime": self.process_manager.status(),
        }


class NavigationTestHandler(BaseHTTPRequestHandler):
    """轻量测试前后端 HTTP 处理器。"""

    server_version = "NavigationTestApp/0.1"

    def do_OPTIONS(self) -> None:  # noqa: N802
        self._send_json({"ok": True})

    def do_GET(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == "/api/status":
                self._send_json(self.app_state.snapshot())
                return
            if path == "/api/maps":
                self._send_json({"maps": self.app_state.process_manager.list_maps()})
                return
            if path == "/api/pcd-view":
                query = parse_qs(parsed.query)
                map_name = (query.get("map_name") or [""])[0].strip()
                if not map_name:
                    raise ValueError("缺少 map_name")
                self._send_json({"ok": True, "pcd_view": self.app_state.process_manager.pcd_view(map_name)})
                return
            if path == "/api/events":
                self._send_events()
                return
            self._serve_static(path)
        except Exception as exc:  # pylint: disable=broad-except
            self._send_error(str(exc))

    def do_POST(self) -> None:  # noqa: N802
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            body = self._read_json_body()
            if path == "/api/mapping/start":
                self._send_json(self._start_mapping(body))
                return
            if path == "/api/mapping/stop":
                self._send_json(self._stop_all())
                return
            if path == "/api/localization/start":
                self._send_json(self._start_localization(body))
                return
            if path == "/api/localization/initialpose":
                self._send_json(self._publish_initialpose(body))
                return
            if path == "/api/navigation/goal":
                self._send_json(self._publish_goal(body))
                return
            if path == "/api/navigation/cancel":
                self._send_json(self._cancel_navigation())
                return
            self._send_error(f"未知接口：{path}", HTTPStatus.NOT_FOUND)
        except Exception as exc:  # pylint: disable=broad-except
            self._send_error(str(exc))

    @property
    def app_state(self) -> TestAppState:
        return self.server.app_state  # type: ignore[attr-defined]

    def _start_mapping(self, body: dict[str, Any]) -> dict[str, Any]:
        map_dir = self._resolve_map_dir(body)
        command = {"command": "start_mapping", "map_dir": str(map_dir)}
        if self._should_use_topic(body):
            self.app_state.ros_bridge.publish_runtime_command(command)
            return {"success": True, "mode": "topic", "command": command}
        result = self.app_state.process_manager.start_mapping_script(map_dir)
        result.update({"mode": "script", "map_dir": str(map_dir)})
        return result

    def _stop_all(self) -> dict[str, Any]:
        if self.app_state.ros_bridge.runtime_seen():
            self.app_state.ros_bridge.publish_runtime_command({"command": "stop"})
        result = self.app_state.process_manager.stop_all()
        result["mode"] = "topic+script" if self.app_state.ros_bridge.runtime_seen() else "script"
        return result

    def _start_localization(self, body: dict[str, Any]) -> dict[str, Any]:
        scene_dir = self._resolve_scene_dir(body)
        command = {"command": "restart_navigation", "scene_dir": str(scene_dir)}
        if self._should_use_topic(body):
            self.app_state.ros_bridge.publish_runtime_command(command)
            return {"success": True, "mode": "topic", "command": command}
        result = self.app_state.process_manager.start_navigation_script(scene_dir)
        result.update({"mode": "script", "scene_dir": str(scene_dir)})
        return result

    def _publish_initialpose(self, body: dict[str, Any]) -> dict[str, Any]:
        pose = self._pose_from_body(body)
        self.app_state.ros_bridge.publish_initialpose(**pose)
        return {"success": True, "published": "initialpose", "pose": pose}

    def _publish_goal(self, body: dict[str, Any]) -> dict[str, Any]:
        pose = self._pose_from_body(body)
        self.app_state.ros_bridge.publish_goal(**pose)
        return {"success": True, "published": "goal", "goal": pose}

    def _cancel_navigation(self) -> dict[str, Any]:
        self.app_state.ros_bridge.publish_cancel()
        return {"success": True, "published": "nav_stop"}

    def _should_use_topic(self, body: dict[str, Any]) -> bool:
        mode = str(body.get("mode", "auto")).strip().lower()
        if mode == "topic":
            return True
        if mode == "script":
            return False
        return self.app_state.ros_bridge.runtime_seen()

    def _resolve_map_dir(self, body: dict[str, Any]) -> Path:
        if body.get("map_dir"):
            path = Path(str(body["map_dir"])).expanduser().resolve()
            self.app_state.process_manager._assert_inside(self.app_state.process_manager.maps_dir, path)
            return path
        name = str(body.get("map_name", "")).strip() or f"test_scene_{int(time.time())}"
        return self.app_state.process_manager.map_dir_from_name(name)

    def _resolve_scene_dir(self, body: dict[str, Any]) -> Path:
        if body.get("scene_dir"):
            path = Path(str(body["scene_dir"])).expanduser().resolve()
            self.app_state.process_manager._assert_inside(self.app_state.process_manager.maps_dir, path)
            return path
        if body.get("map_name"):
            return self.app_state.process_manager.map_dir_from_name(str(body["map_name"]))
        raise ValueError("缺少 scene_dir 或 map_name")

    def _pose_from_body(self, body: dict[str, Any]) -> dict[str, Any]:
        return {
            "x": float(body.get("x", 0.0)),
            "y": float(body.get("y", 0.0)),
            "z": float(body.get("z", 0.0)),
            "yaw": float(body.get("yaw", 0.0)),
            "frame_id": str(body.get("frame_id", "map") or "map"),
        }

    def _read_json_body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0") or "0")
        if length <= 0:
            return {}
        raw = self.rfile.read(length).decode("utf-8")
        if not raw.strip():
            return {}
        value = json.loads(raw)
        if not isinstance(value, dict):
            raise ValueError("请求体必须是 JSON object")
        return value

    def _serve_static(self, path: str) -> None:
        if path in {"", "/"}:
            file_path = self.app_state.frontend_dir / "index.html"
        else:
            relative = path.lstrip("/")
            file_path = (self.app_state.frontend_dir / relative).resolve()
            if self.app_state.frontend_dir.resolve() not in file_path.parents:
                self._send_error("静态文件路径非法", HTTPStatus.FORBIDDEN)
                return
        if not file_path.is_file():
            self._send_error("文件不存在", HTTPStatus.NOT_FOUND)
            return
        content_type = self._content_type(file_path)
        data = file_path.read_bytes()
        self.send_response(HTTPStatus.OK)
        self._send_common_headers(content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self._send_common_headers("text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        while True:
            payload = json.dumps(self.app_state.snapshot(), ensure_ascii=False, separators=(",", ":"))
            try:
                self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                return
            time.sleep(1.0)

    def _send_json(self, payload: dict[str, Any], status: HTTPStatus = HTTPStatus.OK) -> None:
        data = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self.send_response(status)
        self._send_common_headers("application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_error(self, message: str, status: HTTPStatus = HTTPStatus.BAD_REQUEST) -> None:
        self._send_json({"ok": False, "error": message}, status)

    def _send_common_headers(self, content_type: str) -> None:
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET,POST,OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")

    def _content_type(self, path: Path) -> str:
        suffix = path.suffix.lower()
        if suffix == ".html":
            return "text/html; charset=utf-8"
        if suffix == ".js":
            return "application/javascript; charset=utf-8"
        if suffix == ".css":
            return "text/css; charset=utf-8"
        return "application/octet-stream"

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        sys.stderr.write("[NavigationTestApp] " + (format % args) + "\n")


def find_workspace_dir(value: str | None) -> Path:
    if value:
        return Path(value).expanduser().resolve()
    current = Path(__file__).resolve()
    for candidate in [current, *current.parents]:
        if (candidate / "adapters" / "legacy_scripts").is_dir():
            return candidate
    raise RuntimeError("无法定位 Navigation 工作区，请通过 --workspace-dir 指定")


def main() -> None:
    parser = argparse.ArgumentParser(description="Navigation 轻量测试前后端")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--workspace-dir", default="")
    args = parser.parse_args()

    state = TestAppState(find_workspace_dir(args.workspace_dir))
    state.start()

    server = ThreadingHTTPServer((args.host, args.port), NavigationTestHandler)
    server.app_state = state  # type: ignore[attr-defined]

    def shutdown(_signum: int, _frame: Any) -> None:
        threading.Thread(target=server.shutdown, name="navigation-test-shutdown", daemon=True).start()

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    print(f"Navigation 测试前后端已启动：http://{args.host}:{args.port}", flush=True)
    try:
        server.serve_forever()
    finally:
        state.stop()
        server.server_close()


if __name__ == "__main__":
    main()
