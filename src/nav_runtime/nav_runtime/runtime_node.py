import json
import os
import signal
import subprocess
import time
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NavigationRuntimeNode(Node):
    """通过 ROS2 topic 调度 Navigation 兼容脚本。"""

    def __init__(self) -> None:
        super().__init__("nav_runtime")

        self.declare_parameter("workspace_dir", "")
        self.declare_parameter("script_dir", "")
        self.declare_parameter("command_topic", "/nav/command_json")
        self.declare_parameter("legacy_command_topic", "/nav_command_json")
        self.declare_parameter("status_topic", "/nav/runtime_status")
        self.declare_parameter("result_topic", "/nav/command_result")
        self.declare_parameter("status_period_sec", 1.0)
        self.declare_parameter("process_wait_sec", 3.0)
        self.declare_parameter("stop_timeout_sec", 20.0)

        self.workspace_dir = self._resolve_workspace_dir()
        self.script_dir = self._resolve_script_dir()
        self.runtime_dir = self.workspace_dir / "runtime"
        self.logs_dir = self.workspace_dir / "logs"
        self.maps_dir = self.workspace_dir / "maps"
        self.current_process: Optional[subprocess.Popen] = None
        self.current_mode = "idle"
        self.current_log_path: Optional[Path] = None
        self.current_request_id: Optional[str] = None

        self.runtime_dir.mkdir(parents=True, exist_ok=True)
        self.logs_dir.mkdir(parents=True, exist_ok=True)
        self.maps_dir.mkdir(parents=True, exist_ok=True)

        command_topic = self.get_parameter("command_topic").get_parameter_value().string_value
        legacy_topic = self.get_parameter("legacy_command_topic").get_parameter_value().string_value
        status_topic = self.get_parameter("status_topic").get_parameter_value().string_value
        result_topic = self.get_parameter("result_topic").get_parameter_value().string_value
        status_period = self.get_parameter("status_period_sec").get_parameter_value().double_value

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.result_pub = self.create_publisher(String, result_topic, 10)
        self.create_subscription(String, command_topic, self._on_command, 10)
        if legacy_topic and legacy_topic != command_topic:
            self.create_subscription(String, legacy_topic, self._on_command, 10)
        self.create_timer(max(status_period, 0.2), self._publish_status)

        self.get_logger().info(f"Navigation workspace：{self.workspace_dir}")
        self.get_logger().info(f"兼容脚本目录：{self.script_dir}")
        self.get_logger().info(f"命令 topic：{command_topic}")

    def _resolve_workspace_dir(self) -> Path:
        param_value = self.get_parameter("workspace_dir").get_parameter_value().string_value
        if param_value:
            return Path(param_value).expanduser().resolve()

        env_value = os.environ.get("ROBOT_NAV_WS", "")
        if env_value:
            return Path(env_value).expanduser().resolve()

        current = Path.cwd().resolve()
        for candidate in [current, *current.parents]:
            if (candidate / "adapters" / "legacy_scripts").is_dir():
                return candidate

        return (Path.home() / "Projects" / "Navigation").resolve()

    def _resolve_script_dir(self) -> Path:
        param_value = self.get_parameter("script_dir").get_parameter_value().string_value
        if param_value:
            return Path(param_value).expanduser().resolve()

        env_value = os.environ.get("NAV_LEGACY_SCRIPT_DIR", "")
        if env_value:
            return Path(env_value).expanduser().resolve()

        return self.workspace_dir / "adapters" / "legacy_scripts"

    def _on_command(self, msg: String) -> None:
        try:
            command = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            self._publish_result(False, "invalid_json", f"命令不是合法 JSON：{exc}")
            return

        action = str(command.get("command", command.get("action", ""))).strip()
        self.current_request_id = str(command.get("request_id", "")).strip() or None
        if not action:
            self._publish_result(False, "missing_command", "缺少 command 字段")
            self.current_request_id = None
            return

        handlers = {
            "start_mapping": self._start_mapping,
            "mapping_start": self._start_mapping,
            "restart_navigation": self._restart_navigation,
            "start_navigation": self._restart_navigation,
            "navigation_restart": self._restart_navigation,
            "stop": self._stop_navigation,
            "stop_navigation": self._stop_navigation,
            "status": self._handle_status,
        }
        handler = handlers.get(action)
        if handler is None:
            self._publish_result(False, "unsupported_command", f"不支持的命令：{action}")
            self.current_request_id = None
            return

        try:
            handler(command)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().exception(f"执行命令失败：{action}")
            self._publish_result(False, "command_failed", str(exc), {"command": action})
        finally:
            self.current_request_id = None

    def _start_mapping(self, command: Dict[str, Any]) -> None:
        map_dir = str(command.get("map_dir", "")).strip()
        if not map_dir:
            map_name = str(command.get("map_name", f"map_{int(time.time())}")).strip()
            map_dir = str(self.maps_dir / map_name)
        self._launch_script("mapping", "start_mapping.sh", [map_dir])

    def _restart_navigation(self, command: Dict[str, Any]) -> None:
        scene_dir = str(command.get("scene_dir", command.get("map_dir", ""))).strip()
        if not scene_dir:
            self._publish_result(False, "missing_scene_dir", "启动导航需要 scene_dir 或 map_dir")
            return
        self._launch_script("navigation", "restart_navigation_localization.sh", [scene_dir])

    def _stop_navigation(self, command: Dict[str, Any]) -> None:
        del command
        script_path = self._script_path("stop_navigation.sh")
        timeout = self.get_parameter("stop_timeout_sec").get_parameter_value().double_value
        env = self._subprocess_env()

        result = subprocess.run(
            ["bash", str(script_path)],
            cwd=str(self.workspace_dir),
            env=env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=max(timeout, 1.0),
            check=False,
        )
        self._terminate_current_process()
        self.current_mode = "idle"
        self._publish_result(
            result.returncode == 0,
            "stop_navigation",
            result.stdout.strip() or "停止命令已执行",
            {"return_code": result.returncode},
        )

    def _handle_status(self, command: Dict[str, Any]) -> None:
        del command
        self._publish_status()
        self._publish_result(True, "status", "状态已发布")

    def _launch_script(self, mode: str, script_name: str, args: list[str]) -> None:
        script_path = self._script_path(script_name)
        self._terminate_current_process()

        log_path = self.logs_dir / f"nav_runtime_{mode}.log"
        env = self._subprocess_env()
        log_file = log_path.open("a", encoding="utf-8")
        process = subprocess.Popen(
            ["bash", str(script_path), *args],
            cwd=str(self.workspace_dir),
            env=env,
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        log_file.close()

        self.current_process = process
        self.current_mode = mode
        self.current_log_path = log_path

        wait_sec = self.get_parameter("process_wait_sec").get_parameter_value().double_value
        time.sleep(max(wait_sec, 0.0))
        if process.poll() is not None:
            self._publish_result(
                False,
                f"{mode}_exited",
                f"{script_name} 启动后退出，请查看日志：{log_path}",
                {"return_code": process.returncode, "log_file": str(log_path)},
            )
            return

        self._publish_result(
            True,
            f"{mode}_started",
            f"{mode} 已启动",
            {"pid": process.pid, "log_file": str(log_path), "args": args},
        )

    def _script_path(self, script_name: str) -> Path:
        script_path = self.script_dir / script_name
        if not script_path.is_file():
            raise FileNotFoundError(f"找不到兼容脚本：{script_path}")
        return script_path

    def _subprocess_env(self) -> Dict[str, str]:
        env = os.environ.copy()
        env.setdefault("ROBOT_NAV_WS", str(self.workspace_dir))
        env.setdefault("ROBOT_NAV_LOG_ROOT", str(self.logs_dir))
        env.setdefault("ROBOT_NAV_RUNTIME_ROOT", str(self.runtime_dir))
        env.setdefault("ROBOT_NAV_MAP_ROOT", str(self.maps_dir))
        env.setdefault("ROS_LOG_DIR", str(self.logs_dir / "ros"))
        return env

    def _terminate_current_process(self) -> None:
        process = self.current_process
        if process is None or process.poll() is not None:
            return

        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        except Exception:  # pylint: disable=broad-except
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

    def _publish_status(self) -> None:
        payload: Dict[str, Any] = {
            "running": self._is_current_process_running(),
            "mode": self.current_mode,
            "workspace_dir": str(self.workspace_dir),
            "script_dir": str(self.script_dir),
            "timestamp": time.time(),
        }
        if self.current_process is not None:
            payload["pid"] = self.current_process.pid
            payload["return_code"] = self.current_process.poll()
        if self.current_log_path is not None:
            payload["log_file"] = str(self.current_log_path)

        payload["mapping_status"] = self._read_json_file(self.runtime_dir / "mapping_status.json")
        payload["navigation_status"] = self._read_json_file(self.runtime_dir / "navigation_status.json")
        payload["navigation_ready"] = self._read_json_file(self.runtime_dir / "navigation_ready.json")

        self._publish_json(self.status_pub, payload)

    def _is_current_process_running(self) -> bool:
        return self.current_process is not None and self.current_process.poll() is None

    def _read_json_file(self, path: Path) -> Optional[Dict[str, Any]]:
        if not path.is_file():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return {"error": str(exc), "path": str(path)}

    def _publish_result(
        self,
        success: bool,
        code: str,
        message: str,
        extra: Optional[Dict[str, Any]] = None,
    ) -> None:
        payload: Dict[str, Any] = {
            "success": success,
            "code": code,
            "message": message,
            "timestamp": time.time(),
        }
        if self.current_request_id:
            payload["request_id"] = self.current_request_id
        if extra:
            payload.update(extra)
        self._publish_json(self.result_pub, payload)
        log_message = f"{code}：{message}"
        if success:
            self.get_logger().info(log_message)
        else:
            self.get_logger().error(log_message)

    def _publish_json(self, publisher, payload: Dict[str, Any]) -> None:
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        publisher.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = NavigationRuntimeNode()
    try:
        rclpy.spin(node)
    finally:
        node._terminate_current_process()
        node.destroy_node()
        rclpy.shutdown()
