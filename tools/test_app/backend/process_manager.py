from __future__ import annotations

import json
import os
import re
import signal
import struct
import subprocess
import time
from pathlib import Path
from typing import Any


SCENE_NAME_PATTERN = re.compile(r"^[A-Za-z0-9_\-\u4e00-\u9fff]{1,80}$")


class ProcessManager:
    """测试后端使用的脚本进程管理器。"""

    def __init__(self, workspace_dir: Path) -> None:
        self.workspace_dir = workspace_dir.resolve()
        self.script_dir = self.workspace_dir / "adapters" / "legacy_scripts"
        self.maps_dir = self.workspace_dir / "maps"
        self.logs_dir = self.workspace_dir / "logs"
        self.runtime_dir = self.workspace_dir / "runtime"
        self._processes: dict[str, subprocess.Popen[Any]] = {}
        for path in (self.maps_dir, self.logs_dir, self.runtime_dir):
            path.mkdir(parents=True, exist_ok=True)

    def start_mapping_script(self, map_dir: Path) -> dict[str, Any]:
        map_dir = map_dir.resolve()
        self._assert_inside(self.maps_dir, map_dir)
        return self._start_script("mapping", "start_mapping.sh", [str(map_dir)])

    def start_navigation_script(self, scene_dir: Path) -> dict[str, Any]:
        scene_dir = scene_dir.resolve()
        self._assert_inside(self.maps_dir, scene_dir)
        return self._start_script("navigation", "restart_navigation_localization.sh", [str(scene_dir)])

    def stop_all(self) -> dict[str, Any]:
        script = self._script_path("stop_navigation.sh")
        result = subprocess.run(
            ["bash", str(script)],
            cwd=str(self.workspace_dir),
            env=self._env(),
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            timeout=120,
            check=False,
        )
        for process in list(self._processes.values()):
            self._stop_process(process)
        self._processes.clear()
        return {
            "success": result.returncode == 0,
            "return_code": result.returncode,
            "output": result.stdout.strip(),
        }

    def list_maps(self) -> list[dict[str, Any]]:
        items: list[dict[str, Any]] = []
        if not self.maps_dir.is_dir():
            return items
        for path in sorted(self.maps_dir.iterdir(), key=lambda item: item.name):
            if not path.is_dir():
                continue
            files = {child.name for child in path.iterdir() if child.is_file()}
            items.append(
                {
                    "name": path.name,
                    "path": str(path),
                    "has_map": "map.pcd" in files,
                    "has_terrain_map": any(name.endswith("_map.pcd") for name in files),
                    "has_ground": "ground.pcd" in files or any(name.endswith("ground.pcd") for name in files),
                    "has_footprint": "footprint_fill.pcd" in files
                    or "fill_footpoint.pcd" in files
                    or any(name.endswith("footprint_fill.pcd") for name in files)
                    or any("fill_footpoint" in name and name.endswith(".pcd") for name in files),
                    "files": sorted(files),
                }
            )
        return items

    def pcd_view(self, map_name: str, max_points_per_cloud: int = 12000) -> dict[str, Any]:
        scene_dir = self.map_dir_from_name(map_name)
        self._assert_inside(self.maps_dir, scene_dir)
        if not scene_dir.is_dir():
            raise FileNotFoundError(f"场景目录不存在：{scene_dir}")

        layers = [
            ("map", "map.pcd", self._find_scene_file(scene_dir, ["map.pcd"])),
            (
                "ground",
                "ground",
                self._find_scene_file(scene_dir, ["ground.pcd", "*_ground.pcd"]),
            ),
            (
                "base_footprint_fill",
                "base_footprint_fill",
                self._find_scene_file(
                    scene_dir,
                    [
                        "footprint_fill.pcd",
                        "fill_footpoint.pcd",
                        "*_base_footprint_fill.pcd",
                        "*footprint_fill.pcd",
                        "*fill_footpoint*.pcd",
                    ],
                ),
            ),
        ]

        result_layers = []
        for layer_id, label, path in layers:
            if path is None:
                result_layers.append(
                    {
                        "id": layer_id,
                        "label": label,
                        "path": "",
                        "count": 0,
                        "sample_count": 0,
                        "points": [],
                        "missing": True,
                    }
                )
                continue
            points, total_count = self._read_pcd_points(path, max_points_per_cloud)
            result_layers.append(
                {
                    "id": layer_id,
                    "label": label,
                    "path": str(path),
                    "count": total_count,
                    "sample_count": len(points),
                    "points": points,
                    "missing": False,
                }
            )

        return {
            "scene": scene_dir.name,
            "scene_dir": str(scene_dir),
            "layers": result_layers,
            "timestamp": time.time(),
        }

    def status(self) -> dict[str, Any]:
        processes = {}
        for name, process in list(self._processes.items()):
            processes[name] = {
                "pid": process.pid,
                "running": process.poll() is None,
                "return_code": process.poll(),
            }
            if process.poll() is not None:
                self._processes.pop(name, None)
        return {
            "workspace_dir": str(self.workspace_dir),
            "script_dir": str(self.script_dir),
            "maps_dir": str(self.maps_dir),
            "logs_dir": str(self.logs_dir),
            "runtime_dir": str(self.runtime_dir),
            "processes": processes,
            "mapping_status": self._read_json(self.runtime_dir / "mapping_status.json"),
            "navigation_status": self._read_json(self.runtime_dir / "navigation_status.json"),
            "navigation_ready": self._read_json(self.runtime_dir / "navigation_ready.json"),
            "logs": {
                "mapping": self._tail(self.logs_dir / "start_mapping_debug.log"),
                "navigation": self._tail(self.logs_dir / "restart_navigation_localization.log"),
                "runtime": self._tail(self.logs_dir / "nav_runtime_navigation.log"),
            },
        }

    def map_dir_from_name(self, name: str) -> Path:
        normalized = self._normalize_name(name)
        return (self.maps_dir / normalized).resolve()

    def _find_scene_file(self, scene_dir: Path, patterns: list[str]) -> Path | None:
        for pattern in patterns:
            matches = sorted(scene_dir.glob(pattern))
            if matches:
                return matches[-1]
        return None

    def _read_pcd_points(self, path: Path, max_points: int) -> tuple[list[list[float]], int]:
        data = path.read_bytes()
        header_end = data.find(b"DATA ")
        if header_end < 0:
            raise ValueError(f"PCD 缺少 DATA header：{path}")
        line_end = data.find(b"\n", header_end)
        if line_end < 0:
            raise ValueError(f"PCD DATA header 不完整：{path}")

        header_text = data[: line_end + 1].decode("latin1")
        header: dict[str, list[str]] = {}
        for raw_line in header_text.splitlines():
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            header[parts[0].upper()] = parts[1:]

        fields = header.get("FIELDS", [])
        sizes = [int(value) for value in header.get("SIZE", [])]
        types = header.get("TYPE", [])
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        total_count = int(header.get("POINTS", header.get("WIDTH", ["0"]))[0])
        data_type = header.get("DATA", [""])[0].lower()
        if not {"x", "y", "z"}.issubset(set(fields)):
            raise ValueError(f"PCD 缺少 x/y/z 字段：{path}")

        if data_type == "binary":
            return self._read_binary_pcd_points(
                data[line_end + 1 :], fields, sizes, types, counts, total_count, max_points
            )
        if data_type == "ascii":
            return self._read_ascii_pcd_points(
                data[line_end + 1 :].decode("utf-8", errors="replace"),
                fields,
                total_count,
                max_points,
            )
        raise ValueError(f"不支持的 PCD DATA 类型：{data_type}")

    def _read_binary_pcd_points(
        self,
        body: bytes,
        fields: list[str],
        sizes: list[int],
        types: list[str],
        counts: list[int],
        total_count: int,
        max_points: int,
    ) -> tuple[list[list[float]], int]:
        offsets: dict[str, int] = {}
        point_step = 0
        for field, size, count in zip(fields, sizes, counts):
            offsets[field] = point_step
            point_step += size * count

        stride = max(1, total_count // max_points) if max_points > 0 else 1
        points: list[list[float]] = []
        for index in range(0, total_count, stride):
            offset = index * point_step
            if offset + point_step > len(body):
                break
            points.append(
                [
                    self._unpack_pcd_value(body, offset + offsets["x"], fields, sizes, types, "x"),
                    self._unpack_pcd_value(body, offset + offsets["y"], fields, sizes, types, "y"),
                    self._unpack_pcd_value(body, offset + offsets["z"], fields, sizes, types, "z"),
                ]
            )
            if len(points) >= max_points:
                break
        return points, total_count

    def _unpack_pcd_value(
        self, body: bytes, offset: int, fields: list[str], sizes: list[int], types: list[str], field: str
    ) -> float:
        idx = fields.index(field)
        size = sizes[idx]
        value_type = types[idx].upper()
        if value_type == "F" and size == 4:
            return float(struct.unpack_from("<f", body, offset)[0])
        if value_type == "F" and size == 8:
            return float(struct.unpack_from("<d", body, offset)[0])
        if value_type == "I" and size == 4:
            return float(struct.unpack_from("<i", body, offset)[0])
        if value_type == "U" and size == 4:
            return float(struct.unpack_from("<I", body, offset)[0])
        raise ValueError(f"不支持的 PCD 字段类型：{field} {value_type}{size}")

    def _read_ascii_pcd_points(
        self, body: str, fields: list[str], total_count: int, max_points: int
    ) -> tuple[list[list[float]], int]:
        x_idx = fields.index("x")
        y_idx = fields.index("y")
        z_idx = fields.index("z")
        lines = [line for line in body.splitlines() if line.strip()]
        stride = max(1, len(lines) // max_points) if max_points > 0 else 1
        points: list[list[float]] = []
        for line in lines[::stride]:
            values = line.split()
            points.append([float(values[x_idx]), float(values[y_idx]), float(values[z_idx])])
            if len(points) >= max_points:
                break
        return points, total_count or len(lines)

    def _start_script(self, key: str, script_name: str, args: list[str]) -> dict[str, Any]:
        if key in self._processes and self._processes[key].poll() is None:
            return {"success": False, "message": f"{key} 已在运行", "pid": self._processes[key].pid}
        script = self._script_path(script_name)
        log_path = self.logs_dir / f"test_app_{key}.log"
        log_file = log_path.open("a", encoding="utf-8")
        process = subprocess.Popen(
            ["bash", str(script), *args],
            cwd=str(self.workspace_dir),
            env=self._env(),
            stdout=log_file,
            stderr=subprocess.STDOUT,
            start_new_session=True,
            text=True,
        )
        log_file.close()
        self._processes[key] = process
        time.sleep(1.0)
        return {
            "success": process.poll() is None,
            "pid": process.pid,
            "return_code": process.poll(),
            "log_file": str(log_path),
        }

    def _script_path(self, name: str) -> Path:
        path = self.script_dir / name
        if not path.is_file():
            raise FileNotFoundError(f"找不到脚本：{path}")
        return path

    def _env(self) -> dict[str, str]:
        env = os.environ.copy()
        env.setdefault("ROBOT_NAV_WS", str(self.workspace_dir))
        env.setdefault("ROBOT_NAV_MAP_ROOT", str(self.maps_dir))
        env.setdefault("ROBOT_NAV_LOG_ROOT", str(self.logs_dir))
        env.setdefault("ROBOT_NAV_RUNTIME_ROOT", str(self.runtime_dir))
        env.setdefault("ROS_LOG_DIR", str(self.logs_dir / "ros"))
        return env

    def _stop_process(self, process: subprocess.Popen[Any]) -> None:
        if process.poll() is not None:
            return
        try:
            os.killpg(process.pid, signal.SIGINT)
            process.wait(timeout=5)
        except Exception:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except Exception:
                pass

    def _normalize_name(self, value: str) -> str:
        name = value.strip()
        if not SCENE_NAME_PATTERN.match(name):
            raise ValueError("地图名称只能包含中文、英文、数字、下划线和短横线，长度不超过 80")
        if name in {".", ".."} or "/" in name or "\\" in name:
            raise ValueError("地图名称非法")
        return name

    def _assert_inside(self, root: Path, path: Path) -> None:
        root = root.resolve()
        path = path.resolve()
        if root != path and root not in path.parents:
            raise ValueError(f"路径不在允许目录内：{path}")

    def _read_json(self, path: Path) -> dict[str, Any] | None:
        if not path.is_file():
            return None
        try:
            return json.loads(path.read_text(encoding="utf-8"))
        except Exception as exc:
            return {"error": str(exc), "path": str(path)}

    def _tail(self, path: Path, limit: int = 80) -> list[str]:
        if not path.is_file():
            return []
        try:
            lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
            return lines[-limit:]
        except Exception as exc:
            return [f"读取日志失败：{exc}"]
