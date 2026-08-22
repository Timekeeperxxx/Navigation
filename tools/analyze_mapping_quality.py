#!/usr/bin/env python3
"""Summarize frontend observability and loop-closure corrections.

The analyzer is intentionally read-only.  Point it at one offline mapping result
directory containing ``frontend_keyframe_trajectory.txt`` and, when available,
``mapping.log`` and ``loop_pose_graph.txt``::

    python3 tools/analyze_mapping_quality.py /path/to/result

Only the Python standard library and NumPy are required.
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from collections import Counter, defaultdict
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Sequence, Tuple

import numpy as np


FRONTEND_FORMATS: Mapping[str, Tuple[str, ...]] = {
    "NAV_LIO_FRONTEND_KEYFRAME_TRAJECTORY_V1": (
        "keyframe_index",
        "scan_index",
        "timestamp",
        "cumulative_path_m",
        "raw_tx",
        "raw_ty",
        "raw_tz",
        "raw_qx",
        "raw_qy",
        "raw_qz",
        "raw_qw",
        "rot_eig_min",
        "rot_eig_mid",
        "rot_eig_max",
        "rot_min_max_ratio",
        "trans_eig_min",
        "trans_eig_mid",
        "trans_eig_max",
        "trans_min_max_ratio",
        "yaw_information_ratio",
        "effective_matches",
        "points",
    ),
    "NAV_LIO_FRONTEND_KEYFRAME_TRAJECTORY_V2": (
        "keyframe_index",
        "scan_index",
        "timestamp",
        "cumulative_path_m",
        "raw_tx",
        "raw_ty",
        "raw_tz",
        "raw_qx",
        "raw_qy",
        "raw_qz",
        "raw_qw",
        "rot_eig_min",
        "rot_eig_mid",
        "rot_eig_max",
        "rot_min_max_ratio",
        "trans_eig_min",
        "trans_eig_mid",
        "trans_eig_max",
        "trans_min_max_ratio",
        "raw_yaw_information_ratio",
        "conditional_yaw_information_ratio",
        "translation_information_ratio",
        "effective_matches",
        "points",
    ),
}

POSE_GRAPH_MAGIC = "NAV_LIO_POSE_GRAPH_V1"
PERCENTILES = (0, 1, 5, 10, 25, 50, 75, 90, 95, 99, 100)
ANSI_ESCAPE = re.compile(r"\x1b(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])")
LOG_KEY = re.compile(r"(?<![A-Za-z0-9_])([A-Za-z][A-Za-z0-9_]*):\s*")


class AnalysisError(RuntimeError):
    """Raised when an input diagnostic file is malformed."""


@dataclass(frozen=True)
class FrontendTrajectory:
    path: Path
    magic: str
    fields: Tuple[str, ...]
    values: np.ndarray

    def column(self, name: str) -> np.ndarray:
        try:
            index = self.fields.index(name)
        except ValueError as error:
            raise AnalysisError(f"{self.path}: missing field {name!r}") from error
        return self.values[:, index]

    @property
    def size(self) -> int:
        return int(self.values.shape[0])


@dataclass(frozen=True)
class PoseCorrections:
    path: Path
    timestamps: np.ndarray
    matrices: np.ndarray

    @property
    def size(self) -> int:
        return int(self.timestamps.size)


@dataclass(frozen=True)
class MetricInterval:
    start: int
    end: int
    minimum: float
    mean: float
    maximum: float

    @property
    def count(self) -> int:
        return self.end - self.start + 1


@dataclass
class ProactiveGroundZStats:
    """Loosely parsed proactive pure-Z evidence and graph events.

    Values intentionally remain strings: mapping logs are a diagnostic wire
    format, and new optional fields must not make an older analyzer reject a
    run.  Numeric conversion is only done for fields used in summaries.
    """

    candidate_records: List[Dict[str, str]] = field(default_factory=list)
    group_records: List[Dict[str, str]] = field(default_factory=list)
    submission_records: List[Dict[str, str]] = field(default_factory=list)
    normalization_records: List[Dict[str, str]] = field(default_factory=list)
    summary_records: List[Dict[str, str]] = field(default_factory=list)
    graph_summary_records: List[Dict[str, str]] = field(default_factory=list)
    rejections: Counter = field(default_factory=Counter)
    graph_adjustments: Counter = field(default_factory=Counter)


@dataclass
class MappingLogStats:
    anchor_edges_by_group: Dict[int, List[bool]] = field(
        default_factory=lambda: defaultdict(list))
    # One evidence group may now contain several index-overlapping
    # ``original_pair`` records.  Keep every per-pair normalization line;
    # storing only one integer per group silently overwrote earlier pairs.
    normalized_pair_anchors_by_group: Dict[int, List[int]] = field(
        default_factory=lambda: defaultdict(list))
    normalized_original_pairs_by_group: Dict[int, List[Tuple[int, int]]] = field(
        default_factory=lambda: defaultdict(list))
    cross_candidate_normalizations: int = 0
    soft_prevalidation_attempts: int = 0
    soft_prevalidation_accepted: int = 0
    soft_fallback_submitted: int = 0
    hard_rejected_without_anchor: int = 0
    graph_adjustments: Counter = field(default_factory=Counter)
    final_summary: Dict[str, str] = field(default_factory=dict)
    final_graph_rejected: bool = False
    proactive_ground_z: ProactiveGroundZStats = field(
        default_factory=ProactiveGroundZStats)


def _read_nonempty_lines(path: Path) -> List[Tuple[int, str]]:
    try:
        text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        raise AnalysisError(f"cannot read {path}: {error}") from error
    return [
        (line_number, line.strip())
        for line_number, line in enumerate(text.splitlines(), start=1)
        if line.strip()
    ]


def _parse_declared_count(path: Path, item: Tuple[int, str]) -> int:
    line_number, text = item
    try:
        count = int(text)
    except ValueError as error:
        raise AnalysisError(
            f"{path}:{line_number}: expected integer record count, got {text!r}"
        ) from error
    if count < 0:
        raise AnalysisError(f"{path}:{line_number}: record count must be non-negative")
    return count


def _parse_numeric_rows(
    path: Path,
    lines: Iterable[Tuple[int, str]],
    width: int,
) -> np.ndarray:
    rows: List[List[float]] = []
    for line_number, text in lines:
        if text.startswith("#"):
            continue
        tokens = text.split()
        if len(tokens) != width:
            raise AnalysisError(
                f"{path}:{line_number}: expected {width} columns, got {len(tokens)}"
            )
        try:
            row = [float(token) for token in tokens]
        except ValueError as error:
            raise AnalysisError(
                f"{path}:{line_number}: record contains a non-numeric value"
            ) from error
        rows.append(row)
    if not rows:
        return np.empty((0, width), dtype=np.float64)
    values = np.asarray(rows, dtype=np.float64)
    if not np.all(np.isfinite(values)):
        row, column = np.argwhere(~np.isfinite(values))[0]
        raise AnalysisError(
            f"{path}: non-finite value in data row {int(row) + 1}, "
            f"column {int(column) + 1}"
        )
    return values


def load_frontend_trajectory(path: Path) -> FrontendTrajectory:
    lines = _read_nonempty_lines(path)
    if len(lines) < 2:
        raise AnalysisError(f"{path}: missing format header or record count")
    magic = lines[0][1]
    fields = FRONTEND_FORMATS.get(magic)
    if fields is None:
        supported = ", ".join(FRONTEND_FORMATS)
        raise AnalysisError(
            f"{path}:{lines[0][0]}: unsupported format {magic!r}; "
            f"supported: {supported}"
        )
    declared_count = _parse_declared_count(path, lines[1])
    values = _parse_numeric_rows(path, lines[2:], len(fields))
    if values.shape[0] != declared_count:
        raise AnalysisError(
            f"{path}: header declares {declared_count} rows, "
            f"but {values.shape[0]} were read"
        )
    if declared_count == 0:
        raise AnalysisError(f"{path}: trajectory contains no keyframes")

    indices = values[:, fields.index("keyframe_index")]
    expected_indices = np.arange(declared_count, dtype=np.float64)
    if not np.array_equal(indices, expected_indices):
        raise AnalysisError(
            f"{path}: keyframe_index must be the contiguous sequence "
            f"0..{declared_count - 1}"
        )
    scan_indices = values[:, fields.index("scan_index")]
    if not np.all(scan_indices == np.floor(scan_indices)):
        raise AnalysisError(f"{path}: scan_index must contain integers")
    if declared_count > 1 and np.any(np.diff(scan_indices) <= 0.0):
        raise AnalysisError(f"{path}: scan_index must be strictly increasing")
    timestamps = values[:, fields.index("timestamp")]
    if declared_count > 1 and np.any(np.diff(timestamps) <= 0.0):
        raise AnalysisError(f"{path}: timestamps must be strictly increasing")
    path_lengths = values[:, fields.index("cumulative_path_m")]
    if path_lengths[0] < -1.0e-9 or np.any(np.diff(path_lengths) < -1.0e-7):
        raise AnalysisError(f"{path}: cumulative_path_m must be non-decreasing")
    quaternions = values[:, [fields.index(name) for name in (
        "raw_qx", "raw_qy", "raw_qz", "raw_qw"
    )]]
    if np.any(np.linalg.norm(quaternions, axis=1) <= 1.0e-12):
        raise AnalysisError(f"{path}: raw pose contains a zero quaternion")

    return FrontendTrajectory(path=path, magic=magic, fields=fields, values=values)


def load_pose_corrections(path: Path) -> PoseCorrections:
    lines = _read_nonempty_lines(path)
    if len(lines) < 2:
        raise AnalysisError(f"{path}: missing format header or record count")
    if lines[0][1] != POSE_GRAPH_MAGIC:
        raise AnalysisError(
            f"{path}:{lines[0][0]}: expected {POSE_GRAPH_MAGIC!r}, "
            f"got {lines[0][1]!r}"
        )
    declared_count = _parse_declared_count(path, lines[1])
    values = _parse_numeric_rows(path, lines[2:], 17)
    if values.shape[0] != declared_count:
        raise AnalysisError(
            f"{path}: header declares {declared_count} rows, "
            f"but {values.shape[0]} were read"
        )
    if declared_count == 0:
        return PoseCorrections(
            path=path,
            timestamps=np.empty(0, dtype=np.float64),
            matrices=np.empty((0, 4, 4), dtype=np.float64),
        )
    timestamps = values[:, 0]
    if declared_count > 1 and np.any(np.diff(timestamps) <= 0.0):
        raise AnalysisError(f"{path}: timestamps must be strictly increasing")
    matrices = values[:, 1:].reshape((-1, 4, 4))
    expected_bottom = np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (declared_count, 1))
    if not np.allclose(matrices[:, 3, :], expected_bottom, atol=2.0e-5):
        raise AnalysisError(f"{path}: a correction matrix has an invalid bottom row")
    return PoseCorrections(path=path, timestamps=timestamps, matrices=matrices)


def _extract_token(line: str, key: str) -> Optional[str]:
    match = re.search(
        rf"(?<![A-Za-z0-9_]){re.escape(key)}:\s*([^\s]+)", line
    )
    return match.group(1).rstrip(",;") if match else None


def _extract_int(line: str, key: str) -> Optional[int]:
    token = _extract_token(line, key)
    if token is None:
        return None
    try:
        return int(token)
    except ValueError:
        return None


def _extract_bool(line: str, key: str) -> Optional[bool]:
    token = _extract_token(line, key)
    if token is None:
        return None
    normalized = token.lower()
    if normalized in ("1", "true", "yes"):
        return True
    if normalized in ("0", "false", "no"):
        return False
    return None


def _extract_summary_fields(line: str, keys: Sequence[str]) -> Dict[str, str]:
    result: Dict[str, str] = {}
    for key in keys:
        token = _extract_token(line, key)
        if token is not None:
            result[key] = token
    return result


def _extract_index_pair(line: str, key: str) -> Optional[Tuple[int, int]]:
    match = re.search(
        rf"(?<![A-Za-z0-9_]){re.escape(key)}:\s*"
        r"(-?\d+)\s*(?:->|=>|/|,)\s*(-?\d+)",
        line,
    )
    if match is None:
        return None
    return int(match.group(1)), int(match.group(2))


def _extract_loose_key_values(line: str) -> Dict[str, str]:
    """Extract all ``key: value`` fields without fixing a log schema.

    A value ends at the next ASCII identifier followed by a colon, so values
    such as ``492 -> 3485`` and three-component vectors are retained.  This is
    deliberately more permissive than the strict parsers for trajectory files.
    """

    matches = list(LOG_KEY.finditer(line))
    result: Dict[str, str] = {}
    for index, match in enumerate(matches):
        value_end = matches[index + 1].start() if index + 1 < len(matches) else len(line)
        value = line[match.end() : value_end].strip().rstrip(",;")
        # Color escapes have already been removed, but log punctuation may
        # remain at the end of a final field.
        value = value.rstrip("。")
        if value:
            result[match.group(1)] = value
    return result


def _record_bool(record: Mapping[str, str], *keys: str) -> Optional[bool]:
    for key in keys:
        value = record.get(key)
        if value is None:
            continue
        normalized = value.split()[0].strip(",;").lower()
        if normalized in ("1", "true", "yes"):
            return True
        if normalized in ("0", "false", "no"):
            return False
    return None


def _record_float(record: Mapping[str, str], *keys: str) -> Optional[float]:
    for key in keys:
        value = record.get(key)
        if value is None:
            continue
        try:
            number = float(value.split()[0].strip("(),;"))
        except ValueError:
            continue
        if math.isfinite(number):
            return number
    return None


def _proactive_event_acceptance(
    kind: str,
    line: str,
    record: Mapping[str, str],
) -> Optional[bool]:
    accepted = _record_bool(
        record,
        "accepted_as_proposal",
        "accepted",
        "submitted",
        "committed",
    )
    if accepted is not None:
        return accepted
    if "拒绝" in line or "未提交" in line:
        return False
    # These two prefixes describe successful events by construction.  A group
    # evaluation is left unknown unless it carries an explicit accepted field.
    if kind == "candidate" and "局部提案" in line:
        return True
    if kind == "submission" and "提交" in line:
        return True
    return None


def _record_proactive_rejection(
    stats: ProactiveGroundZStats,
    kind: str,
    line: str,
    record: Mapping[str, str],
) -> None:
    if _proactive_event_acceptance(kind, line, record) is not False:
        return
    reason = record.get("rejection_reason") or record.get("reason")
    if reason is None and kind == "group":
        inferred: List[str] = []
        if _record_bool(record, "strict_full_edge_present") is True:
            inferred.append("strict_full_edge_present")
        consensus_anchors = _record_float(
            record, "consensus_anchors", "selected_anchors")
        if consensus_anchors is not None and consensus_anchors < 2.0:
            inferred.append("insufficient_anchors")
        anchor_span = _record_float(record, "anchor_span", "physical_span")
        minimum_span = _record_float(
            record, "minimum_anchor_span", "minimum_physical_span")
        if (
            anchor_span is not None
            and minimum_span is not None
            and anchor_span < minimum_span
        ):
            inferred.append("insufficient_span")
        z_adjustment = _record_float(record, "z_adjustment", "tz_median")
        maximum_z = _record_float(record, "maximum_group_z", "span_tz_limit")
        if (
            z_adjustment is not None
            and maximum_z is not None
            and abs(z_adjustment) > maximum_z
        ):
            inferred.append("excessive_z")
        if _record_bool(record, "vertical_path_ratio_valid") is False:
            inferred.append("vertical_path_ratio")
        reason = "+".join(inferred) if inferred else None
    reason = reason or "unspecified"
    stats.rejections[f"{kind}:{reason.split()[0].strip(',;')}"] += 1


def parse_mapping_log(path: Path) -> MappingLogStats:
    stats = MappingLogStats()
    try:
        raw_text = path.read_text(encoding="utf-8", errors="replace")
    except OSError as error:
        raise AnalysisError(f"cannot read {path}: {error}") from error

    summary_keys = (
        "nodes",
        "internal_loops",
        "internal_groups",
        "pruned_internal_loops",
        "pruned_internal_groups",
        "endpoint_loop_active",
        "pruned_endpoint_loops",
        "downgraded_internal_z_loops",
        "downgraded_endpoint_z_loops",
        "downweighted_soft_fallback_loops",
        "proactive_ground_z_loops",
        "proactive_ground_z_groups",
        "pruned_proactive_ground_z_loops",
        "pruned_proactive_ground_z_groups",
        "proactive_ground_z_only_loops",
        "proactive_ground_z_only_groups",
        "pruned_proactive_ground_z_only_loops",
        "pruned_proactive_ground_z_only_groups",
        "downweighted_proactive_ground_z_only_loops",
        "raw_route_length",
        "max_correction_translation",
        "max_correction_rotation_deg",
        "max_adjacent_translation",
        "max_adjacent_translation_index",
        "max_adjacent_rotation_deg",
        "adjacent_correction_supported",
    )
    for raw_line in raw_text.splitlines():
        line = ANSI_ESCAPE.sub("", raw_line)
        if "中途回环提交局部锚边" in line:
            group_id = _extract_int(line, "group_id")
            soft_fallback = _extract_bool(line, "soft_fallback")
            if group_id is not None:
                stats.anchor_edges_by_group[group_id].append(bool(soft_fallback))
        if "中途回环组内信息归一" in line:
            group_id = _extract_int(line, "group_id")
            anchors = _extract_int(line, "anchors")
            if group_id is not None and anchors is not None:
                stats.normalized_pair_anchors_by_group[group_id].append(anchors)
            original_pair = _extract_index_pair(line, "original_pair")
            if group_id is not None and original_pair is not None:
                stats.normalized_original_pairs_by_group[group_id].append(original_pair)
        if "中途回环跨候选证据组归一" in line:
            stats.cross_candidate_normalizations += 1
        if "中途回环 soft fallback 预验" in line:
            stats.soft_prevalidation_attempts += 1
            if _extract_bool(line, "eligible_if_no_local_anchor") is True:
                stats.soft_prevalidation_accepted += 1
        if "中途回环无 strict 局部锚，提交低权 soft fallback" in line:
            stats.soft_fallback_submitted += 1
        if "中途回环无局部锚且 soft fallback 不满足门限，硬拒绝" in line:
            stats.hard_rejected_without_anchor += 1
        if "调整冲突回环并重新优化" in line:
            action = _extract_token(line, "action") or "unknown"
            stats.graph_adjustments[action] += 1
            proactive_ground_z_only = _extract_bool(
                line, "proactive_ground_z_only")
            if (
                proactive_ground_z_only is True
                or "proactive_ground_z_only" in action
                or "ground_z_only" in action
                or "pure_z" in action
                or "主动纯Z" in line
                or "独立纯Z" in line
            ):
                stats.proactive_ground_z.graph_adjustments[action] += 1
        if "internal-only 位姿图" in line:
            stats.proactive_ground_z.graph_summary_records.append(
                _extract_loose_key_values(
                    line[line.index("internal-only 位姿图") :]))
        pure_z_label = (
            "主动纯Z" if "主动纯Z" in line
            else "独立纯Z" if "独立纯Z" in line
            else None
        )
        if pure_z_label is not None:
            # Drop logger timestamps/file prefixes before loose parsing; a
            # ``super_lio.cpp:1234`` prefix would otherwise look like a field.
            record = _extract_loose_key_values(line[line.index(pure_z_label) :])
            if "局部提案" in line or "候选取证" in line:
                acceptance = _proactive_event_acceptance("candidate", line, record)
                if acceptance is not None:
                    record["_accepted"] = str(acceptance).lower()
                stats.proactive_ground_z.candidate_records.append(record)
                _record_proactive_rejection(
                    stats.proactive_ground_z, "candidate", line, record)
            elif "证据组" in line:
                acceptance = _proactive_event_acceptance("group", line, record)
                if acceptance is not None:
                    record["_accepted"] = str(acceptance).lower()
                stats.proactive_ground_z.group_records.append(record)
                _record_proactive_rejection(
                    stats.proactive_ground_z, "group", line, record)
            elif "提交" in line:
                acceptance = _proactive_event_acceptance("submission", line, record)
                if acceptance is not None:
                    record["_accepted"] = str(acceptance).lower()
                stats.proactive_ground_z.submission_records.append(record)
                _record_proactive_rejection(
                    stats.proactive_ground_z, "submission", line, record)
            elif "组内信息归一" in line:
                stats.proactive_ground_z.normalization_records.append(record)
            elif "汇总" in line:
                stats.proactive_ground_z.summary_records.append(record)
        if "多回环位姿图优化完成" in line:
            stats.final_summary = _extract_summary_fields(line, summary_keys)
            stats.final_graph_rejected = False
        elif "位姿图修正不可信，拒绝生成闭环地图" in line:
            stats.final_summary = _extract_summary_fields(line, summary_keys)
            stats.final_graph_rejected = True
    return stats


def _rotation_matrix_from_quaternion(quaternion_xyzw: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion_xyzw, dtype=np.float64)
    quaternion /= np.linalg.norm(quaternion)
    x, y, z, w = quaternion
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float64,
    )


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = float((np.trace(rotation) - 1.0) * 0.5)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def _relative_yaw_deg(first: np.ndarray, last: np.ndarray) -> float:
    relative = first.T @ last
    return math.degrees(math.atan2(float(relative[1, 0]), float(relative[0, 0])))


def _percentile_text(values: np.ndarray) -> str:
    percentile_values = np.percentile(values, PERCENTILES)
    return " ".join(
        f"p{percentile:g}={value:.6g}"
        for percentile, value in zip(PERCENTILES, percentile_values)
    )


def _make_interval(values: np.ndarray, start: int, end: int) -> MetricInterval:
    selected = values[start : end + 1]
    return MetricInterval(
        start=start,
        end=end,
        minimum=float(np.min(selected)),
        mean=float(np.mean(selected)),
        maximum=float(np.max(selected)),
    )


def _longest_below(values: np.ndarray, threshold: float) -> Optional[MetricInterval]:
    best: Optional[MetricInterval] = None
    run_start: Optional[int] = None
    for index, below in enumerate(values < threshold):
        if below and run_start is None:
            run_start = index
        at_end = index == values.size - 1
        if run_start is not None and ((not below) or at_end):
            run_end = index if below and at_end else index - 1
            candidate = _make_interval(values, run_start, run_end)
            if (
                best is None
                or candidate.count > best.count
                or (candidate.count == best.count and candidate.mean < best.mean)
            ):
                best = candidate
            run_start = None
    return best


def _worst_window(values: np.ndarray, requested_window: int) -> MetricInterval:
    window = min(requested_window, int(values.size))
    cumulative = np.concatenate(([0.0], np.cumsum(values, dtype=np.float64)))
    means = (cumulative[window:] - cumulative[:-window]) / float(window)
    start = int(np.argmin(means))
    return _make_interval(values, start, start + window - 1)


def _interval_text(interval: MetricInterval, trajectory: FrontendTrajectory) -> str:
    scans = trajectory.column("scan_index")
    timestamps = trajectory.column("timestamp")
    paths = trajectory.column("cumulative_path_m")
    duration = float(timestamps[interval.end] - timestamps[interval.start])
    path_span = float(paths[interval.end] - paths[interval.start])
    return (
        f"kf={interval.start}..{interval.end} "
        f"scan={int(scans[interval.start])}..{int(scans[interval.end])} "
        f"time={timestamps[interval.start]:.6f}..{timestamps[interval.end]:.6f} "
        f"(dt={duration:.3f}s) path={paths[interval.start]:.3f}.."
        f"{paths[interval.end]:.3f}m (d={path_span:.3f}m) count={interval.count} "
        f"min/mean/max={interval.minimum:.6g}/{interval.mean:.6g}/"
        f"{interval.maximum:.6g}"
    )


def _event_location(index: int, trajectory: FrontendTrajectory) -> str:
    if index < 0 or index >= trajectory.size:
        return f"kf={index}"
    scan = int(trajectory.column("scan_index")[index])
    timestamp = trajectory.column("timestamp")[index]
    path = trajectory.column("cumulative_path_m")[index]
    xyz = np.array(
        [
            trajectory.column("raw_tx")[index],
            trajectory.column("raw_ty")[index],
            trajectory.column("raw_tz")[index],
        ]
    )
    return (
        f"kf={index} scan={scan} time={timestamp:.6f} path={path:.3f}m "
        f"raw_xyz=({xyz[0]:.3f},{xyz[1]:.3f},{xyz[2]:.3f})"
    )


def _print_frontend_summary(trajectory: FrontendTrajectory) -> None:
    timestamps = trajectory.column("timestamp")
    path = trajectory.column("cumulative_path_m")
    positions = np.column_stack(
        (
            trajectory.column("raw_tx"),
            trajectory.column("raw_ty"),
            trajectory.column("raw_tz"),
        )
    )
    quaternion_indices = [
        trajectory.fields.index(name)
        for name in ("raw_qx", "raw_qy", "raw_qz", "raw_qw")
    ]
    rotations = [
        _rotation_matrix_from_quaternion(row[quaternion_indices])
        for row in trajectory.values[[0, -1]]
    ]
    delta = positions[-1] - positions[0]
    gap_xy = float(np.linalg.norm(delta[:2]))
    gap_xyz = float(np.linalg.norm(delta))
    route_length = float(path[-1])
    closure_ratio = gap_xyz / route_length if route_length > 1.0e-12 else math.nan
    relative_rotation = rotations[0].T @ rotations[1]

    print("\n[前端 raw 轨迹]")
    print(
        f"格式={trajectory.magic} keyframes={trajectory.size} "
        f"scan={int(trajectory.column('scan_index')[0])}.."
        f"{int(trajectory.column('scan_index')[-1])} "
        f"duration={timestamps[-1] - timestamps[0]:.3f}s"
    )
    print(
        f"起点=({positions[0, 0]:.3f},{positions[0, 1]:.3f},{positions[0, 2]:.3f})m "
        f"终点=({positions[-1, 0]:.3f},{positions[-1, 1]:.3f},{positions[-1, 2]:.3f})m"
    )
    print(
        f"起末差 dxyz=({delta[0]:.3f},{delta[1]:.3f},{delta[2]:.3f})m "
        f"xy={gap_xy:.3f}m xyz={gap_xyz:.3f}m "
        f"yaw={_relative_yaw_deg(rotations[0], rotations[1]):.3f}deg "
        f"rotation={_rotation_angle_deg(relative_rotation):.3f}deg"
    )
    print(
        f"累计路程={route_length:.3f}m "
        f"起末距离/路程={closure_ratio:.6g}"
    )


def _print_observability(
    trajectory: FrontendTrajectory,
    rotation_threshold: float,
    translation_threshold: float,
    yaw_threshold: float,
    window: int,
) -> None:
    conditional_yaw_field = (
        "conditional_yaw_information_ratio"
        if "conditional_yaw_information_ratio" in trajectory.fields
        else "yaw_information_ratio"
    )
    translation_field = (
        "translation_information_ratio"
        if "translation_information_ratio" in trajectory.fields
        else "trans_min_max_ratio"
    )
    metrics = (
        ("rotation min/max", "rot_min_max_ratio", rotation_threshold),
        ("translation min/max", translation_field, translation_threshold),
        ("conditional yaw", conditional_yaw_field, yaw_threshold),
    )
    print("\n[前端可观测性]")
    if trajectory.magic.endswith("_V1"):
        print("说明: V1 没有 conditional yaw；此处用 yaw_information_ratio 兼容代替。")
    for label, field_name, threshold in metrics:
        values = trajectory.column(field_name)
        print(f"{label} ({field_name}) threshold={threshold:.6g}")
        print(f"  分位数: {_percentile_text(values)}")
        longest = _longest_below(values, threshold)
        if longest is None:
            print("  最长低于门限连续区间: 无")
        else:
            print(f"  最长低于门限连续区间: {_interval_text(longest, trajectory)}")
        worst = _worst_window(values, window)
        print(
            f"  最差连续窗口(请求={window},实际={worst.count}): "
            f"{_interval_text(worst, trajectory)}"
        )


def _proactive_acceptance_counts(
    records: Sequence[Mapping[str, str]],
) -> Tuple[int, int, int]:
    accepted = 0
    rejected = 0
    unknown = 0
    for record in records:
        value = _record_bool(record, "_accepted")
        if value is True:
            accepted += 1
        elif value is False:
            rejected += 1
        else:
            unknown += 1
    return accepted, rejected, unknown


def _proactive_numeric_values(
    records: Sequence[Mapping[str, str]],
    *keys: str,
) -> np.ndarray:
    values = [
        value
        for record in records
        if (value := _record_float(record, *keys)) is not None
    ]
    return np.asarray(values, dtype=np.float64)


def _numeric_min_median_max(values: np.ndarray) -> str:
    return (
        f"{float(np.min(values)):.6g}/"
        f"{float(np.median(values)):.6g}/"
        f"{float(np.max(values)):.6g}"
    )


def _proactive_group_ids(
    records: Sequence[Mapping[str, str]],
) -> set[str]:
    result: set[str] = set()
    for record in records:
        for key in (
            "ground_group_id",
            "source_evidence_group_id",
            "evidence_group_id",
            "group_id",
        ):
            value = record.get(key)
            if value is not None:
                result.add(value.split()[0].strip(",;"))
                break
    return result


def _print_proactive_ground_z_summary(stats: MappingLogStats) -> None:
    proactive = stats.proactive_ground_z
    candidate_counts = _proactive_acceptance_counts(proactive.candidate_records)
    group_counts = _proactive_acceptance_counts(proactive.group_records)
    submission_counts = _proactive_acceptance_counts(proactive.submission_records)
    original_pairs = {
        record["original_pair"]
        for record in proactive.candidate_records
        if "original_pair" in record
    }
    submitted_groups = _proactive_group_ids(proactive.submission_records)
    print(
        "主动纯Z: "
        f"候选={len(proactive.candidate_records)} "
        f"通过/拒绝/未知={candidate_counts[0]}/{candidate_counts[1]}/{candidate_counts[2]} "
        f"source_groups={len(_proactive_group_ids(proactive.candidate_records))} "
        f"original_pairs={len(original_pairs)}; "
        f"证据组验收={len(proactive.group_records)} "
        f"通过/拒绝/未知={group_counts[0]}/{group_counts[1]}/{group_counts[2]}; "
        f"提交记录={len(proactive.submission_records)} "
        f"通过/拒绝/未知={submission_counts[0]}/{submission_counts[1]}/"
        f"{submission_counts[2]} submitted_groups={len(submitted_groups)} "
        f"组内归一={len(proactive.normalization_records)}"
    )

    z_adjustments = _proactive_numeric_values(
        proactive.candidate_records, "z_adjustment", "proposed_tz")
    pair_counts = _proactive_numeric_values(
        proactive.candidate_records, "pairs", "raw_ground_pairs")
    block_counts = _proactive_numeric_values(
        proactive.candidate_records, "blocks", "supported_blocks")
    spans = _proactive_numeric_values(
        proactive.candidate_records, "ground_span", "support_span")
    minor_spans = _proactive_numeric_values(
        proactive.candidate_records,
        "ground_minor_span",
        "support_minor_span",
    )
    candidate_details: List[str] = []
    for label, values in (
        ("z_adjustment", z_adjustments),
        ("pairs", pair_counts),
        ("blocks", block_counts),
        ("ground_span", spans),
        ("ground_minor_span", minor_spans),
    ):
        if values.size:
            candidate_details.append(
                f"{label}(min/median/max)={_numeric_min_median_max(values)}")
    if candidate_details:
        print(f"  候选质量: {' '.join(candidate_details)}")

    group_spreads = _proactive_numeric_values(
        proactive.group_records, "tz_spread", "z_spread")
    if group_spreads.size:
        print(
            "  证据组 tz_spread(min/median/max)="
            f"{_numeric_min_median_max(group_spreads)}"
        )
    normalized_weights_by_group: Dict[str, List[float]] = defaultdict(list)
    for record in proactive.submission_records:
        group_ids = _proactive_group_ids([record])
        normalized_weight = _record_float(record, "normalized_weight")
        if group_ids and normalized_weight is not None:
            normalized_weights_by_group[next(iter(group_ids))].append(normalized_weight)
    if normalized_weights_by_group:
        weight_l2 = np.asarray(
            [
                math.sqrt(sum(weight * weight for weight in weights))
                for weights in normalized_weights_by_group.values()
            ],
            dtype=np.float64,
        )
        print(
            "  提交组 normalized_weight_L2(min/median/max)="
            f"{_numeric_min_median_max(weight_l2)}"
        )
    if proactive.rejections:
        rejection_text = " ".join(
            f"{reason}={count}"
            for reason, count in sorted(proactive.rejections.items())
        )
        print(f"  主动纯Z拒绝: {rejection_text}")
    else:
        print("  主动纯Z拒绝: 0")

    if proactive.summary_records:
        latest_summary = proactive.summary_records[-1]
        summary_text = " ".join(
            f"{key}={value}"
            for key, value in latest_summary.items()
            if not key.startswith("_")
        )
        print(f"  主动纯Z汇总: {summary_text}")

    proactive_final = {
        key: value
        for key, value in stats.final_summary.items()
        if "proactive_ground_z" in key
    }
    graph_parts: List[str] = []
    if proactive.graph_summary_records:
        latest_graph = proactive.graph_summary_records[-1]
        classification_fields = (
            "internal_loops",
            "strict_internal_loops",
            "soft_internal_loops",
            "proactive_ground_z_loops",
            "internal_groups",
            "strict_internal_groups",
            "soft_internal_groups",
            "proactive_ground_z_groups",
        )
        classification = [
            f"{key}={latest_graph[key]}"
            for key in classification_fields
            if key in latest_graph
        ]
        if classification:
            loop_total = _record_float(latest_graph, "internal_loops")
            strict_loops = _record_float(
                latest_graph, "strict_internal_loops")
            soft_loops = _record_float(latest_graph, "soft_internal_loops")
            z_loops = _record_float(
                latest_graph,
                "proactive_ground_z_loops",
                "proactive_ground_z_only_loops",
            )
            loop_check = "unavailable"
            if None not in (loop_total, strict_loops, soft_loops, z_loops):
                loop_check = (
                    "OK"
                    if abs(loop_total - strict_loops - soft_loops - z_loops)
                    <= 1.0e-6
                    else "MISMATCH"
                )
            print(
                "  internal-only三分类: "
                f"{' '.join(classification)} edge_partition={loop_check}"
            )
            print(
                "  口径: edge三类互斥；group三类是成员数，"
                "同一evidence_group含strict与soft时可能重叠。"
            )
        for key in (
            "proactive_ground_z_loops",
            "proactive_ground_z_groups",
            "proactive_ground_z_only_loops",
            "proactive_ground_z_only_groups",
            "valid",
        ):
            if key in latest_graph:
                graph_parts.append(f"internal_{key}={latest_graph[key]}")
    if proactive.graph_adjustments:
        graph_parts.append(
            "adjustments=" + ",".join(
                f"{action}:{count}"
                for action, count in sorted(proactive.graph_adjustments.items())
            )
        )
    if proactive_final:
        graph_parts.extend(
            f"{key}={value}" for key, value in proactive_final.items())
        graph_parts.append(
            "note=proactive_current_is_internal_subset;pruned_proactive_is_pruned_internal_subset"
        )
    print(
        "  主动纯Z图安全: "
        + (" ".join(graph_parts) if graph_parts else "无专用调整/最终计数")
    )


def _print_mapping_log_summary(path: Path, stats: MappingLogStats) -> None:
    group_sizes = [len(edges) for edges in stats.anchor_edges_by_group.values()]
    strict_edges = sum(
        1 for edges in stats.anchor_edges_by_group.values() for soft in edges if not soft
    )
    soft_edges = sum(
        1 for edges in stats.anchor_edges_by_group.values() for soft in edges if soft
    )
    soft_groups = sum(any(edges) for edges in stats.anchor_edges_by_group.values())
    print("\n[回环日志]")
    print(f"文件={path}")
    if group_sizes:
        size_array = np.asarray(group_sizes, dtype=np.float64)
        print(
            f"提交的 internal groups={len(group_sizes)} anchors={sum(group_sizes)} "
            f"strict_anchors={strict_edges} soft_anchors={soft_edges} "
            f"含soft组={soft_groups} group_size(min/median/max)="
            f"{int(np.min(size_array))}/{np.median(size_array):.3g}/"
            f"{int(np.max(size_array))}"
        )
    else:
        print("提交的 internal groups=0 anchors=0（日志中未找到 v14/v15 局部锚记录）")
    normalized_pair_counts = [
        len(records) for records in stats.normalized_pair_anchors_by_group.values()
    ]
    normalized_pair_records = sum(normalized_pair_counts)
    normalized_evidence_groups = len(normalized_pair_counts)
    normalized_unique_pairs = sum(
        len(set(records))
        for records in stats.normalized_original_pairs_by_group.values()
    )
    normalized_multi_pair_groups = 0
    for group_id, records in stats.normalized_pair_anchors_by_group.items():
        original_pairs = stats.normalized_original_pairs_by_group.get(group_id, [])
        pair_count = len(set(original_pairs)) if original_pairs else len(records)
        normalized_multi_pair_groups += pair_count > 1
    normalization_text = (
        f"组内归一pair记录={normalized_pair_records} "
        f"evidence_groups={normalized_evidence_groups} "
        f"unique_original_pairs={normalized_unique_pairs} "
        f"multi_pair_groups={normalized_multi_pair_groups} "
        f"跨候选归一={stats.cross_candidate_normalizations}"
    )
    if normalized_pair_counts:
        pair_count_array = np.asarray(normalized_pair_counts, dtype=np.float64)
        normalization_text += (
            " pair_records_per_group(min/median/max)="
            f"{int(np.min(pair_count_array))}/"
            f"{np.median(pair_count_array):.3g}/"
            f"{int(np.max(pair_count_array))}"
        )
    print(
        f"{normalization_text} "
        f"soft预验={stats.soft_prevalidation_attempts} "
        f"soft预验通过={stats.soft_prevalidation_accepted} "
        f"soft提交={stats.soft_fallback_submitted} "
        f"无锚硬拒绝={stats.hard_rejected_without_anchor}"
    )
    _print_proactive_ground_z_summary(stats)
    if stats.graph_adjustments:
        action_text = " ".join(
            f"{action}={count}" for action, count in sorted(stats.graph_adjustments.items())
        )
        print(f"图安全调整: {action_text}")
    else:
        print("图安全调整: 0")
    if stats.final_summary:
        state = "REJECTED" if stats.final_graph_rejected else "ACCEPTED"
        details = " ".join(
            f"{key}={value}" for key, value in stats.final_summary.items()
        )
        print(f"最终位姿图={state} {details}")
    else:
        print("最终位姿图: 日志中未找到完成/拒绝汇总")


def _print_pose_correction_summary(
    corrections: PoseCorrections,
    trajectory: FrontendTrajectory,
) -> List[str]:
    warnings: List[str] = []
    print("\n[回环修正轨迹]")
    print(f"文件={corrections.path} corrections={corrections.size}")
    if corrections.size == 0:
        print("无修正记录")
        return warnings

    rotations = corrections.matrices[:, :3, :3]
    translations = corrections.matrices[:, :3, 3]
    translation_norms = np.linalg.norm(translations, axis=1)
    rotation_angles = np.asarray(
        [_rotation_angle_deg(rotation) for rotation in rotations], dtype=np.float64
    )
    max_translation_index = int(np.argmax(translation_norms))
    max_rotation_index = int(np.argmax(rotation_angles))
    endpoint_translation = float(translation_norms[-1])
    endpoint_rotation = float(rotation_angles[-1])

    print(
        f"最大绝对平移修正={translation_norms[max_translation_index]:.6g}m "
        f"{_event_location(max_translation_index, trajectory)}"
    )
    print(
        f"最大绝对旋转修正={rotation_angles[max_rotation_index]:.6g}deg "
        f"{_event_location(max_rotation_index, trajectory)}"
    )
    print(
        f"终点修正 translation={endpoint_translation:.6g}m "
        f"rotation={endpoint_rotation:.6g}deg"
    )

    if corrections.size > 1:
        adjacent_translations = np.empty(corrections.size - 1, dtype=np.float64)
        adjacent_rotations = np.empty(corrections.size - 1, dtype=np.float64)
        for index in range(1, corrections.size):
            relative_rotation = rotations[index - 1].T @ rotations[index]
            relative_translation = rotations[index - 1].T @ (
                translations[index] - translations[index - 1]
            )
            adjacent_translations[index - 1] = np.linalg.norm(relative_translation)
            adjacent_rotations[index - 1] = _rotation_angle_deg(relative_rotation)
        max_adjacent_translation_end = int(np.argmax(adjacent_translations)) + 1
        max_adjacent_rotation_end = int(np.argmax(adjacent_rotations)) + 1
        print(
            f"最大相邻平移修正={adjacent_translations[max_adjacent_translation_end - 1]:.6g}m "
            f"transition={max_adjacent_translation_end - 1}->{max_adjacent_translation_end} "
            f"end({_event_location(max_adjacent_translation_end, trajectory)})"
        )
        print(
            f"最大相邻旋转修正={adjacent_rotations[max_adjacent_rotation_end - 1]:.6g}deg "
            f"transition={max_adjacent_rotation_end - 1}->{max_adjacent_rotation_end} "
            f"end({_event_location(max_adjacent_rotation_end, trajectory)})"
        )
    else:
        print("相邻修正: 仅一个记录，无法计算")

    frontend_timestamps = trajectory.column("timestamp")
    timestamps_aligned = (
        corrections.size == trajectory.size
        and np.allclose(corrections.timestamps, frontend_timestamps, atol=1.0e-5, rtol=0.0)
    )
    if not timestamps_aligned:
        warnings.append(
            "loop_pose_graph 与 frontend 轨迹的数量或时间戳不一致，"
            "已跳过优化后轨迹重建；修正本身的最大值仍有效。"
        )
        return warnings

    raw_positions = np.column_stack(
        (
            trajectory.column("raw_tx"),
            trajectory.column("raw_ty"),
            trajectory.column("raw_tz"),
        )
    )
    optimized_positions = np.einsum("nij,nj->ni", rotations, raw_positions) + translations
    optimized_steps = np.linalg.norm(np.diff(optimized_positions, axis=0), axis=1)
    optimized_path = float(np.sum(optimized_steps))
    optimized_delta = optimized_positions[-1] - optimized_positions[0]
    optimized_gap_xy = float(np.linalg.norm(optimized_delta[:2]))
    optimized_gap_xyz = float(np.linalg.norm(optimized_delta))
    raw_path = float(trajectory.column("cumulative_path_m")[-1])
    print(
        f"优化后起末差 dxyz=({optimized_delta[0]:.3f},{optimized_delta[1]:.3f},"
        f"{optimized_delta[2]:.3f})m xy={optimized_gap_xy:.3f}m "
        f"xyz={optimized_gap_xyz:.3f}m"
    )
    print(
        f"优化后路程={optimized_path:.3f}m "
        f"相对raw变化={optimized_path - raw_path:+.3f}m"
    )
    return warnings


def _resolve_input(result_dir: Path, value: str) -> Path:
    path = Path(value).expanduser()
    return path if path.is_absolute() else result_dir / path


def _unit_interval(value: str) -> float:
    try:
        parsed = float(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"expected a number, got {value!r}") from error
    if not 0.0 <= parsed <= 1.0:
        raise argparse.ArgumentTypeError("value must be in [0, 1]")
    return parsed


def _positive_integer(value: str) -> int:
    try:
        parsed = int(value)
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"expected an integer, got {value!r}") from error
    if parsed <= 0:
        raise argparse.ArgumentTypeError("value must be positive")
    return parsed


def build_argument_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "只读分析一次 Navigation 建图结果的前端退化区间、回环组和位姿图修正。"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("result_dir", help="建图结果目录")
    parser.add_argument(
        "--frontend-file",
        default="frontend_keyframe_trajectory.txt",
        help="前端关键帧轨迹文件（相对结果目录或绝对路径）",
    )
    parser.add_argument(
        "--mapping-log",
        default="mapping.log",
        help="建图日志（相对结果目录或绝对路径；不存在时跳过）",
    )
    parser.add_argument(
        "--pose-graph",
        default="loop_pose_graph.txt",
        help="回环修正轨迹（相对结果目录或绝对路径；不存在时跳过）",
    )
    parser.add_argument(
        "--window-keyframes",
        type=_positive_integer,
        default=20,
        help="最差连续窗口的关键帧数量",
    )
    parser.add_argument(
        "--rotation-threshold",
        type=_unit_interval,
        default=0.10,
        help="rotation min/max 低可观测性门限",
    )
    parser.add_argument(
        "--translation-threshold",
        type=_unit_interval,
        default=0.10,
        help="translation min/max 低可观测性门限",
    )
    parser.add_argument(
        "--yaw-threshold",
        type=_unit_interval,
        default=0.40,
        help="conditional yaw 低可观测性门限",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    arguments = build_argument_parser().parse_args(argv)
    result_dir = Path(arguments.result_dir).expanduser().resolve()
    if not result_dir.is_dir():
        raise AnalysisError(f"result directory does not exist: {result_dir}")

    frontend_path = _resolve_input(result_dir, arguments.frontend_file)
    if not frontend_path.is_file():
        raise AnalysisError(
            f"missing required frontend trajectory: {frontend_path}. "
            "This result may predate the diagnostic output; rerun mapping with the current build."
        )
    trajectory = load_frontend_trajectory(frontend_path)

    print("Navigation 建图质量诊断（只读）")
    print(f"结果目录={result_dir}")
    _print_frontend_summary(trajectory)
    _print_observability(
        trajectory,
        arguments.rotation_threshold,
        arguments.translation_threshold,
        arguments.yaw_threshold,
        arguments.window_keyframes,
    )

    warnings: List[str] = []
    mapping_log_path = _resolve_input(result_dir, arguments.mapping_log)
    if mapping_log_path.is_file():
        _print_mapping_log_summary(mapping_log_path, parse_mapping_log(mapping_log_path))
    else:
        warnings.append(f"未找到 mapping.log，已跳过回环组/soft fallback 统计: {mapping_log_path}")

    pose_graph_path = _resolve_input(result_dir, arguments.pose_graph)
    if pose_graph_path.is_file():
        corrections = load_pose_corrections(pose_graph_path)
        warnings.extend(_print_pose_correction_summary(corrections, trajectory))
    else:
        warnings.append(f"未找到 loop_pose_graph.txt，可能未生成/未接受回环: {pose_graph_path}")

    if warnings:
        print("\n[提示]")
        for warning in warnings:
            print(f"- {warning}")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AnalysisError as error:
        print(f"error: {error}", file=sys.stderr)
        raise SystemExit(2)
