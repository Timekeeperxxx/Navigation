from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
HEADER = (
    PACKAGE_ROOT / "include" / "global_planner" / "hybrid_a_star.h"
).read_text(encoding="utf-8")
SOURCE = (PACKAGE_ROOT / "src" / "hybrid_a_star.cpp").read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_hybrid_search_has_fast_reference_and_unbounded_cancellable_fallback() -> None:
    reference = function_body(
        SOURCE, "bool Hybrid_A_Star::planOnPlangroundOnly"
    )
    hybrid = function_body(SOURCE, "bool Hybrid_A_Star::planOnHybrid")

    assert "reference_planning_time_ = 0.75" in HEADER
    assert "max_planning_time_ = 0.0" in HEADER
    assert "setMaxPlanningTime(reference_planning_time_)" in reference
    assert "setMaxPlanningTime(max_planning_time_)" in hybrid
    assert "setHeuristicWeight(heuristic_weight_)" in hybrid
    assert "seconds < 0.0" in function_body(
        SOURCE, "void Hybrid_A_Star::setMaxPlanningTime"
    )


def test_reference_astar_is_run_once_and_reused_directly_when_safe() -> None:
    hybrid = function_body(SOURCE, "bool Hybrid_A_Star::planOnHybrid")

    assert hybrid.count("planOnPlangroundOnly(") == 1
    assert "&reference_path" in hybrid
    assert "path = std::move(reference_path)" in hybrid
    assert "planground_fallback" not in hybrid


def test_reference_and_lazy_hybrid_candidates_are_fully_validated() -> None:
    reference = function_body(
        SOURCE, "bool Hybrid_A_Star::planOnPlangroundOnly"
    )
    hybrid = function_body(SOURCE, "bool Hybrid_A_Star::planOnHybrid")

    assert "setCancelChecker(cancel_checker_)" in reference
    assert "setUsePerceptionCosts(false)" in reference
    assert "setEdgeValidator(edge_validator_)" not in reference
    assert "setCancelChecker(cancel_checker_)" in hybrid
    assert "setUsePerceptionCosts(false)" in hybrid
    assert "setIndexEdgeValidator(" in hybrid
    assert "a_star_planner_->setEdgeValidator(edge_validator_)" not in hybrid
    assert "edge_validator_(" in hybrid
    assert "Using validated fill_footprint reference directly" in hybrid
    assert "Lazy footprint validation accepted" in hybrid
    assert "newly_blocked == 0" in hybrid


def test_nondefault_downsample_setting_rebuilds_hybrid_cloud() -> None:
    setter = function_body(
        SOURCE, "void Hybrid_A_Star::setDownsampleLeafSize"
    )

    assert "hybrid_downsample_leaf_size_ = leaf_size" in setter
    assert "rebuildHybridCloud()" in setter


def test_smoothed_path_is_validated_with_safe_raw_fallback() -> None:
    output = function_body(
        SOURCE, "void Hybrid_A_Star::smoothPathToRosPath"
    )

    assert "validate_edges(path_points)" in output
    assert "build_validated_raw_path()" in output
    assert "kRawInterpolationSpacing = 0.1" in output
    assert "edge_validator_(from, to)" in output
    assert "edge_validator_(previous, sample)" not in output
    assert "final B2 path gate independently samples every published translation" in output
    assert "Raw A* path also failed edge validation" in output
    assert "Exact-goal connector rejected" in output
