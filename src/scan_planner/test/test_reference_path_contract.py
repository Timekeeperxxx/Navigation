from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
SOURCE = (PACKAGE_ROOT / "src" / "scan_replan_fsm.cpp").read_text(
    encoding="utf-8"
)
MANAGER_SOURCE = (PACKAGE_ROOT / "src" / "planner_manager.cpp").read_text(
    encoding="utf-8"
)


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


def test_dense_reference_path_is_not_fit_as_global_waypoint_polynomial() -> None:
    plan = function_body(
        SOURCE,
        "bool SCANReplanFSM::planGlobalTrajByWaypoints",
    )

    reference_branch = plan.index(
        "navi_mode_ == NAVI_MODE::REFERENCE_PATH\n"
        "        ? planner_manager_->planGlobalTraj("
    )
    dense_fit_branch = plan.index(
        ": planner_manager_->planGlobalTrajWaypoints("
    )
    assert reference_branch < dense_fit_branch

    # RViz must show the original verified polyline. Sampling the placeholder
    # polynomial here would both hide corner geometry and reintroduce cutting.
    assert "if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)" in plan
    assert "gloabl_traj = waypoints;" in plan


def test_reference_local_target_bypasses_global_data_geometry() -> None:
    reference_target = function_body(
        SOURCE,
        "bool SCANReplanFSM::getReferencePathLocalTarget",
    )
    target_dispatch = function_body(
        SOURCE,
        "void SCANReplanFSM::getLocalTarget",
    )

    assert "active_waypoints_" in reference_target
    assert "global_data_" not in reference_target

    early_return = target_dispatch.index(
        "if (getReferencePathLocalTarget())\n      return;"
    )
    first_global_data_use = target_dispatch.index("global_data_")
    assert early_return < first_global_data_use


def test_reference_replans_and_execution_safety_ignore_global_placeholder() -> None:
    replan = function_body(
        SOURCE,
        "bool SCANReplanFSM::planFromCurrentTraj",
    )
    dispatch = function_body(
        SOURCE,
        "bool SCANReplanFSM::callReboundReplan",
    )
    safety = function_body(
        SOURCE,
        "bool SCANReplanFSM::isTrajectorySafeForExecution",
    )

    assert (
        "if (navi_mode_ != NAVI_MODE::REFERENCE_PATH &&\n"
        "        !planner_manager_->planGlobalTraj("
    ) in replan
    assert (
        "if (navi_mode_ != NAVI_MODE::REFERENCE_PATH && "
        "!adjustGlobalTargetIfOccupied())"
    ) in replan

    # Execution is generated from the selected local target and validated on
    # the resulting B-spline; the placeholder is absent from both operations.
    assert "getLocalTarget();" in dispatch
    assert "local_target_pt_" in dispatch
    assert "isTrajectorySafeForExecution(" in dispatch
    assert "info->position_traj_," in dispatch
    assert "&validated_yaw_schedule" in dispatch
    assert "bspline.yaw_pts = validated_yaw_schedule" in dispatch
    assert "global_data_" not in dispatch
    assert "global_data_" not in safety


def test_all_reference_candidates_are_direction_guarded_before_publish() -> None:
    dispatch = function_body(
        SOURCE,
        "bool SCANReplanFSM::callReboundReplan",
    )

    direction_guard = dispatch.index(
        "!isB2TrajectoryDirectionSafe("
    )
    execution_safety = dispatch.index("isTrajectorySafeForExecution(")
    publish = dispatch.index("bspline_pub_->publish")
    assert direction_guard < execution_safety < publish
    assert (
        "navi_mode_ == NAVI_MODE::REFERENCE_PATH &&\n"
        "          !isB2TrajectoryDirectionSafe"
    ) in dispatch
    assert (
        "flag_randomPolyTraj &&\n"
        "          !isB2TrajectoryDirectionSafe"
    ) not in dispatch
    assert "hasB2ImmediateDynamicObstacle(" in dispatch
    assert "enforce_initial_reference_cone = false" in dispatch
    assert "A verified obstacle bypass may have to begin almost sideways" in dispatch
    assert "shouldUseB2BoundedObstacleDetour(" in dispatch
    assert "b2_obstacle_recovery_latched_" in dispatch
    assert "searchB2ForwardDetour(" in SOURCE


def test_latched_obstacle_recovery_only_uses_verified_current_heading_seed() -> None:
    seed = function_body(
        SOURCE,
        "bool SCANReplanFSM::applyB2ForwardStartSeed",
    )

    body_heading_seed = seed.index("makeB2ForwardSeedVelocity(")
    frozen = seed.index("if (b2_obstacle_recovery_latched_)")
    accept_seed = seed.index("if (seeded)", frozen)
    zero_velocity = seed.index("start_vel_.setZero();", accept_seed)
    reference_heading_seed = seed.index("makeB2ReferenceSeedVelocity(")
    assert body_heading_seed < frozen < accept_seed < zero_velocity < reference_heading_seed


def test_local_rebound_uses_reference_subpath_without_flattening_terrain() -> None:
    target = function_body(
        SOURCE,
        "bool SCANReplanFSM::getReferencePathLocalTarget",
    )
    dispatch = function_body(
        SOURCE,
        "bool SCANReplanFSM::callReboundReplan",
    )
    rebound = function_body(
        MANAGER_SOURCE,
        "bool SCANPlannerManager::reboundReplan",
    )

    assert "local_reference_guide_.clear();" in target
    assert "appendGuidePoint(start_pt_);" in target
    assert "appendGuidePoint(local_target_pt_);" in target
    assert "&local_reference_guide_" in dispatch
    assert "!flag_randomPolyTraj" in dispatch
    assert "resampleReferenceGuide(" in rebound
    assert "if (have_reference_guide ||" in rebound
    assert "if (!using_reference_guide)" in rebound
    assert "applyLinearZReference(" in rebound


def test_reference_attraction_is_removed_when_obstacle_intersects_guide() -> None:
    rebound = function_body(
        MANAGER_SOURCE,
        "bool SCANPlannerManager::reboundReplan",
    )

    occupancy_check = rebound.index(
        "grid_map_->getInflateOccupancy(point, yaw)"
    )
    clear_reference = rebound.index(
        "bspline_optimizer_rebound_->clearReboundReference();"
    )
    init_astar = rebound.index(
        "bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true)"
    )
    assert occupancy_check < clear_reference < init_astar
    assert "setReboundReference(point_set)" in rebound


def test_every_new_reference_path_publishes_handoff_before_generation() -> None:
    callback = function_body(
        SOURCE,
        "void SCANReplanFSM::pathCallback",
    )

    accepted = callback.index("if (success)")
    odom_gate = callback.index("if (have_odom_)", accepted)
    stationary_handoff = callback.index(
        "callEmergencyStop(odom_pos_);",
        odom_gate,
    )
    generation = callback.index(
        "changeFSMExecState(GEN_NEW_TRAJ, \"TRIG\");",
        stationary_handoff,
    )

    assert accepted < odom_gate < stationary_handoff < generation
    # The first goal is received while the FSM is still INIT.  Excluding INIT
    # here would leave the controller's pending terminal yaw unbound forever.
    handoff_gate = callback[odom_gate:stationary_handoff]
    assert "exec_state_" not in handoff_gate


def test_reverse_recovery_preflights_fresh_path_and_keeps_target() -> None:
    request = function_body(
        SOURCE,
        "bool SCANReplanFSM::requestB2ReverseRecovery",
    )
    finish = function_body(
        SOURCE,
        "void SCANReplanFSM::finishProcess",
    )

    assert "!safety_execution_frozen_" not in request
    assert "!have_target_" in request
    assert "shouldTriggerB2ReverseRecovery(" in request
    assert "hasB2ImmediateDynamicObstacle(" in request
    assert "ground/slope/direction/optimizer failures may not command reverse" in request
    assert "changeFSMExecState(REVERSE_RECOVERY" in request
    assert "const bool keep_active_target = have_target_;" in finish


def test_only_reference_segment_ending_at_global_goal_is_terminal() -> None:
    dispatch = function_body(
        SOURCE,
        "bool SCANReplanFSM::callReboundReplan",
    )
    emergency = function_body(
        SOURCE,
        "bool SCANReplanFSM::callEmergencyStop",
    )

    assert "bspline.terminal_goal =" in dispatch
    assert "active_waypoints_.back()" in dispatch
    assert "bspline.terminal_goal = false;" in emergency


def test_terminal_reference_spline_converges_without_close_goal_reverse() -> None:
    execute = function_body(
        SOURCE,
        "void SCANReplanFSM::execFSMCallback",
    )

    assert "const bool reference_terminal_trajectory =" in execute
    assert "reference_final_goal_distance <= 0.20" in execute
    assert "Terminal trajectory time elapsed" in execute
    assert "keep endpoint tracking instead of" in execute


def test_completed_reverse_round_replans_without_retry_limit() -> None:
    callback = function_body(
        SOURCE,
        "void SCANReplanFSM::reverseRecoveryStatusCallback",
    )

    assert "Another reverse round " in callback
    assert "remains allowed only if" in callback
    assert "replan_fail_count_ = 0;" in callback
    assert "replan_not_before_seconds_ = 0.0;" in callback
    assert "changeFSMExecState(GEN_NEW_TRAJ" in callback
    assert "max_replan_fail_count_" not in callback
