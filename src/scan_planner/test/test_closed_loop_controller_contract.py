from pathlib import Path


SOURCE = (
    Path(__file__).parents[1] / "src" / "closed_loop_controller.cpp"
).read_text(encoding="utf-8")


def function_body(signature: str) -> str:
    start = SOURCE.index(signature)
    brace = SOURCE.index("{", start)
    depth = 0
    for index in range(brace, len(SOURCE)):
        if SOURCE[index] == "{":
            depth += 1
        elif SOURCE[index] == "}":
            depth -= 1
            if depth == 0:
                return SOURCE[brace + 1 : index]
    raise AssertionError(f"unterminated function: {signature}")


def test_final_yaw_sweep_uses_live_rotation_centre() -> None:
    publish = function_body("void publishExecutionPath")

    assert "terminal_goal_trajectory" in publish
    assert "live_rotation_point.x() = odom_pos.x()" in publish
    assert "live_rotation_point.y() = odom_pos.y()" in publish
    assert "append_point_with_yaw(live_rotation_point, yaw)" in publish


def test_final_yaw_requires_generation_matched_safety_ack() -> None:
    ack = function_body("void finalYawValidationCallback")
    command = function_body("void cmdCallback")

    assert (
        "std::abs(generation - final_yaw_validation_generation) > 1e-3"
        in ack
    )
    assert (
        "final_yaw_validation_state = FinalYawValidationState::APPROVED"
        in ack
    )
    approved_gate = (
        "final_yaw_validation_state ==\n"
        "          FinalYawValidationState::APPROVED"
    )
    assert approved_gate in command
    assert command.index(approved_gate) < command.index(
        "cmd.angular.z = enforceMinimumSignedMagnitude"
    )


def test_timeout_and_safety_denial_never_authorize_rotation() -> None:
    command = function_body("void cmdCallback")

    assert "final_yaw_validation_timeout" in command
    assert "final_yaw_safety_aborted = true" in command
    assert "final-yaw safety ACK timed out" in command
    assert "final_yaw_preflight_delay" not in SOURCE


def test_new_goal_and_trajectory_reset_validation_generation() -> None:
    assert function_body("void bsplineCallback").count(
        "resetFinalYawValidation()"
    ) == 1
    assert function_body("void goalYawCallback").count(
        "resetFinalYawValidation()"
    ) == 1
    assert function_body("void goalPoseCallback").count(
        "resetFinalYawValidation()"
    ) == 1


def test_reverse_recovery_command_is_strictly_one_dimensional() -> None:
    command = function_body("void cmdCallback")
    recovery = command[
        command.index("if (reverse_recovery_policy.active())") :
        command.index("if (reverse_recovery_policy.holding())")
    ]

    assert "cmd.linear.x =" in recovery
    assert "-std::min(reverse_recovery_speed" in recovery
    assert "cmd.linear.y = 0.0;" in recovery
    assert "cmd.angular.z = 0.0;" in recovery
    assert "allow_reverse" not in recovery


def test_reverse_recovery_request_uses_generation_fresh_safety_preflight() -> None:
    request = function_body("void reverseRecoveryRequestCallback")
    command = function_body("void cmdCallback")

    assert "if (!safety_execution_frozen)" not in request
    assert "reverse_recovery_policy.begin(" in request
    assert "last_safety_execution_frozen_time >=" in command
    assert "reverse_recovery_generation_time" in command
    assert "!have_fresh_reverse_safety_sample || safety_execution_frozen" in command
    assert "publishExecutionFrozen(true);" in command


def test_only_global_terminal_spline_can_latch_final_yaw() -> None:
    callback = function_body("void bsplineCallback")
    command = function_body("void cmdCallback")

    assert "msg->terminal_goal && !msg->emergency_stop" in callback
    terminal_gate = command.index("if (terminal_goal_trajectory)")
    latch = command.index("final_yaw_latch.update(")
    assert terminal_gate < latch


def test_reverse_recovery_path_height_is_latched_not_feedback_from_odom() -> None:
    publish = function_body("void publishReverseRecoveryPath")

    assert "reverse_recovery_policy.startZ()" in publish
    assert "odom_pos.z()" not in publish


def test_normal_trajectory_reverse_remains_independently_disabled() -> None:
    command = function_body("void cmdCallback")

    assert (
        "body_vx, allow_reverse ? -max_vx : 0.0, max_vx"
        in command
    )
    assert (
        'getParamWithDefault<bool>("allow_reverse", false)'
        in SOURCE
    )
