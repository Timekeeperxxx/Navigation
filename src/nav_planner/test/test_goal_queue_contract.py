from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
HEADER = (
    PACKAGE_ROOT / "include" / "global_planner" / "global_planner.h"
).read_text(encoding="utf-8")
SOURCE = (PACKAGE_ROOT / "src" / "global_planner.cpp").read_text(
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


def enclosing_block(source: str, statement: str) -> str:
    statement_index = source.index(statement)
    stack: list[int] = []
    for index, character in enumerate(source[:statement_index]):
        if character == "{":
            stack.append(index)
        elif character == "}":
            stack.pop()
    if not stack:
        raise AssertionError(f"statement is not inside a block: {statement}")

    opening = stack[-1]
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1 : index]
    raise AssertionError(f"unterminated enclosing block: {statement}")


def test_pending_goal_carries_pose_and_generation_as_one_item() -> None:
    assert "struct PendingGoal" in HEADER
    pending = HEADER[
        HEADER.index("struct PendingGoal") : HEADER.index(
            "std::mutex pending_goal_mutex_"
        )
    ]

    assert "geometry_msgs::msg::PoseStamped pose;" in pending
    assert "uint64_t generation{0};" in pending
    assert "bool enforce_goal_yaw{false};" in pending
    assert "std::optional<PendingGoal> pending_goal_;" in HEADER


def test_generation_is_committed_with_pending_pose_under_the_same_lock() -> None:
    enqueue = function_body(
        SOURCE,
        "void GlobalPlanner::enqueueGoal",
    )
    locked = enclosing_block(
        enqueue,
        "std::lock_guard<std::mutex> lock(pending_goal_mutex_);",
    )

    increment = locked.index("generation = ++planning_generation_;")
    commit = locked.index(
        "PendingGoal{normalized_goal, generation, enforce_goal_yaw};"
    )
    assert increment < commit

    # Publishing and waking the worker happen only after the complete
    # pose/generation item has become visible.
    committed_item = "PendingGoal{normalized_goal, generation, enforce_goal_yaw};"
    assert enqueue.index(committed_item) < enqueue.index(
        'publishPlanningStatus("queued"'
    )
    assert enqueue.index(committed_item) < enqueue.index(
        "pending_goal_cv_.notify_one();"
    )


def test_worker_uses_the_generation_stored_with_the_dequeued_pose() -> None:
    worker = function_body(
        SOURCE,
        "void GlobalPlanner::goalWorkerLoop",
    )

    dequeue = worker.index("pending = *pending_goal_;")
    reset = worker.index("pending_goal_.reset();")
    dispatch = worker.index(
        "pending.pose, pending.generation, pending.enforce_goal_yaw"
    )
    assert dequeue < reset < dispatch
    assert "planning_generation_.load()" not in worker
    assert "geometry_msgs::msg::PoseStamped goal;" not in worker
