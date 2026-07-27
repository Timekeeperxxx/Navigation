#ifndef SCAN_PLANNER_FINAL_YAW_LATCH_H_
#define SCAN_PLANNER_FINAL_YAW_LATCH_H_

#include <cmath>

namespace scan_planner
{

enum class FinalYawPhase
{
  TRACKING_TRAJECTORY,
  ALIGNING,
  COMPLETE,
};

struct FinalYawDecision
{
  bool hold_position = false;
  bool alignment_started = false;
  bool alignment_completed = false;
};

class FinalYawLatch
{
public:
  void reset()
  {
    phase_ = FinalYawPhase::TRACKING_TRAJECTORY;
  }

  FinalYawDecision update(
      bool trajectory_finished,
      double position_error,
      double position_tolerance,
      bool final_yaw_enabled,
      bool have_goal_yaw,
      double yaw_error,
      double yaw_tolerance)
  {
    FinalYawDecision decision;

    if (phase_ == FinalYawPhase::COMPLETE)
    {
      decision.hold_position = true;
      return decision;
    }

    if (phase_ == FinalYawPhase::ALIGNING)
    {
      decision.hold_position = true;
      if (
          !final_yaw_enabled ||
          !have_goal_yaw ||
          std::abs(yaw_error) <= yaw_tolerance)
      {
        phase_ = FinalYawPhase::COMPLETE;
        decision.alignment_completed = true;
      }
      return decision;
    }

    if (!trajectory_finished || position_error > position_tolerance)
      return decision;

    decision.hold_position = true;
    if (
        final_yaw_enabled &&
        have_goal_yaw &&
        std::abs(yaw_error) > yaw_tolerance)
    {
      phase_ = FinalYawPhase::ALIGNING;
      decision.alignment_started = true;
      return decision;
    }

    phase_ = FinalYawPhase::COMPLETE;
    decision.alignment_completed = true;
    return decision;
  }

  bool isAligning() const
  {
    return phase_ == FinalYawPhase::ALIGNING;
  }

  bool isComplete() const
  {
    return phase_ == FinalYawPhase::COMPLETE;
  }

private:
  FinalYawPhase phase_ = FinalYawPhase::TRACKING_TRAJECTORY;
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER_FINAL_YAW_LATCH_H_
