#ifndef SCAN_PLANNER_REPLAN_FAILURE_POLICY_H_
#define SCAN_PLANNER_REPLAN_FAILURE_POLICY_H_

#include <algorithm>

namespace scan_planner
{

inline bool frozenReplanAttemptDue(
    const bool execution_frozen,
    const double now_seconds,
    const double retry_not_before_seconds)
{
  return !execution_frozen || now_seconds >= retry_not_before_seconds;
}

inline double nextFrozenReplanTime(
    const double now_seconds,
    const double retry_interval_seconds)
{
  return now_seconds + std::max(0.05, retry_interval_seconds);
}

inline bool replanAttemptDue(
    const double now_seconds,
    const double retry_not_before_seconds)
{
  return now_seconds >= retry_not_before_seconds;
}

inline double nextReplanTime(
    const double now_seconds,
    const double retry_interval_seconds)
{
  return now_seconds + std::max(0.05, retry_interval_seconds);
}

inline bool shouldEscalateReplanFailure(
    const int failure_count,
    const int max_failure_count,
    const bool execution_frozen,
    const bool keep_active_target = false)
{
  return !execution_frozen && !keep_active_target &&
         max_failure_count > 0 &&
         failure_count >= max_failure_count;
}

} // namespace scan_planner

#endif // SCAN_PLANNER_REPLAN_FAILURE_POLICY_H_
