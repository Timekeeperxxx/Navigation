#ifndef SCAN_PLANNER_HEADING_ALIGNMENT_LATCH_H_
#define SCAN_PLANNER_HEADING_ALIGNMENT_LATCH_H_

#include <algorithm>
#include <cmath>

namespace scan_planner
{

// Schmitt trigger for B2's turn-in-place gate.  A single threshold lets the
// controller alternate between rotate and translate whenever odometry noise
// or a curved spline moves the desired heading across that threshold.
class HeadingAlignmentLatch
{
public:
  void configure(double stop_threshold, double resume_threshold)
  {
    stop_threshold_ = std::max(0.0, stop_threshold);
    resume_threshold_ = std::max(
        0.0, std::min(resume_threshold, stop_threshold_));
    aligning_ = false;
  }

  void reset()
  {
    aligning_ = false;
  }

  bool update(double heading_error)
  {
    const double error = std::abs(heading_error);
    if (aligning_)
    {
      if (error <= resume_threshold_)
        aligning_ = false;
    }
    else if (error > stop_threshold_)
    {
      aligning_ = true;
    }
    return aligning_;
  }

  bool isAligning() const
  {
    return aligning_;
  }

  double stopThreshold() const
  {
    return stop_threshold_;
  }

  double resumeThreshold() const
  {
    return resume_threshold_;
  }

private:
  double stop_threshold_{0.5};
  double resume_threshold_{0.35};
  bool aligning_{false};
};

}  // namespace scan_planner

#endif  // SCAN_PLANNER_HEADING_ALIGNMENT_LATCH_H_
