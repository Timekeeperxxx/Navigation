

#ifndef SUPER_LIO_RELOCATION_H_
#define SUPER_LIO_RELOCATION_H_


#include "super_lio.h"

#include <future>
#include <pcl/kdtree/kdtree_flann.h>


namespace LI2Sup{

class SuperLIOReLoc : public SuperLIO {
public:
  SuperLIOReLoc(){
    BASIC::V3 init_t = BASIC::V3(g_init_px, g_init_py, g_init_pz);
    Eigen::Matrix3d init_R = (Eigen::AngleAxisd(g_init_yaw / 180 * M_PI, Eigen::Vector3d::UnitZ()) *
                              Eigen::AngleAxisd(g_init_pitch /180 * M_PI, Eigen::Vector3d::UnitY()) *
                              Eigen::AngleAxisd(g_init_roll / 180 * M_PI, Eigen::Vector3d::UnitX())).toRotationMatrix();
                
    BASIC::M3 init_R2 = init_R.cast<BASIC::scalar>();             
      
    re_init_pose_ = BASIC::SE3(BASIC::SO3(init_R2), init_t);
  };
  ~SuperLIOReLoc(){};

  void init() override;

private:
  bool kf_init() override;
  bool map_init() override;
  void UpdateMap() override;
  void Output() override;
  void appendAnchorFrame();
  bool tryFixedMapAnchor();

  struct AnchorFrame {
    BASIC::CloudPtr cloud_odom;
    BASIC::SE3 odom_pose;
    double timestamp = 0.0;
  };

  struct AnchorAlignmentMetrics {
    bool valid = false;
    double overlap = 0.0;
    double rmse = std::numeric_limits<double>::infinity();
    double support_major = 0.0;
    double support_minor = 0.0;
    int structural_points = 0;
  };

  struct AnchorRegistrationResult {
    bool evaluated = false;
    bool accepted = false;
    BASIC::SE3 estimated_map_from_odom;
    AnchorAlignmentMetrics full_metrics;
    AnchorAlignmentMetrics early_metrics;
    AnchorAlignmentMetrics late_metrics;
    double requested_translation = 0.0;
    double requested_yaw_deg = 0.0;
    double requested_tilt_deg = 0.0;
    std::string reason = "not_evaluated";
  };

  AnchorAlignmentMetrics evaluateAnchorAlignment(
      const BASIC::CloudPtr& source_odom,
      const BASIC::CloudPtr& target_map,
      const Eigen::Matrix4f& map_from_odom,
      double sensor_height_map) const;
  AnchorRegistrationResult computeFixedMapAnchor(
      const std::deque<AnchorFrame>& frames,
      const BASIC::SE3& initial_map_from_odom,
      const BASIC::SE3& current_map_pose) const;
  bool applyFixedMapAnchor(const AnchorRegistrationResult& result);

private:
  BASIC::CloudPtr init_obs_data_;
  bool flg_get_init_guess_ = false;
  BASIC::SE3 re_init_pose_;
  BASIC::SE3 odom_pose_;
  BASIC::SE3 map_to_odom_;
  BASIC::SE3 local_reference_map_pose_;
  bool relocation_frames_initialized_ = false;
  std::uint64_t localization_rejected_count_ = 0;
  pcl::KdTreeFLANN<BASIC::PointType>::Ptr fixed_map_tree_;
  std::deque<AnchorFrame> anchor_frames_;
  std::uint64_t anchor_frame_counter_ = 0;
  std::uint64_t anchor_success_count_ = 0;
  std::uint64_t anchor_failure_count_ = 0;
  int consecutive_anchor_failures_ = 0;
  bool relocation_health_valid_ = true;
  std::future<AnchorRegistrationResult> anchor_future_;
  bool anchor_worker_running_ = false;
  BASIC::SE3 pending_anchor_estimate_;
  bool pending_anchor_estimate_valid_ = false;
};

} // namespace END.

#endif
