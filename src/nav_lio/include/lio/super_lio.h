

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <deque>
#include <vector>
#include <iostream>
#include <cassert>
#include <cstdint>
#include <filesystem>
#include <limits>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>

#include "basic/alias.h"
#include "common/ds.h"
#include "common/timer.h"
#include "params.h"
#include "ESKF.h"
#include "OctVoxMap/OctVoxMap.hpp"
#include "OctVoxMap/VoxelGridFilter.h"
#include "ros/ROSWrapper.h"

namespace LI2Sup{

class SuperLIO{
public:
  SuperLIO(){};
  ~SuperLIO(){};

  void setROSWrapper(const ROSWrapper::Ptr& wrapper){
    data_wrapper_ = wrapper;
  }
  virtual void init();
  void process();
  void saveMap();
  void printTimeRecord();

protected:
  void stateWaitKFInit();
  void stateWaitMapInit();
  void stateProcess();
  virtual bool kf_init();
  virtual bool map_init();
  bool Propagation_Undistort();
  void DownSample();
  void Observe();
  void updateGravityReference();
  struct LevelPlaneObservation {
    bool valid = false;
    BASIC::V3 normal_body = BASIC::V3::UnitZ();
    int candidate_count = 0;
    int inlier_count = 0;
    double inlier_ratio = 0.0;
    double rms = 0.0;
    double gravity_angle_deg = 0.0;
    double innovation_deg = 0.0;
  };
  LevelPlaneObservation estimateLevelPlane() const;
  struct WallYawObservation {
    bool extraction_attempted = false;
    bool valid = false;
    bool plane_valid = false;
    bool reference_valid = false;
    BASIC::V3 normal_body = BASIC::V3::UnitX();
    BASIC::V3 target_normal_world = BASIC::V3::UnitX();
    int candidate_count = 0;
    int inlier_count = 0;
    double inlier_ratio = 0.0;
    double rms = 0.0;
    double vertical_angle_deg = 0.0;
    double vertical_span = 0.0;
    double horizontal_span = 0.0;
    double innovation_deg = 0.0;
    double lidar_yaw_information_ratio = 0.0;
    double observability_gate = 0.0;
    int reference_index = -1;
  };
  WallYawObservation estimateWallYaw() const;
  void prepareWallYawConstraint(
      WallYawObservation& observation,
      const BASIC::SE3& pose,
      double lidar_yaw_information_ratio);
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void updateMapPreview(double timestamp);
  void ProcessCaceMap();
  void maybeCacheLoopKeyFrame(const BASIC::SE3& pose, double timestamp);
  void saveLoopClosedMap();

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  struct LoopKeyFrame {
    BASIC::SE3 pose;
    BASIC::CloudPtr cloud_body;
    double timestamp = 0.0;
    int frame_id = 0;
  };
  StateFn state_fn_;
  ESKF::Ptr kf_;
  OctVoxMapType::Ptr ivox_;
  VoxelGridClosest<BASIC::PointType> voxel_grid_fliter_;
  ROSWrapper::Ptr data_wrapper_;
  MeasureGroup measures_;
  
  bool flg_init_ = false;
  bool flg_first_scan_ = true;
  std::vector<DynamicState> propagate_states_;
  BASIC::CloudPtr scan_undistort_full_;
  BASIC::CloudPtr ds_undistort_;
  BASIC::CloudPtr point_map_, world_pc_, ds_world_;
  BASIC::CloudPtr map_preview_, map_preview_pending_;
  int map_preview_scan_count_ = 0;
  float map_preview_effective_leaf_size_ = 0.0f;
  int frame_num_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;
  bool has_last_accepted_pose_ = false;
  SysState last_accepted_state_;
  ESKF::COV last_accepted_covariance_ = ESKF::COV::Identity();
  bool has_last_accepted_state_ = false;
  bool observation_valid_ = true;
  std::uint64_t rejected_undistortion_count_ = 0;
  std::chrono::steady_clock::time_point last_undistortion_warning_time_{};
  std::size_t effective_match_count_ = 0;
  int consecutive_invalid_observations_ = 0;
  std::deque<IMUData> imu_init_window_;
  std::chrono::steady_clock::time_point last_imu_motion_warning_time_{};
  struct GravityDirectionSample {
    double timestamp = 0.0;
    BASIC::V3 up_world = BASIC::V3::UnitZ();
  };
  std::deque<GravityDirectionSample> gravity_direction_window_;
  BASIC::V3 gravity_reference_world_ = BASIC::V3::UnitZ();
  double imu_reference_accel_norm_ = 0.0;
  double last_gravity_sample_time_ = -1.0;
  bool gravity_reference_valid_ = false;
  std::uint64_t level_constraint_accepted_count_ = 0;
  std::uint64_t level_constraint_rejected_count_ = 0;
  std::chrono::steady_clock::time_point last_level_constraint_log_time_{};
  struct WallYawReferenceSample {
    double axis_rad = 0.0;
    BASIC::V3 position = BASIC::V3::Zero();
    int frame = 0;
  };
  struct WallYawReference {
    double axis_rad = 0.0;
    BASIC::V3 center = BASIC::V3::Zero();
    std::uint64_t last_used_frame = 0;
    std::uint64_t accepted_count = 0;
  };
  std::deque<WallYawReferenceSample> wall_yaw_reference_samples_;
  std::vector<WallYawReference> wall_yaw_references_;
  std::uint64_t wall_yaw_constraint_accepted_count_ = 0;
  std::uint64_t wall_yaw_constraint_rejected_count_ = 0;
  std::uint64_t wall_yaw_constraint_gated_count_ = 0;
  std::uint64_t wall_yaw_extraction_skipped_count_ = 0;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  std::vector<uint8_t> effect_mask_;
  std::vector<uint8_t> effect_knn_mask_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  std::vector<LoopKeyFrame> loop_keyframes_;
  BASIC::V3 last_loop_keyframe_pos_ = BASIC::V3(
      std::numeric_limits<BASIC::scalar>::quiet_NaN(),
      std::numeric_limits<BASIC::scalar>::quiet_NaN(),
      std::numeric_limits<BASIC::scalar>::quiet_NaN());
  int pcd_index_ = -1;

  Timer time_record_;
};

} // namespace END.

#endif
