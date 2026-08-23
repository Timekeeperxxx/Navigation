

#ifndef SUPER_LIO_H_
#define SUPER_LIO_H_

#include <queue>
#include <deque>
#include <vector>
#include <iostream>
#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <mutex>
#include <string>
#include <thread>

#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/gicp.h>
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
  ~SuperLIO();

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
    bool plane_valid = false;
    bool slope_rejected = false;
    bool slope_protection_active = false;
    bool slope_enter_evidence = false;
    bool slope_exit_evidence = false;
    bool dynamic_gravity_mismatch = false;
    BASIC::V3 normal_body = BASIC::V3::UnitZ();
    double plane_offset_body = 0.0;
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
    double lidar_raw_yaw_information_ratio = 0.0;
    double lidar_conditional_yaw_information_ratio = 0.0;
    double lidar_translation_information_ratio = 0.0;
    double observability_gate = 0.0;
    double recapture_scene_quality = 0.0;
    double recapture_constraint_gate = 0.0;
    double recapture_reference_distance_m = 0.0;
    double recapture_core_radius_m = 0.0;
    std::uint64_t recapture_reference_age_frames = 0;
    double frame_target_delta_deg = 0.0;
    bool recapture = false;
    int recapture_sample_count = 0;
    std::string recapture_gate_reason = "not_evaluated";
    int reference_index = -1;
  };
  WallYawObservation estimateWallYaw() const;
  void prepareWallYawConstraint(
      WallYawObservation& observation,
      const BASIC::SE3& pose,
      double lidar_raw_yaw_information_ratio,
      double lidar_conditional_yaw_information_ratio,
      double lidar_translation_information_ratio);
  virtual void UpdateMap();
  virtual void Output();
  void caceData();
  void updateMapPreview(double timestamp);
  bool initializeMapPersistence();
  enum class CloudWriteEnqueueStatus {
    Queued,
    Busy,
    Unavailable,
  };
  CloudWriteEnqueueStatus enqueueCloudWrite(
      const std::filesystem::path& path,
      const BASIC::CloudPtr& cloud,
      bool downsample,
      const std::string& context,
      bool wait_for_space);
  bool flushMapFragment(bool force);
  void mapWriterLoop();
  bool stopMapWriter();
  bool ProcessCaceMap();
  BASIC::CloudPtr loadLoopKeyFrameCloud(std::size_t index) const;
  void cleanupMapPersistenceFiles();
  void maybeCacheLoopKeyFrame(const BASIC::SE3& pose, double timestamp);
  bool initializeOnlineLoopWorker();
  void scheduleOnlineLoopTask(std::size_t later_index);
  void onlineLoopWorker();
  void stopOnlineLoopWorker();
  void pollOnlineLoopResults(
      const BASIC::SE3& current_raw_pose, double timestamp);
  void advanceOnlineLoopCorrection(const BASIC::SE3& current_raw_pose);
  bool saveFrontendKeyFrameTrajectory() const;
  void saveLoopClosedMap();

  using StateFn = void (SuperLIO::*)();
  using OctVoxMapType = OctVoxMap<BASIC::V3, BASIC::scalar>;
  using KNNHeapType = KNNHeap<5, BASIC::V3>;
  struct LoopKeyFrame {
    BASIC::SE3 pose;
    BASIC::CloudPtr cloud_body;
    // Coarser bounded copy retained for online loop closure. Full save-time
    // keyframes may be persisted and released from RAM independently.
    BASIC::CloudPtr online_cloud_body;
    std::filesystem::path cloud_path;
    std::size_t point_count = 0;
    double timestamp = 0.0;
    std::uint64_t scan_index = 0;
    std::size_t effective_match_count = 0;
    Eigen::Vector3d lidar_rotation_information_eigenvalues =
        Eigen::Vector3d::Zero();
    Eigen::Vector3d lidar_translation_information_eigenvalues =
        Eigen::Vector3d::Zero();
    double lidar_yaw_information_ratio = 0.0;
    double lidar_conditional_yaw_information_ratio = 0.0;
    double lidar_translation_information_ratio = 0.0;
  };
  struct OnlineLoopFrame {
    BASIC::SE3 raw_pose;
    BASIC::CloudPtr cloud_body;
    double timestamp = 0.0;
  };
  struct OnlineLoopEdge {
    int from = -1;
    int to = -1;
    BASIC::SE3 measurement;
    float weight = 1.0f;
  };
  struct OnlineLoopPending {
    bool valid = false;
    int earlier = -1;
    int later = -1;
    BASIC::SE3 correction;
    int confirmations = 0;
  };
  struct OnlineLoopResult {
    bool accepted = false;
    int earlier = -1;
    int later = -1;
    BASIC::SE3 latest_correction;
    BASIC::VV3 raw_path;
    BASIC::VV3 corrected_path;
    float overlap = 0.0f;
    float rmse = 0.0f;
  };
  struct CloudWriteJob {
    std::filesystem::path path;
    BASIC::CloudPtr cloud;
    bool downsample = false;
    std::string context;
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
  std::uint64_t processed_scan_index_ = 0;
  BASIC::SE3 sys_init_pose_;
  BASIC::SE3 last_pose_;
  bool has_last_accepted_pose_ = false;
  SysState last_accepted_state_;
  ESKF::COV last_accepted_covariance_ = ESKF::COV::Identity();
  bool has_last_accepted_state_ = false;
  bool observation_valid_ = true;
  // IMU propagation result captured before scan-to-map correction.  The
  // relocation frontend uses its relative motion to maintain a continuous
  // odom frame while scan matching is allowed to correct map->odom.
  BASIC::SE3 latest_prediction_pose_;
  bool latest_prediction_pose_valid_ = false;
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
  std::uint64_t level_constraint_gated_count_ = 0;
  bool level_slope_protection_active_ = false;
  int level_slope_enter_count_ = 0;
  int level_slope_exit_count_ = 0;
  int level_slope_pending_invalid_count_ = 0;
  bool level_slope_recovery_active_ = false;
  int level_slope_recovery_count_ = 0;
  double level_slope_bounded_lease_path_m_ = 0.0;
  bool level_slope_bounded_lease_reentry_blocked_ = false;
  double level_slope_bounded_lease_reentry_path_m_ = 0.0;
  int level_slope_bounded_lease_reentry_flat_count_ = 0;
  std::uint64_t level_slope_bounded_lease_expired_count_ = 0;
  double level_slope_spatial_path_m_ = 0.0;
  double level_slope_spatial_supported_path_m_ = 0.0;
  double level_slope_spatial_observed_dz_m_ = 0.0;
  double level_slope_spatial_expected_dz_m_ = 0.0;
  int level_slope_spatial_mismatch_count_ = 0;
  bool level_slope_spatial_reentry_blocked_ = false;
  int level_slope_spatial_reentry_consistent_count_ = 0;
  double level_slope_spatial_last_observed_grade_deg_ = 0.0;
  double level_slope_spatial_last_expected_grade_deg_ = 0.0;
  double level_slope_spatial_last_error_deg_ = 0.0;
  double level_slope_spatial_last_support_ratio_ = 0.0;
  std::chrono::steady_clock::time_point last_level_constraint_log_time_{};
  struct GroundHeightContinuityReference {
    bool valid = false;
    BASIC::V3 normal_world = BASIC::V3::UnitZ();
    BASIC::V3 sensor_position_world = BASIC::V3::Zero();
    double ground_height_world = 0.0;
    std::uint64_t scan_index = 0;
  } ground_height_continuity_reference_;
  std::uint64_t ground_height_continuity_accepted_count_ = 0;
  std::uint64_t ground_height_continuity_rejected_count_ = 0;
  std::uint64_t ground_height_continuity_gated_count_ = 0;
  std::uint64_t ground_height_continuity_budget_gated_count_ = 0;
  double ground_height_continuity_applied_offset_m_ = 0.0;
  struct WallYawReferenceSample {
    double axis_rad = 0.0;
    BASIC::V3 position = BASIC::V3::Zero();
    std::uint64_t frame = 0;
  };
  struct WallYawReference {
    double axis_rad = 0.0;
    BASIC::V3 center = BASIC::V3::Zero();
    std::uint64_t created_frame = 0;
    std::uint64_t last_used_frame = 0;
    std::uint64_t accepted_count = 0;
    bool mature = false;
  };
  struct WallYawRecaptureState {
    int reference_index = -1;
    std::uint64_t last_frame = 0;
    std::deque<double> signed_innovations_rad;
  };
  std::deque<WallYawReferenceSample> wall_yaw_reference_samples_;
  std::vector<WallYawReference> wall_yaw_references_;
  WallYawRecaptureState wall_yaw_recapture_state_;
  bool wall_yaw_reference_capacity_warning_logged_ = false;
  std::uint64_t wall_yaw_constraint_accepted_count_ = 0;
  std::uint64_t wall_yaw_constraint_rejected_count_ = 0;
  std::uint64_t wall_yaw_constraint_gated_count_ = 0;
  std::uint64_t wall_yaw_extraction_skipped_count_ = 0;
  Eigen::Vector3d latest_lidar_rotation_information_eigenvalues_ =
      Eigen::Vector3d::Zero();
  Eigen::Vector3d latest_lidar_translation_information_eigenvalues_ =
      Eigen::Vector3d::Zero();
  double latest_lidar_yaw_information_ratio_ = 0.0;
  double latest_lidar_conditional_yaw_information_ratio_ = 0.0;
  double latest_lidar_translation_information_ratio_ = 0.0;

  std::size_t effect_knn_num_ = 0;
  BASIC::VV3 points_world_v3_, points_body_v3_;
  std::vector<uint8_t> effect_mask_;
  std::vector<uint8_t> effect_knn_mask_;
  std::vector<int> effect_knn_idxs_;
  std::vector<std::pair<BASIC::M6, BASIC::V6>> H_R_;
  std::vector<std::array<double, 4>> abcd_vec_;
  std::vector<LoopKeyFrame> loop_keyframes_;
  mutable std::mutex loop_keyframes_mutex_;
  BASIC::V3 last_loop_keyframe_pos_ = BASIC::V3(
      std::numeric_limits<BASIC::scalar>::quiet_NaN(),
      std::numeric_limits<BASIC::scalar>::quiet_NaN(),
      std::numeric_limits<BASIC::scalar>::quiet_NaN());
  int pcd_index_ = -1;
  int map_scans_since_fragment_ = 0;
  bool map_persistence_enabled_ = false;
  bool map_writer_stopping_ = false;
  bool map_writer_failed_ = false;
  bool map_writer_failure_logged_ = false;
  std::size_t map_write_jobs_outstanding_ = 0;
  std::filesystem::path map_work_dir_;
  std::filesystem::path map_fragment_dir_;
  std::filesystem::path loop_keyframe_dir_;
  std::vector<std::filesystem::path> map_fragment_paths_;
  std::deque<CloudWriteJob> map_write_queue_;
  std::vector<CloudWriteJob> failed_cloud_write_jobs_;
  std::size_t map_writer_queue_full_count_ = 0;
  std::size_t map_writer_max_outstanding_ = 0;
  std::thread map_writer_thread_;
  mutable std::mutex map_writer_mutex_;
  std::condition_variable map_writer_cv_;
  std::condition_variable map_writer_space_cv_;

  // One background worker is deliberately used on Jetson. A generous task
  // queue absorbs revisits, while the frontend only copies one coarse cloud
  // and enqueues an index.
  bool online_loop_stopping_ = false;
  std::thread online_loop_thread_;
  std::mutex online_loop_mutex_;
  std::condition_variable online_loop_cv_;
  std::deque<std::size_t> online_loop_tasks_;
  std::deque<OnlineLoopResult> online_loop_results_;
  std::vector<OnlineLoopEdge> online_loop_edges_;
  OnlineLoopPending online_loop_pending_;
  std::size_t online_loop_last_scheduled_index_ = 0;
  std::size_t online_loop_dropped_tasks_ = 0;
  std::size_t online_loop_accepted_count_ = 0;
  std::size_t online_loop_rejected_count_ = 0;
  BASIC::SE3 online_loop_map_to_odom_;
  BASIC::SE3 online_loop_target_map_to_odom_;

  Timer time_record_;
};

} // namespace END.

#endif
