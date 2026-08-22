
#include "lio/super_lio_reloc.h"

#include <cmath>
#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/voxel_grid.h>


using namespace BASIC;

namespace LI2Sup{

namespace {

SE3 normalized_rigid_pose(const SE3& pose)
{
  Quat quaternion(pose.R_);
  if (!std::isfinite(quaternion.norm()) || quaternion.norm() < 1e-6) {
    return pose;
  }
  quaternion.normalize();
  return SE3(quaternion, pose.t_);
}

void normalize_cloud_layout(PointCloudType& cloud)
{
  cloud.width = static_cast<std::uint32_t>(cloud.size());
  cloud.height = 1;
  cloud.is_dense = false;
}

CloudPtr downsample_cloud(const CloudPtr& cloud, const float leaf_size)
{
  CloudPtr result(new PointCloudType());
  if (!cloud || cloud->empty()) {
    return result;
  }
  pcl::VoxelGrid<PointType> filter;
  filter.setLeafSize(leaf_size, leaf_size, leaf_size);
  filter.setInputCloud(cloud);
  filter.filter(*result);
  normalize_cloud_layout(*result);
  return result;
}

SE3 pose_from_matrix(const Eigen::Matrix4f& matrix)
{
  return normalized_rigid_pose(SE3(
      SO3(matrix.block<3, 3>(0, 0).cast<scalar>()),
      matrix.block<3, 1>(0, 3).cast<scalar>()));
}

}  // namespace

inline bool calc_plane_coeff(const int N, const std::array<V3, 5>& points, std::array<double, 4>& abcd)
{
  Eigen::Vector3d normvec;
  if (N == 5) {
    Eigen::Matrix<double, 5, 3> A;
    Eigen::Matrix<double, 5, 1> b;
    for (int j = 0; j < 5; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }
  else {
    Eigen::Matrix<double, 4, 3> A;
    Eigen::Matrix<double, 4, 1> b;

    for (int j = 0; j < N; j++) {
      A.row(j) = points[j].cast<double>();
      b(j) = -1.0;
    }
    normvec = A.colPivHouseholderQr().solve(b);
  }

  double n = normvec.norm();
  if (n < 1e-6f) return false;

  abcd[3] = 1.0 / n;
  normvec *= abcd[3];
  abcd[0] = normvec[0];
  abcd[1] = normvec[1];
  abcd[2] = normvec[2];
  
  for (int i = 0; i < N; ++i) {
    const V3& p = points[i];
    auto dist = abcd[0] * p(0) + abcd[1] * p(1) + abcd[2] * p(2) + abcd[3];
    if (std::abs(dist) > 0.1) return false;
  }
  return true;
}


inline bool compute_error(
  const std::array<double, 4>& abcd, const V3& point, 
  const float length, scalar& error)
{
  error = abcd[0] * point[0] + abcd[1] * point[1] + abcd[2] * point[2] + abcd[3];
  return length > 81 * error * error;
}


void SuperLIOReLoc::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());
  point_map_.reset(new PointCloudType());
  init_obs_data_.reset(new PointCloudType());
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;

  auto start_time = std::chrono::high_resolution_clock::now();
  SuperLIOReLoc::map_init();
  auto end_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
  LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init success. Time: " << duration.count() << " ms." << RESET;

  state_fn_ = &SuperLIOReLoc::stateWaitKFInit;
}


bool SuperLIOReLoc::map_init(){
  static bool pcd_loaded = false;
  if(pcd_loaded) return true;

  std::string map_name = g_save_map_dir + "/" + g_map_name;
  if(pcl::io::loadPCDFile<PointType>(map_name, *point_map_) == -1){
    LOG(ERROR) << RED << " ---> Load map failed. File: " << map_name << RESET;
    return false;
  }

  std::vector<int> useless_indices;
  pcl::removeNaNFromPointCloud(*point_map_, *point_map_, useless_indices);

  VV3 point_map_v3;
  point_map_v3.reserve(point_map_->size());
  for(const auto& point: *point_map_){
    V3 pt(point.x, point.y, point.z);
    point_map_v3.push_back(pt);
  }

  ivox_->insert(point_map_v3);

  fixed_map_tree_.reset(new pcl::KdTreeFLANN<PointType>());
  fixed_map_tree_->setInputCloud(point_map_);

  LOG(INFO) << GREEN << " ---> Load map success. File: " << map_name << RESET;
  LOG(INFO) << GREEN << " ---> Map size: " << point_map_->size() << RESET;
  ivox_->printInfo();

  pcd_loaded = true;

  data_wrapper_->set_global_map(point_map_);
  data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_);
  return true;
}


bool SuperLIOReLoc::kf_init(){
  const int need_init_frames = 10;
  static int imu_cout = 0;
  static int init_frame_count = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  LOG(INFO) << YELLOW << " ---> [DEBUG] kf_init() called. flg_get_init_guess_=" 
            << (flg_get_init_guess_ ? "true" : "false") 
            << " init_frame_count=" << init_frame_count
            << " imu_cout=" << imu_cout
            << RESET;

  CloudPtr point_cloud_pcl = CloudPtr(new PointCloudType());
  for(std::size_t i = 0; i < measures_.lidar.pc->size(); i++){
    auto p = measures_.lidar.pc->points[i];
    PointType point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    point.intensity = p.intensity;
    point_cloud_pcl->points.push_back(point);
  }

  if(init_frame_count < need_init_frames){
    *init_obs_data_ += *point_cloud_pcl;
  }
  init_frame_count++;

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  if(imu_cout < 20){
    return false;
  }

  if(init_frame_count < need_init_frames){
    return false;
  }

  // 如果 flg_get_init_guess_ 为 false，说明用户还没有通过话题发布初始位姿
  // 等待用户输入，不自动执行 NDT+ICP
  if(!flg_get_init_guess_){
    LOG(INFO) << YELLOW << " ---> Waiting for initial pose from topic (/initialpose)..." << RESET;
    return false;
  }

  LOG(INFO) << YELLOW << " ---> INIT start... obs_data size: " << init_obs_data_->size() << " target size: " << point_map_->size() << RESET;

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 
  M3 rot = R_yaw_inv * init_rot;

  M3 init_guess_R_ = re_init_pose_.R_ * rot;
  V3 init_guess_t_ = re_init_pose_.t_;
  M4 init_guess_T = M4::Identity();
  init_guess_T.block<3, 3>(0, 0) = init_guess_R_;
  init_guess_T.block<3, 1>(0, 3) = init_guess_t_;


  CloudPtr tmp_src_raw(new PointCloudType());
  pcl::transformPointCloud(
      *init_obs_data_, *tmp_src_raw,
      g_lidar_imu.matrix().cast<float>());
  // Initial alignment used to run NDT+ICP on every raw map/scan point and
  // block the processing thread for 8-10 seconds.  Live IMU buffering is
  // intentionally bounded, so that delay discarded the first navigation
  // segment.  A 0.5 m coarse alignment is sufficient here: the regular
  // fixed-map frontend and the multi-frame anchor immediately refine it.
  const float init_leaf = static_cast<float>(
      std::max(0.35, g_relocation_anchor_voxel_size));
  CloudPtr tmp_src = downsample_cloud(tmp_src_raw, init_leaf);
  CloudPtr init_target = downsample_cloud(point_map_, init_leaf);

  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
  ndt.setTransformationEpsilon(1e-4);
  ndt.setEuclideanFitnessEpsilon(1e-4);
  ndt.setMaximumIterations(25);
  ndt.setResolution(1.0);
  ndt.setInputTarget(init_target);

  pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;
  icp.setMaxCorrespondenceDistance(4.0);
  icp.setMaximumIterations(40);
  icp.setTransformationEpsilon(1e-4);
  icp.setEuclideanFitnessEpsilon(1e-4);
  icp.setRANSACIterations(0);
  icp.setInputTarget(init_target);

  ndt.setInputSource(tmp_src);
  icp.setInputSource(tmp_src);

  pcl::PointCloud<pcl::PointXYZI>::Ptr unused_result(new pcl::PointCloud<pcl::PointXYZI>());
  ndt.align(*unused_result, init_guess_T.matrix().cast<float>());
  icp.align(*unused_result, ndt.getFinalTransformation());

  if (icp.hasConverged() == false || icp.getFitnessScore() > 1.5)
  // if (icp.hasConverged() == false)
  {
    /// reset init state.
    imu_cout = 0;
    init_frame_count = 0;
    init_obs_data_->clear();
    mean_gyro = V3::Zero();
    mean_acce = V3::Zero();
    LOG(INFO) << RED << " ---> Global ICP Converged Fail! FitnessScore: " << icp.getFitnessScore() << RESET;
    return false;
  } else{
    const M4 icp_result = icp.getFinalTransformation().cast<scalar>();
    const V3 correction = icp_result.block<3, 1>(0, 3) - init_guess_T.block<3, 1>(0, 3);
    const M3 rotation_delta =
        init_guess_T.block<3, 3>(0, 0).transpose() * icp_result.block<3, 3>(0, 0);
    const double rotation_correction = Eigen::AngleAxis<scalar>(rotation_delta).angle();
    constexpr double max_xy_correction = 2.0;
    constexpr double max_z_correction = 0.75;
    constexpr double max_rotation_correction = M_PI / 3.0;
    const bool implausible_correction =
        correction.head<2>().norm() > max_xy_correction ||
        std::abs(correction(2)) > max_z_correction ||
        std::abs(rotation_correction) > max_rotation_correction;

    if (implausible_correction)
    {
      LOG(WARNING) << YELLOW
                   << " ---> Global ICP result rejected as an alias match. "
                   << "correction_xyz=" << correction.transpose()
                   << " rotation=" << rotation_correction
                   << " rad; keep the supplied initial pose."
                   << RESET;
    }
    else
    {
      init_guess_T = icp_result;
      LOG(INFO) << GREEN
                << " ---> Global ICP Converged Succeed! FitnessScore: "
                << icp.getFitnessScore() << RESET;
    }
  }

  LOG(INFO) << GREEN << "\n" << init_guess_T << RESET;

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;
  options.estimate_gravity_ = g_kf_estimate_gravity;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  /// The horizontal initial state of the imu in the robot coordinate system.
  state.R = SO3(init_guess_T.block<3, 3>(0, 0));
  state.p = init_guess_T.block<3, 1>(0, 3);
  state.timestamp = -1.0;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();

  // The fixed-map pose initializes map->odom.  odom itself starts at the
  // robot's localization origin and will subsequently integrate only the IMU
  // prediction's relative motion, never a scan-to-map correction jump.
  odom_pose_ = SE3();
  local_reference_map_pose_ = normalized_rigid_pose(sys_init_pose_);
  map_to_odom_ = local_reference_map_pose_;
  relocation_frames_initialized_ = true;
  localization_rejected_count_ = 0;

  {
    // Keep the immutable fixed map and its spatial index alive.  Navigation
    // uses them for periodic multi-frame verification; the normal per-scan
    // matcher continues to query ivox_.
    init_obs_data_->clear();
    init_obs_data_ = nullptr;
    data_wrapper_->set_initial_data(re_init_pose_, flg_get_init_guess_, true);
  }

  return true;
}


void SuperLIOReLoc::appendAnchorFrame()
{
  if (!g_relocation_anchor_enable || !ds_undistort_ ||
      ds_undistort_->empty() || !relocation_frames_initialized_) {
    return;
  }

  CloudPtr cloud_odom(new PointCloudType());
  pcl::transformPointCloud(
      *ds_undistort_, *cloud_odom,
      odom_pose_.matrix().cast<float>());
  normalize_cloud_layout(*cloud_odom);
  if (cloud_odom->empty()) {
    return;
  }

  AnchorFrame frame;
  frame.cloud_odom = std::move(cloud_odom);
  frame.odom_pose = odom_pose_;
  frame.timestamp = kf_->GetTime();
  anchor_frames_.push_back(std::move(frame));
  while (anchor_frames_.size() >
         static_cast<std::size_t>(g_relocation_anchor_window_frames)) {
    anchor_frames_.pop_front();
  }
  ++anchor_frame_counter_;
}


SuperLIOReLoc::AnchorAlignmentMetrics
SuperLIOReLoc::evaluateAnchorAlignment(
    const CloudPtr& source_odom,
    const CloudPtr& target_map,
    const Eigen::Matrix4f& map_from_odom,
    const double sensor_height_map) const
{
  AnchorAlignmentMetrics result;
  if (!source_odom || !target_map || source_odom->empty() ||
      target_map->empty() || !map_from_odom.allFinite()) {
    return result;
  }

  CloudPtr aligned(new PointCloudType());
  pcl::transformPointCloud(*source_odom, *aligned, map_from_odom);
  pcl::KdTreeFLANN<PointType> target_tree;
  target_tree.setInputCloud(target_map);
  std::vector<int> nearest_index(1);
  std::vector<float> nearest_squared_distance(1);
  const double maximum_squared_distance =
      g_relocation_anchor_verification_distance *
      g_relocation_anchor_verification_distance;
  double squared_error_sum = 0.0;
  int matched_points = 0;
  std::vector<Eigen::Vector2d> support_xy;
  support_xy.reserve(aligned->size());

  for (const auto& point : aligned->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) ||
        target_tree.nearestKSearch(
            point, 1, nearest_index, nearest_squared_distance) <= 0 ||
        nearest_squared_distance[0] > maximum_squared_distance) {
      continue;
    }
    ++matched_points;
    squared_error_sum += nearest_squared_distance[0];
    const auto& target_point =
        target_map->points[static_cast<std::size_t>(nearest_index[0])];
    support_xy.emplace_back(target_point.x, target_point.y);
    // A horizontal floor below the sensor can constrain Z but is ambiguous
    // in XY/yaw.  Require a separate population at or above the sensor's
    // lower body band before a six-DoF registration may anchor navigation.
    if (target_point.z >= sensor_height_map - 0.30) {
      ++result.structural_points;
    }
  }

  if (matched_points == 0 || support_xy.size() < 3) {
    return result;
  }
  result.overlap = static_cast<double>(matched_points) /
      static_cast<double>(source_odom->size());
  result.rmse = std::sqrt(
      squared_error_sum / static_cast<double>(matched_points));

  Eigen::Vector2d mean = Eigen::Vector2d::Zero();
  for (const auto& point : support_xy) {
    mean += point;
  }
  mean /= static_cast<double>(support_xy.size());
  Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
  for (const auto& point : support_xy) {
    const Eigen::Vector2d centered = point - mean;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<double>(support_xy.size());
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
  if (solver.info() != Eigen::Success) {
    return result;
  }
  const Eigen::Matrix2d axes = solver.eigenvectors();
  Eigen::Vector2d minimum = Eigen::Vector2d::Constant(
      std::numeric_limits<double>::infinity());
  Eigen::Vector2d maximum = Eigen::Vector2d::Constant(
      -std::numeric_limits<double>::infinity());
  for (const auto& point : support_xy) {
    const Eigen::Vector2d projection = axes.transpose() * (point - mean);
    minimum = minimum.cwiseMin(projection);
    maximum = maximum.cwiseMax(projection);
  }
  const Eigen::Vector2d extents = maximum - minimum;
  result.support_major = std::max(extents.x(), extents.y());
  result.support_minor = std::min(extents.x(), extents.y());
  result.valid =
      result.overlap >= g_relocation_anchor_min_overlap &&
      result.rmse <= g_relocation_anchor_max_rmse &&
      result.support_major >= g_relocation_anchor_min_support_major &&
      result.support_minor >= g_relocation_anchor_min_support_minor &&
      result.structural_points >=
          g_relocation_anchor_min_structural_points;
  return result;
}


SuperLIOReLoc::AnchorRegistrationResult
SuperLIOReLoc::computeFixedMapAnchor(
    const std::deque<AnchorFrame>& frames,
    const SE3& initial_map_from_odom,
    const SE3& current_map_pose) const
{
  AnchorRegistrationResult result;
  result.evaluated = true;
  result.reason = "unknown";

  CloudPtr early_raw(new PointCloudType());
  CloudPtr late_raw(new PointCloudType());
  const std::size_t split = frames.size() / 2;
  for (std::size_t i = 0; i < frames.size(); ++i) {
    if (i < split) {
      *early_raw += *frames[i].cloud_odom;
    } else {
      *late_raw += *frames[i].cloud_odom;
    }
  }
  normalize_cloud_layout(*early_raw);
  normalize_cloud_layout(*late_raw);
  const float leaf = static_cast<float>(g_relocation_anchor_voxel_size);
  CloudPtr early = downsample_cloud(early_raw, leaf);
  CloudPtr late = downsample_cloud(late_raw, leaf);
  CloudPtr source_raw(new PointCloudType());
  *source_raw += *early;
  *source_raw += *late;
  normalize_cloud_layout(*source_raw);
  CloudPtr source = downsample_cloud(source_raw, leaf);
  if (early->size() < 200 || late->size() < 200 ||
      source->size() < 500) {
    result.reason = "insufficient_source_points";
    return result;
  }

  PointType query;
  query.x = current_map_pose.t_.x();
  query.y = current_map_pose.t_.y();
  query.z = current_map_pose.t_.z();
  std::vector<int> map_indices;
  std::vector<float> map_squared_distances;
  fixed_map_tree_->radiusSearch(
      query, g_relocation_anchor_map_radius,
      map_indices, map_squared_distances);
  if (map_indices.size() < 1000) {
    result.reason = "insufficient_fixed_map_points";
    return result;
  }
  CloudPtr target_raw(new PointCloudType());
  target_raw->reserve(map_indices.size());
  for (const int index : map_indices) {
    if (index >= 0 &&
        static_cast<std::size_t>(index) < point_map_->size()) {
      target_raw->push_back(
          point_map_->points[static_cast<std::size_t>(index)]);
    }
  }
  normalize_cloud_layout(*target_raw);
  CloudPtr target = downsample_cloud(target_raw, leaf);
  if (target->size() < 500) {
    result.reason = "insufficient_fixed_map_points_after_filter";
    return result;
  }

  pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(
      g_relocation_anchor_max_correspondence_distance);
  icp.setMaximumIterations(40);
  icp.setTransformationEpsilon(1e-5);
  icp.setEuclideanFitnessEpsilon(1e-4);
  PointCloudType unused_aligned;
  const Eigen::Matrix4f initial =
      initial_map_from_odom.matrix().cast<float>();
  icp.align(unused_aligned, initial);
  const Eigen::Matrix4f estimate = icp.getFinalTransformation();
  if (!icp.hasConverged() || !estimate.allFinite()) {
    result.reason = "gicp_failed";
    return result;
  }

  result.full_metrics = evaluateAnchorAlignment(
      source, target, estimate, current_map_pose.t_.z());
  result.early_metrics = evaluateAnchorAlignment(
      early, target, estimate, current_map_pose.t_.z());
  result.late_metrics = evaluateAnchorAlignment(
      late, target, estimate, current_map_pose.t_.z());
  const bool temporal_support =
      result.early_metrics.overlap >= g_relocation_anchor_min_overlap &&
      result.late_metrics.overlap >= g_relocation_anchor_min_overlap &&
      result.early_metrics.rmse <= g_relocation_anchor_max_rmse &&
      result.late_metrics.rmse <= g_relocation_anchor_max_rmse;

  result.estimated_map_from_odom = pose_from_matrix(estimate);
  const SE3 desired_map_pose = normalized_rigid_pose(
      result.estimated_map_from_odom * frames.back().odom_pose);
  const M3 correction_rotation =
      desired_map_pose.R_ * current_map_pose.R_.transpose();
  const double signed_yaw = std::atan2(
      correction_rotation(1, 0), correction_rotation(0, 0));
  const M3 yaw_rotation = Eigen::AngleAxis<scalar>(
      static_cast<scalar>(signed_yaw), V3::UnitZ()).toRotationMatrix();
  const double tilt_deg = std::abs(
      Eigen::AngleAxis<scalar>(
          yaw_rotation.transpose() * correction_rotation).angle() *
      180.0 / M_PI);
  const double translation =
      (desired_map_pose.t_ - current_map_pose.t_).norm();
  const double yaw_deg = std::abs(signed_yaw * 180.0 / M_PI);
  result.requested_translation = translation;
  result.requested_yaw_deg = signed_yaw * 180.0 / M_PI;
  result.requested_tilt_deg = tilt_deg;
  const bool correction_safe =
      translation <= g_relocation_anchor_max_translation &&
      yaw_deg <= g_relocation_anchor_max_yaw_deg &&
      tilt_deg <= g_relocation_anchor_max_tilt_deg;

  if (!result.full_metrics.valid) {
    result.reason = "full_geometry_rejected";
    return result;
  }
  if (!temporal_support) {
    result.reason = "temporal_halves_disagree";
    return result;
  }
  if (!correction_safe) {
    result.reason = "correction_out_of_bounds";
    return result;
  }

  result.accepted = true;
  result.reason = "accepted";
  return result;
}


bool SuperLIOReLoc::applyFixedMapAnchor(
    const AnchorRegistrationResult& result)
{
  const auto reject_result = [&](const std::string& reason) {
    ++anchor_failure_count_;
    ++consecutive_anchor_failures_;
    if (consecutive_anchor_failures_ >=
        g_relocation_anchor_max_failures) {
      relocation_health_valid_ = false;
    }
    LOG(WARNING) << YELLOW
                 << " ---> [RelocationAnchor]: rejected. reason=" << reason
                 << " overlap=" << result.full_metrics.overlap
                 << " early=" << result.early_metrics.overlap
                 << " late=" << result.late_metrics.overlap
                 << " rmse=" << result.full_metrics.rmse
                 << " support=" << result.full_metrics.support_major
                 << "x" << result.full_metrics.support_minor
                 << " structural=" << result.full_metrics.structural_points
                 << " correction=" << result.requested_translation << "m/"
                 << result.requested_yaw_deg << "deg tilt="
                 << result.requested_tilt_deg
                 << "deg failures=" << consecutive_anchor_failures_
                 << RESET;
  };

  if (!result.evaluated || !result.accepted) {
    reject_result(result.reason);
    pending_anchor_estimate_valid_ = false;
    return false;
  }

  // A valid geometry score from one window is still not enough to move the
  // navigation frame.  Require the next independently accumulated window to
  // converge to the same absolute map<-odom transform.
  if (!pending_anchor_estimate_valid_) {
    pending_anchor_estimate_ = result.estimated_map_from_odom;
    pending_anchor_estimate_valid_ = true;
    LOG(INFO) << GREEN
              << " ---> [RelocationAnchor]: candidate pending temporal "
              << "confirmation. overlap=" << result.full_metrics.overlap
              << " rmse=" << result.full_metrics.rmse << RESET;
    return false;
  }
  const SE3 pending_map_pose = normalized_rigid_pose(
      pending_anchor_estimate_ * odom_pose_);
  const SE3 estimated_map_pose = normalized_rigid_pose(
      result.estimated_map_from_odom * odom_pose_);
  const double estimate_translation =
      (estimated_map_pose.t_ - pending_map_pose.t_).norm();
  const double estimate_rotation_deg = std::abs(
      std::atan2(
          (estimated_map_pose.R_ * pending_map_pose.R_.transpose())(1, 0),
          (estimated_map_pose.R_ * pending_map_pose.R_.transpose())(0, 0)) *
      180.0 / M_PI);
  pending_anchor_estimate_ = result.estimated_map_from_odom;
  if (estimate_translation > 0.25 || estimate_rotation_deg > 1.0) {
    reject_result("consecutive_estimates_disagree");
    return false;
  }

  const SE3 current_map_pose = normalized_rigid_pose(kf_->GetSE3());
  const SE3 desired_map_pose = normalized_rigid_pose(
      result.estimated_map_from_odom * odom_pose_);
  const M3 correction_rotation =
      desired_map_pose.R_ * current_map_pose.R_.transpose();
  const double signed_yaw = std::atan2(
      correction_rotation(1, 0), correction_rotation(0, 0));
  const M3 yaw_rotation = Eigen::AngleAxis<scalar>(
      static_cast<scalar>(signed_yaw), V3::UnitZ()).toRotationMatrix();
  const double tilt_deg = std::abs(
      Eigen::AngleAxis<scalar>(
          yaw_rotation.transpose() * correction_rotation).angle() *
      180.0 / M_PI);
  const V3 requested_translation =
      desired_map_pose.t_ - current_map_pose.t_;
  const double translation = requested_translation.norm();
  const double yaw_deg = std::abs(signed_yaw * 180.0 / M_PI);
  if (translation > g_relocation_anchor_max_translation ||
      yaw_deg > g_relocation_anchor_max_yaw_deg ||
      tilt_deg > g_relocation_anchor_max_tilt_deg) {
    reject_result("delayed_correction_out_of_bounds");
    return false;
  }

  const double translation_scale = translation > 1e-6
      ? std::min(
          1.0,
          g_relocation_anchor_max_translation_step / translation)
      : 1.0;
  const double applied_yaw = std::clamp(
      signed_yaw,
      -g_relocation_anchor_max_yaw_step_deg * M_PI / 180.0,
      g_relocation_anchor_max_yaw_step_deg * M_PI / 180.0);
  const M3 applied_rotation = Eigen::AngleAxis<scalar>(
      static_cast<scalar>(applied_yaw), V3::UnitZ()).toRotationMatrix();
  const V3 applied_translation =
      requested_translation * static_cast<scalar>(translation_scale);
  const bool correction_needed =
      translation >= 0.02 || yaw_deg >= 0.05;
  if (correction_needed) {
    SysState corrected_state = kf_->GetSysState();
    corrected_state.R = SO3(applied_rotation * corrected_state.R.R_);
    // Translate at the robot anchor rather than rotating its world position
    // around the map origin.  The latter creates an artificial lever arm on
    // long routes and makes the same yaw correction look larger with range.
    corrected_state.p += applied_translation;
    corrected_state.v = applied_rotation * corrected_state.v;
    kf_->SetX(corrected_state);
  }
  const SE3 corrected_map_pose = normalized_rigid_pose(kf_->GetSE3());
  map_to_odom_ = normalized_rigid_pose(
      corrected_map_pose * odom_pose_.inverse());
  local_reference_map_pose_ = corrected_map_pose;
  consecutive_anchor_failures_ = 0;
  relocation_health_valid_ = true;
  ++anchor_success_count_;
  observation_valid_ = true;

  LOG(INFO) << GREEN
            << " ---> [RelocationAnchor]: accepted. overlap="
            << result.full_metrics.overlap
            << " early=" << result.early_metrics.overlap
            << " late=" << result.late_metrics.overlap
            << " rmse=" << result.full_metrics.rmse
            << " support=" << result.full_metrics.support_major
            << "x" << result.full_metrics.support_minor
            << " structural=" << result.full_metrics.structural_points
            << " requested=" << translation << "m/"
            << signed_yaw * 180.0 / M_PI << "deg"
            << " applied="
            << (correction_needed ? applied_translation.norm() : 0.0)
            << "m/"
            << (correction_needed
                    ? applied_yaw * 180.0 / M_PI
                    : 0.0)
            << "deg"
            << " successes=" << anchor_success_count_ << RESET;
  return true;
}


bool SuperLIOReLoc::tryFixedMapAnchor()
{
  bool correction_applied = false;
  if (anchor_worker_running_ &&
      anchor_future_.wait_for(std::chrono::seconds(0)) ==
          std::future_status::ready) {
    AnchorRegistrationResult result = anchor_future_.get();
    anchor_worker_running_ = false;
    correction_applied = applyFixedMapAnchor(result);
  }

  if (!g_relocation_anchor_enable || anchor_worker_running_ ||
      !fixed_map_tree_ || !point_map_ || point_map_->empty() ||
      anchor_frames_.size() <
          static_cast<std::size_t>(g_relocation_anchor_min_frames) ||
      anchor_frame_counter_ %
          static_cast<std::uint64_t>(
              g_relocation_anchor_interval_frames) != 0) {
    return correction_applied;
  }

  double motion_path = 0.0;
  double rotation_path_deg = 0.0;
  for (std::size_t i = 1; i < anchor_frames_.size(); ++i) {
    const SE3 relative = anchor_frames_[i - 1].odom_pose.inverse() *
        anchor_frames_[i].odom_pose;
    motion_path += relative.t_.norm();
    rotation_path_deg += std::abs(
        Eigen::AngleAxis<scalar>(relative.R_).angle() * 180.0 / M_PI);
  }
  if (motion_path < g_relocation_anchor_min_motion &&
      rotation_path_deg < 15.0) {
    return correction_applied;
  }

  const std::deque<AnchorFrame> frames = anchor_frames_;
  const SE3 initial_map_from_odom = map_to_odom_;
  const SE3 current_map_pose = normalized_rigid_pose(
      map_to_odom_ * odom_pose_);
  anchor_future_ = std::async(
      std::launch::async,
      [this, frames, initial_map_from_odom, current_map_pose]() {
        return computeFixedMapAnchor(
            frames, initial_map_from_odom, current_map_pose);
      });
  anchor_worker_running_ = true;
  return correction_applied;
}


void SuperLIOReLoc::UpdateMap() {
  // Observe() can roll an aliased scan match back to the IMU prediction (or
  // to the last accepted state).  The relocation override used to ignore
  // that decision and, unlike SuperLIO::UpdateMap(), never committed a last
  // accepted pose.  Consequently the frame-motion guard stayed disabled for
  // the whole localization run and rejected scans could still be written
  // into the reference map.  During an in-place turn this lets a repetitive
  // wall/corridor pull the map and pose together by metres.
  SE3 final_map_pose = normalized_rigid_pose(kf_->GetSE3());
  bool local_motion_available = observation_valid_;
  if (!relocation_frames_initialized_) {
    odom_pose_ = SE3();
    local_reference_map_pose_ = final_map_pose;
    map_to_odom_ = final_map_pose;
    relocation_frames_initialized_ = true;
  } else {
    // Observe() starts from latest_prediction_pose_ and may then either accept
    // a map correction, roll back to that prediction, or reject an unsafe
    // prediction and restore the last accepted map state.  Only the first two
    // cases contain a usable local motion increment for odom.
    bool prediction_used = observation_valid_;
    if (!observation_valid_ && latest_prediction_pose_valid_) {
      const SE3 normalized_prediction =
          normalized_rigid_pose(latest_prediction_pose_);
      const SE3 prediction_to_final =
          normalized_prediction.inverse() * final_map_pose;
      const double final_translation_error =
          prediction_to_final.t_.norm();
      const double final_rotation_error_deg = std::abs(
          Eigen::AngleAxis<scalar>(prediction_to_final.R_).angle() *
          180.0 / M_PI);
      prediction_used =
          final_translation_error <= 1e-4 &&
          final_rotation_error_deg <= 1e-3;
    }

    bool local_increment_applied = false;
    if (prediction_used && latest_prediction_pose_valid_) {
      const SE3 normalized_prediction =
          normalized_rigid_pose(latest_prediction_pose_);
      const SE3 local_increment = normalized_rigid_pose(
          local_reference_map_pose_.inverse() * normalized_prediction);
      const double increment_translation = local_increment.t_.norm();
      const double increment_rotation_deg = std::abs(
          Eigen::AngleAxis<scalar>(local_increment.R_).angle() *
          180.0 / M_PI);
      const bool increment_valid =
          local_increment.t_.allFinite() &&
          local_increment.R_.allFinite() &&
          increment_translation <= g_max_frame_translation &&
          increment_rotation_deg <= g_max_frame_rotation_deg;
      if (increment_valid) {
        odom_pose_ = normalized_rigid_pose(odom_pose_ * local_increment);
        local_increment_applied = true;
        local_motion_available = true;
      } else {
        LOG(WARNING) << YELLOW
                     << " ---> [SuperLIO]: reject unsafe local odom increment. "
                     << "translation=" << increment_translation
                     << "m rotation=" << increment_rotation_deg
                     << "deg map_observation_valid=" << observation_valid_
                     << RESET;
      }
    }
    // The next propagation starts from the state left by Observe(), whether
    // it was a corrected map pose, an IMU prediction, or a safety rollback.
    local_reference_map_pose_ = final_map_pose;

    if (!observation_valid_ &&
        (!prediction_used || !local_increment_applied)) {
      // A safety rollback may reset the map-state estimator to an older
      // accepted pose.  Never undo that distance in continuous odom; rebase
      // only map->odom so the TF chain still resolves exactly to the state
      // used for map-frame clouds and legacy /lio/odom consumers.
      map_to_odom_ = normalized_rigid_pose(
          final_map_pose * odom_pose_.inverse());
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: rebase map->odom after localization "
                   << "prediction rollback; keep odom->base_link continuous."
                   << RESET;
    }
  }

  if (observation_valid_) {
    const SE3 measured_map_to_odom = normalized_rigid_pose(
        final_map_pose * odom_pose_.inverse());
    const SE3 previous_map_pose = map_to_odom_ * odom_pose_;
    const double correction_translation =
        (final_map_pose.t_ - previous_map_pose.t_).norm();
    const double correction_rotation_deg = std::abs(
        Eigen::AngleAxis<scalar>(
            previous_map_pose.R_.transpose() * final_map_pose.R_).angle() *
        180.0 / M_PI);
    map_to_odom_ = measured_map_to_odom;
    if (correction_translation > 0.25 || correction_rotation_deg > 1.0) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: map localization correction isolated "
                   << "in map->odom. translation=" << correction_translation
                   << "m rotation=" << correction_rotation_deg
                   << "deg; odom->base_link remains continuous."
                   << RESET;
    }
  }

  // The multi-frame anchor consumes scans in continuous odom coordinates.
  // It is also allowed to recover a frame rejected by the single-scan
  // matcher, provided the IMU/local increment was usable.
  if (local_motion_available) {
    appendAnchorFrame();
  }
  const bool anchor_accepted = tryFixedMapAnchor();
  if (anchor_accepted) {
    final_map_pose = normalized_rigid_pose(kf_->GetSE3());
  }

  if (!observation_valid_ && !anchor_accepted) {
    ++localization_rejected_count_;
    if (localization_rejected_count_ == 1 ||
        localization_rejected_count_ % 10 == 0) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: freeze map->odom on rejected "
                   << "localization frame. consecutive="
                   << localization_rejected_count_ << RESET;
    }
    return;
  }

  if (localization_rejected_count_ > 0) {
    LOG(INFO) << GREEN
              << " ---> [SuperLIO]: map localization recovered after "
              << localization_rejected_count_ << " rejected frames."
              << RESET;
    localization_rejected_count_ = 0;
  }

  // State acceptance is independent from map mutation.  A localization map
  // is normally read-only, but the next observation still needs the accepted
  // pose/state as its consistency reference and rollback anchor.
  last_pose_ = kf_->GetSE3();
  has_last_accepted_pose_ = true;
  last_accepted_state_ = kf_->GetSysState();
  last_accepted_covariance_ = kf_->GetCov();
  has_last_accepted_state_ = true;

  if (!g_update_map)
    return;

  static int update_delay = 100;
  if (update_delay > 0) {
    --update_delay;
    return;
  }

  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0)
    return;

  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);
}


void SuperLIOReLoc::Output() {
  auto state = kf_->GetNavState();
  if (relocation_frames_initialized_) {
    data_wrapper_->pub_relocation_odom(
        state,
        odom_pose_,
        map_to_odom_,
        observation_valid_ && relocation_health_valid_);
  } else {
    data_wrapper_->pub_odom(state);
  }

  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  CloudPtr world_pc(new PointCloudType());
  
  if(g_visual_map){
    static int count = -1;
    count++;
    if(count % g_pub_step != 0){
      return;
    }
    count = 0;
    if(g_visual_dense){
      pcl::transformPointCloud(*scan_undistort_full_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }else{
      pcl::transformPointCloud(*ds_undistort_, *world_pc, transformation);
      data_wrapper_->pub_cloud_world(world_pc, state.timestamp);
    }
  }

}



} // namespace END.
