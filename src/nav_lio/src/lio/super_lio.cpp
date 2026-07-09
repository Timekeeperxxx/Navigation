
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <limits>


using namespace BASIC;

namespace LI2Sup{

inline bool save_pcd_binary_safe(const std::string& filename, const PointCloudType& cloud)
{
  try {
    const auto parent = std::filesystem::path(filename).parent_path();
    if (!parent.empty()) {
      std::error_code ec;
      std::filesystem::create_directories(parent, ec);
      if (ec) {
        LOG(ERROR) << RED << " ---> 创建地图目录失败: "
                   << parent.string() << " error: " << ec.message() << RESET;
        return false;
      }
    }

    if (pcl::io::savePCDFileBinary(filename, cloud) != 0) {
      LOG(ERROR) << RED << " ---> 保存 PCD 文件失败: " << filename << RESET;
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    LOG(ERROR) << RED << " ---> 保存 PCD 文件失败: "
               << filename << " error: " << e.what() << RESET;
    return false;
  }
}

inline void normalize_cloud_layout(PointCloudType& cloud)
{
  if (!cloud.empty()) {
    cloud.width = cloud.size();
    cloud.height = 1;
    cloud.is_dense = false;
  }
}

inline void make_map_pcd_cloud(
  const PointCloudType::ConstPtr& source,
  PointCloudType& output,
  const float leaf_size,
  const char* context,
  const bool adapt_leaf_on_overflow = false)
{
  output.clear();
  if (!source || source->empty()) {
    return;
  }

  if (!g_if_filter) {
    output = *source;
    normalize_cloud_layout(output);
    return;
  }

  if (leaf_size <= 0.0f) {
    LOG(WARNING) << YELLOW << " ---> " << context
                 << " map downsample leaf size invalid, use accumulated map directly." << RESET;
    output = *source;
    normalize_cloud_layout(output);
    return;
  }

  bool has_finite_point = false;
  float min_x = 0.0f, min_y = 0.0f, min_z = 0.0f;
  float max_x = 0.0f, max_y = 0.0f, max_z = 0.0f;
  for (const auto& point : source->points) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
      continue;
    }
    if (!has_finite_point) {
      min_x = max_x = point.x;
      min_y = max_y = point.y;
      min_z = max_z = point.z;
      has_finite_point = true;
      continue;
    }
    min_x = std::min(min_x, point.x);
    min_y = std::min(min_y, point.y);
    min_z = std::min(min_z, point.z);
    max_x = std::max(max_x, point.x);
    max_y = std::max(max_y, point.y);
    max_z = std::max(max_z, point.z);
  }

  if (!has_finite_point) {
    return;
  }

  const auto voxel_count_x =
    static_cast<long double>(std::floor((max_x - min_x) / leaf_size) + 1.0);
  const auto voxel_count_y =
    static_cast<long double>(std::floor((max_y - min_y) / leaf_size) + 1.0);
  const auto voxel_count_z =
    static_cast<long double>(std::floor((max_z - min_z) / leaf_size) + 1.0);
  const auto max_index = static_cast<long double>(std::numeric_limits<int>::max());
  auto effective_leaf_size = leaf_size;

  if (voxel_count_x * voxel_count_y * voxel_count_z > max_index) {
    if (adapt_leaf_on_overflow) {
      const auto range_x = static_cast<long double>(std::max(max_x - min_x, leaf_size));
      const auto range_y = static_cast<long double>(std::max(max_y - min_y, leaf_size));
      const auto range_z = static_cast<long double>(std::max(max_z - min_z, leaf_size));
      const auto safe_leaf = std::cbrt((range_x * range_y * range_z) / max_index) * 1.1L;
      effective_leaf_size =
        static_cast<float>(std::max(static_cast<long double>(leaf_size), safe_leaf));
      LOG(WARNING) << YELLOW << " ---> " << context
                   << " map preview leaf size increased to avoid voxel index overflow."
                   << " requested_leaf_size: " << leaf_size
                   << " effective_leaf_size: " << effective_leaf_size
                   << " map_size: " << source->size() << RESET;
    } else {
      LOG(WARNING) << YELLOW << " ---> " << context
                   << " map downsample voxel index would overflow, use accumulated map directly."
                   << " leaf_size: " << leaf_size
                   << " map_size: " << source->size() << RESET;
      output = *source;
      normalize_cloud_layout(output);
      return;
    }
  }

  pcl::VoxelGrid<PointType> voxel_filter;
  voxel_filter.setInputCloud(source);
  voxel_filter.setLeafSize(effective_leaf_size, effective_leaf_size, effective_leaf_size);
  voxel_filter.filter(output);
  normalize_cloud_layout(output);
}

inline Eigen::Matrix4f se3_to_matrix4f(const SE3& pose)
{
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  matrix.block<3, 3>(0, 0) = pose.R_.cast<float>();
  matrix.block<3, 1>(0, 3) = pose.t_.cast<float>();
  return matrix;
}

inline Eigen::Matrix4f interpolate_correction(const Eigen::Matrix4f& correction, const float alpha)
{
  const float a = std::clamp(alpha, 0.0f, 1.0f);
  Eigen::Quaternionf q_target(correction.block<3, 3>(0, 0));
  q_target.normalize();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity().slerp(a, q_target);

  Eigen::Matrix4f output = Eigen::Matrix4f::Identity();
  output.block<3, 3>(0, 0) = q.toRotationMatrix();
  output.block<3, 1>(0, 3) = a * correction.block<3, 1>(0, 3);
  return output;
}

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


void SuperLIO::init(){
  ivox_.reset(new OctVoxMapType(OctVoxMapType::Options{g_ivox_resolution, g_ivox_capacity}));
  kf_.reset(new ESKF());
  data_wrapper_->setESKF(kf_);
  
  scan_undistort_full_.reset(new PointCloudType());
  ds_undistort_.reset(new PointCloudType());
  world_pc_.reset(new PointCloudType());
  ds_world_.reset(new PointCloudType());

  if(g_save_map){
    point_map_.reset(new PointCloudType());
  }
  
  points_world_v3_.reserve(21000);
  abcd_vec_.resize(20000);
  effect_knn_idxs_.resize(20000);
  effect_mask_.resize(20000, false);
  effect_knn_mask_.resize(20000, false);
  voxel_grid_fliter_.setLeafSize(g_voxel_fliter_size);

  state_fn_ = &SuperLIO::stateWaitKFInit;

  LOG(INFO) << GREEN << " ---> [SuperLIO]: initialized." << RESET;
}


void SuperLIO::stateWaitKFInit()
{
  if (kf_init()) {
    state_fn_ = &SuperLIO::stateWaitMapInit;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: KF init done" << RESET;
  }
}

void SuperLIO::stateWaitMapInit()
{
  if (map_init()) {
    kf_->init_ = true;
    state_fn_ = &SuperLIO::stateProcess;
    LOG(INFO) << GREEN << " ---> [SuperLIO]: Map init done" << RESET;
  }
}

void SuperLIO::process(){
  if(!data_wrapper_->sync_measure(measures_)){
    return;
  }
  (this->*state_fn_)();
}


bool SuperLIO::kf_init(){
  static int imu_cout = 0;
  static V3 mean_gyro = V3::Zero();
  static V3 mean_acce = V3::Zero();

  for(auto& imu: measures_.imu){
    imu_cout ++;
    mean_gyro += (imu.gyr - mean_gyro) / imu_cout;
    mean_acce += (imu.acc - mean_acce) / imu_cout;
  }

  /// 100 Hz for 1 second.
  if(imu_cout < 100){
    return false;
  }

  V3 gravity = - mean_acce * g_gravity_norm / mean_acce.norm();
  V3 ref_gravity(0, 0, - g_gravity_norm);
  M3 init_rot = Quat::FromTwoVectors(gravity, ref_gravity).toRotationMatrix();
  V3 n = init_rot.col(0);
  double yaw = atan2(n(1), n(0));

  M3 R_yaw_inv = Eigen::AngleAxis<scalar>(-yaw, V3::UnitZ()).toRotationMatrix(); 

  // init_rot represents the IMU orientation after gravity alignment (level orientation).
  // Perform LiDAR leveling correction, then transform the orientation into the robot frame.
  M3 rot = g_lidar_robo_yaw * R_yaw_inv * init_rot;  

  ESKF::Options options;
  options.gyro_var_ = g_imu_ng;
  options.acce_var_ = g_imu_na;
  options.bias_gyro_var_ = g_imu_nbg;
  options.bias_acce_var_ = g_imu_nba;
  options.num_iterations_ = g_kf_max_iterations;
  options.quit_eps_ = g_kf_quit_eps;
  options.estimate_gravity_ = g_kf_estimate_gravity;

  float imu_scale = g_gravity_norm / mean_acce.norm();
  kf_->SetInitialConditions(options, V3::Zero(), V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  return true;
}


bool SuperLIO::map_init(){
  frame_num_++;

  std::size_t ptsize = measures_.lidar.pc->size();
  points_world_v3_.resize(ptsize);

  const SE3 transform = sys_init_pose_ * g_lidar_imu;

  tbb::parallel_for(
    tbb::blocked_range<size_t>(0, ptsize),
    [&](const tbb::blocked_range<size_t>& r) {
      for (size_t idx = r.begin(); idx < r.end(); ++idx) {
        auto& point_pcl = measures_.lidar.pc->points[idx];
        V3 point_body(point_pcl.x, point_pcl.y, point_pcl.z);
        points_world_v3_[idx] = transform * point_body;
      }
    }
  );

  ivox_->insert(points_world_v3_);
  kf_->SetLastObsTime(measures_.lidar.end_time);

  // 20 Hz for 1.0 seconds. Integral coverage area > 70%
  if(frame_num_ > 3){
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    time_record_.Evaluate([this](){Propagation_Undistort();}, "[Undistort]");
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
  }else{
    Propagation_Undistort();
    DownSample();
    Observe();
    UpdateMap();
  }
  Output();
  caceData();
}


void SuperLIO::caceData(){
  if(!g_save_map) return;
  auto state = kf_->GetNavState();
  Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
  transformation.block<3, 3>(0, 0) = state.R.R_.cast<float>();
  transformation.block<3, 1>(0, 3) = state.p.cast<float>();

  if(g_if_filter){
    pcl::transformPointCloud(*ds_undistort_, *world_pc_, transformation);
  }else{
    pcl::transformPointCloud(*scan_undistort_full_, *world_pc_, transformation);
  }

  static int scan_wait_num = 0;
  if(!world_pc_->empty()){
    *point_map_ += *world_pc_;
    scan_wait_num++;
    maybeCacheLoopKeyFrame(SE3(state.R.R_, state.p), state.timestamp);

    static int accumulated_map_pub_count = 0;
    accumulated_map_pub_count++;
    if(accumulated_map_pub_count >= 10){
      CloudPtr map_preview(new PointCloudType());
      make_map_pcd_cloud(
        point_map_, *map_preview, g_map_preview_ds_size, "preview", true);
      if (!map_preview->empty()) {
        data_wrapper_->pub_map_accumulated(map_preview, state.timestamp);
      }
      accumulated_map_pub_count = 0;
    }
  }

  if(g_pcd_save_interval < 0) {
    scan_wait_num = 0;
    return;
  }

  static bool rm_PCD_dir = false;
  if(!rm_PCD_dir){
    rm_PCD_dir = true;
    std::string cmd = "rm -rf " + g_save_map_dir + "/PCD";
    [[maybe_unused]] int res;
    res = system(cmd.c_str());
    cmd = "mkdir -p " + g_save_map_dir + "/PCD";
    res = system(cmd.c_str());
  }

  if (point_map_->size() > 0 && scan_wait_num >= g_pcd_save_interval) {
    pcd_index_++;
    std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                               std::string(".pcd"));
    LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
    save_pcd_binary_safe(map_name, *point_map_);
    point_map_->clear();
    scan_wait_num = 0;
  }
}


void SuperLIO::ProcessCaceMap(){
  namespace fs = std::filesystem;

  std::string pcd_folder = g_save_map_dir + "/PCD";
  std::string output_map_name = g_save_map_dir + "/" + g_map_name;

  LOG(INFO) << YELLOW << " ---> Merging PCD fragments in: " << pcd_folder << RESET;

  PointCloudType::Ptr merged_map(new PointCloudType());

  int count = 0;
  for (const auto& entry : fs::directory_iterator(pcd_folder)) {
    if (entry.path().extension() == ".pcd" &&
      entry.path().filename().string().find("scans_") != std::string::npos) {
      PointCloudType::Ptr tmp_cloud(new PointCloudType());
      if (pcl::io::loadPCDFile<PointType>(entry.path().string(), *tmp_cloud) == 0) {
        *merged_map += *tmp_cloud;
        count++;
        // LOG(INFO) << GREEN << " ---> Merged: " << entry.path().filename().string() 
        //           << "   size: " << tmp_cloud->size() << RESET;
      } else {
        LOG(WARNING) << RED << " ---> Failed to load: " << entry.path().string() << RESET;
      }
    }
  }

  LOG(INFO) << YELLOW << " ---> Total merged fragments: " << count << RESET;

  PointCloudType filtered_map;

  if(g_if_filter){
    LOG(INFO) << YELLOW << " ---> Downsampling merged map before final save..." << RESET;
    make_map_pcd_cloud(merged_map, filtered_map, g_map_ds_size, "final merged");
  }else{
    LOG(INFO) << YELLOW << " ---> Not Downsampling merged map before final save..." << RESET;
    filtered_map = *merged_map;
  }
  
  normalize_cloud_layout(filtered_map);

  if (save_pcd_binary_safe(output_map_name, filtered_map)) {
    LOG(INFO) << GREEN << " ---> Final map saved to: " << output_map_name << RESET;
    LOG(INFO) << GREEN << " ---> Final map size: " << filtered_map.size() << RESET;
  }
}

void SuperLIO::maybeCacheLoopKeyFrame(const SE3& pose, double timestamp)
{
  if (!g_loop_closure_enable || !ds_undistort_ || ds_undistort_->empty()) {
    return;
  }

  const V3 current_pos = pose.t_;
  if (std::isfinite(last_loop_keyframe_pos_.x())) {
    const auto distance = (current_pos - last_loop_keyframe_pos_).norm();
    if (distance < g_loop_keyframe_min_distance) {
      return;
    }
  }

  LoopKeyFrame keyframe;
  keyframe.pose = pose;
  keyframe.timestamp = timestamp;
  keyframe.frame_id = frame_num_;
  keyframe.cloud_body.reset(new PointCloudType());
  *keyframe.cloud_body = *ds_undistort_;
  normalize_cloud_layout(*keyframe.cloud_body);
  loop_keyframes_.push_back(keyframe);
  last_loop_keyframe_pos_ = current_pos;
}

void SuperLIO::saveLoopClosedMap()
{
  if (!g_loop_closure_enable) {
    return;
  }

  if (loop_keyframes_.size() <= static_cast<std::size_t>(g_loop_keyframe_min_gap + 2)) {
    LOG(WARNING) << YELLOW << " ---> 闭环关键帧数量不足，跳过 map_loop.pcd 生成。"
                 << " keyframes: " << loop_keyframes_.size() << RESET;
    return;
  }

  const int end_idx = static_cast<int>(loop_keyframes_.size()) - 1;
  const auto& end_keyframe = loop_keyframes_[end_idx];
  int candidate_idx = -1;
  float best_distance = std::numeric_limits<float>::max();

  for (int i = 0; i <= end_idx - g_loop_keyframe_min_gap; ++i) {
    const float distance = (end_keyframe.pose.t_ - loop_keyframes_[i].pose.t_).norm();
    if (distance < g_loop_search_radius && distance < best_distance) {
      best_distance = distance;
      candidate_idx = i;
    }
  }

  if (candidate_idx < 0) {
    LOG(WARNING) << YELLOW << " ---> 未找到闭环候选关键帧，跳过 map_loop.pcd 生成。"
                 << " search_radius: " << g_loop_search_radius << RESET;
    return;
  }

  auto build_local_world_cloud = [&](int center_idx) {
    CloudPtr cloud(new PointCloudType());
    const int start_idx = std::max(0, center_idx - 2);
    const int stop_idx = std::min(end_idx, center_idx + 2);
    for (int i = start_idx; i <= stop_idx; ++i) {
      CloudPtr transformed(new PointCloudType());
      pcl::transformPointCloud(
          *loop_keyframes_[i].cloud_body,
          *transformed,
          se3_to_matrix4f(loop_keyframes_[i].pose));
      *cloud += *transformed;
    }
    normalize_cloud_layout(*cloud);
    CloudPtr filtered(new PointCloudType());
    make_map_pcd_cloud(cloud, *filtered, g_loop_map_ds_size, "loop local");
    return filtered;
  };

  CloudPtr source = build_local_world_cloud(end_idx);
  CloudPtr target = build_local_world_cloud(candidate_idx);
  if (source->empty() || target->empty()) {
    LOG(WARNING) << YELLOW << " ---> 闭环 ICP 输入点云为空，跳过 map_loop.pcd 生成。" << RESET;
    return;
  }

  pcl::IterativeClosestPoint<PointType, PointType> icp;
  icp.setInputSource(source);
  icp.setInputTarget(target);
  icp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
  icp.setMaximumIterations(80);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-5);

  PointCloudType aligned;
  icp.align(aligned);
  const double score = icp.getFitnessScore();
  if (!icp.hasConverged() || score > g_loop_icp_score_threshold) {
    LOG(WARNING) << YELLOW << " ---> 闭环 ICP 未通过，跳过 map_loop.pcd 生成。"
                 << " converged: " << icp.hasConverged()
                 << " score: " << score
                 << " threshold: " << g_loop_icp_score_threshold << RESET;
    return;
  }

  const Eigen::Matrix4f correction = icp.getFinalTransformation();
  LOG(INFO) << GREEN << " ---> 找到闭环候选。candidate_idx: " << candidate_idx
            << " end_idx: " << end_idx
            << " pose_distance: " << best_distance
            << " icp_score: " << score << RESET;

  CloudPtr loop_map(new PointCloudType());
  const float denom = static_cast<float>(std::max(1, end_idx - candidate_idx));
  for (int i = 0; i <= end_idx; ++i) {
    const float alpha = i <= candidate_idx ? 0.0f : static_cast<float>(i - candidate_idx) / denom;
    const Eigen::Matrix4f corrected_pose =
        interpolate_correction(correction, alpha) * se3_to_matrix4f(loop_keyframes_[i].pose);

    CloudPtr transformed(new PointCloudType());
    pcl::transformPointCloud(*loop_keyframes_[i].cloud_body, *transformed, corrected_pose);
    *loop_map += *transformed;
  }

  PointCloudType filtered_loop_map;
  make_map_pcd_cloud(loop_map, filtered_loop_map, g_loop_map_ds_size, "loop final");

  const std::string loop_map_path = g_save_map_dir + "/" + g_loop_map_name;
  if (save_pcd_binary_safe(loop_map_path, filtered_loop_map)) {
    LOG(INFO) << GREEN << " ---> 闭环地图已保存: " << loop_map_path << RESET;
    LOG(INFO) << GREEN << " ---> 闭环地图点数: " << filtered_loop_map.size() << RESET;
  }
}


void SuperLIO::saveMap(){
  if(!g_save_map) return;
  if(g_pcd_save_interval > 0){
    LOG(INFO) << YELLOW << " ---> Saving last cace ... " << RESET;
    if (point_map_->size() > 0) {
      pcd_index_++;
      std::string map_name(std::string(g_save_map_dir + "/PCD/scans_") + std::to_string(pcd_index_) +
                                 std::string(".pcd"));
      LOG(INFO) << GREEN << " ---> current scan saved to /PCD/scans_" << pcd_index_ << "  size:  " << point_map_->size() << RESET;
      save_pcd_binary_safe(map_name, *point_map_);
      point_map_->clear();
    }
    LOG(INFO) << GREEN << " ---> Save last cace success. " << RESET;
    LOG(INFO) << YELLOW << " ---> Process cace map ... " << RESET;
    ProcessCaceMap();
    LOG(INFO) << GREEN << " ---> Process cace map success. " << RESET;
    saveLoopClosedMap();
    return;
  }

  LOG(INFO) << YELLOW << " ---> Saving map..... " << RESET;
  if(!point_map_->empty()){
    std::string map_name = g_save_map_dir + "/" + g_map_name;
    LOG(INFO) << YELLOW << " ---> Save map to: " << map_name << RESET;
    PointCloudType latst_map;
    make_map_pcd_cloud(point_map_, latst_map, g_map_ds_size, "final");
    if (save_pcd_binary_safe(map_name, latst_map)) {
      LOG(INFO) << GREEN << " ---> Save map success. File: " << map_name << RESET;
      LOG(INFO) << GREEN << " ---> Map size: " << latst_map.size() << RESET;
    }
  }
  saveLoopClosedMap();
}


inline double get_cpu_time_seconds() {
  struct rusage usage;
  getrusage(RUSAGE_SELF, &usage);
  return usage.ru_utime.tv_sec + usage.ru_utime.tv_usec / 1e6 +
         usage.ru_stime.tv_sec + usage.ru_stime.tv_usec / 1e6;
}


void SuperLIO::Propagation_Undistort(){
  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;
  auto& raw_pc = measures_.lidar.pc;

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t, w_t;
    auto copy_raw_point = [&](const auto& pt, auto& pt_full) {
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = TLI_R * raw + TLI_t;
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    };
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;

      if (propagate_states_.size() < 2 ||
          query_time <= propagate_states_.front().time ||
          query_time > propagate_states_.back().time) {
        copy_raw_point(pt, pt_full);
        continue;
      }
      auto match_iter = propagate_states_.end();
      for (auto iter = propagate_states_.begin();
           std::next(iter) != propagate_states_.end(); ++iter) {
        auto next_iter = std::next(iter);
        if (iter->time < query_time && next_iter->time >= query_time) {
          match_iter = iter;
          break;
        }
      }
      if (match_iter == propagate_states_.end()) {
        copy_raw_point(pt, pt_full);
        continue;
      }
      auto match_iter_n = std::next(match_iter);
      double imu_dt = match_iter_n->time - match_iter->time;
      if (imu_dt <= 1e-9) {
        copy_raw_point(pt, pt_full);
        continue;
      }
      double query_dt = query_time - match_iter->time;
      double s = query_dt / imu_dt;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
      w_t = match_iter_n->w;
      M3 R_i = Quat(R_h).slerp(s, Quat(R_t)).toRotationMatrix();
      const double trans_dt = g_use_query_time_undistort ? query_dt : imu_dt;
      V3 t_ei(p_h + v_h * trans_dt + 0.5 * acc_t * trans_dt * trans_dt - T_end_t);
      V3 raw(pt.x, pt.y, pt.z);
      V3 eigen_point = R_inv * (R_i * (TLI_R * raw + TLI_t) + t_ei);
      pt_full.x = eigen_point[0];
      pt_full.y = eigen_point[1];
      pt_full.z = eigen_point[2];
    }
  });
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  size_t ptsize = ds_undistort_->size();
  
  static std::vector<float> _lengths;
  points_body_v3_.resize(ptsize);
  _lengths.resize(ptsize);
  if (effect_knn_idxs_.size() < ptsize) {
    effect_knn_idxs_.resize(ptsize);
  }
  if (abcd_vec_.size() < ptsize) {
    abcd_vec_.resize(ptsize);
  }
  if (effect_mask_.size() < ptsize) {
    effect_mask_.resize(ptsize, false);
  }
  if (effect_knn_mask_.size() < ptsize) {
    effect_knn_mask_.resize(ptsize, false);
  }

  effect_knn_num_ = ptsize;
  std::iota(effect_knn_idxs_.begin(), effect_knn_idxs_.begin() + ptsize, 0);

  for(size_t i = 0; i < ptsize; ++i){
    const auto& point_body_pcl = ds_undistort_->points[i];
    points_body_v3_[i] = V3(point_body_pcl.x, point_body_pcl.y, point_body_pcl.z);
    _lengths[i] = points_body_v3_[i].norm();
  }

  ivox_->reset_max_group();
  int iter_num = 0;

  kf_->UpdateObserve([&, this](const ESKF::KFState &kf_state, M6 &HTVH, V6 &HTVr) {
    const SE3 pose = kf_state.pose;
    const bool need_converge = kf_state.need_converge;
    const M3d R_transpose = (pose.R_.transpose()).cast<double>();

    tbb::enumerable_thread_specific<ThreadACC> tls_acc;

    tbb::parallel_for(
      tbb::blocked_range<size_t>(0, effect_knn_num_),
      [&](const tbb::blocked_range<size_t>& r) {
        KNNHeapType top_K;
        auto& local_acc = tls_acc.local();
        for (size_t r_s = r.begin(); r_s < r.end(); ++r_s) {
          int idx = effect_knn_idxs_[r_s];
          V3& point_body = points_body_v3_[idx];
          V3 point_world = pose * point_body;

          if(!need_converge){
            top_K.reset();
            ivox_->getTopK(point_world, top_K);
            if(top_K.count < 4){
              effect_mask_[idx] = false;
              effect_knn_mask_[idx] = false;
              continue;
            }
            effect_knn_mask_[idx] = true;
            effect_mask_[idx] = calc_plane_coeff(top_K.count, top_K.points_, abcd_vec_[idx]);
          }

          if(!effect_mask_[idx]) continue;

          auto& abcd = abcd_vec_[idx];
          scalar error;
          effect_mask_[idx] = compute_error(abcd, point_world, _lengths[idx], error);
          if(!effect_mask_[idx]) continue;
          
          {
            V3d normvec(abcd[0], abcd[1], abcd[2]);
            V3d nb = R_transpose * normvec;
            V3d point_body_d = point_body.cast<double>();
            V6d J;
            J.head<3>() = point_body_d.cross(nb);
            J.tail<3>() = normvec;
      
            local_acc.HTVH += J * 1000 * J.transpose();
            local_acc.HTVr -= J * 1000 * error;
          }
        }
    });

    M6d sum_HTVH = M6d::Zero();
    V6d sum_HTVr = V6d::Zero();
    for(const auto& local_acc : tls_acc){
      sum_HTVH += local_acc.HTVH;
      sum_HTVr += local_acc.HTVr;
    }
    HTVH = sum_HTVH.cast<scalar>();
    HTVr = sum_HTVr.cast<scalar>();

    if(need_converge) return;

    int _effect_knn_num = 0;
    for(size_t i = 0; i < effect_knn_num_; ++i){
      int idx = effect_knn_idxs_[i];
      if(!effect_knn_mask_[idx]) continue;
      effect_knn_idxs_[_effect_knn_num] = idx;
      _effect_knn_num++;
    }

    // LOG(INFO) << "effect_knn_num_: " << effect_knn_num_ << ", _effect_knn_num: " << _effect_knn_num;
    effect_knn_num_ = _effect_knn_num;

    iter_num++;
  });

  frame_num_++;
}

void SuperLIO::UpdateMap() {
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  points_world_v3_.resize(ptsize);
  
  const auto R = last_pose_.R_;
  const auto t = last_pose_.t_;
  
  for (size_t i = 0; i < ptsize; ++i) {
    const auto& pt = points_body_v3_[i];
    points_world_v3_[i] = R * pt + t;
  }
  
  ivox_->insert(points_world_v3_);

}


void SuperLIO::Output(){
  auto state = kf_->GetNavState();
  data_wrapper_->pub_odom(state);  

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

void SuperLIO::printTimeRecord(){
  if(!g_time_eva) return;
  time_record_.PrintAll();
}

} // namespace END.
