
#include "lio/super_lio.h"

#include <sys/resource.h>
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/concurrent_vector.h>
#include <tbb/enumerable_thread_specific.h>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <optional>
#include <random>
#include <sstream>

#include <Eigen/SparseCholesky>

using namespace BASIC;

namespace LI2Sup{

inline bool save_pcd_binary_safe(const std::string& filename, const PointCloudType& cloud)
{
  const auto temporary_filename = filename + ".tmp";
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

    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    ec.clear();
    if (pcl::io::savePCDFileBinary(temporary_filename, cloud) != 0) {
      LOG(ERROR) << RED << " ---> 保存 PCD 临时文件失败: " << temporary_filename << RESET;
      return false;
    }

    std::filesystem::rename(temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 提交 PCD 文件失败: "
                 << filename << " error: " << ec.message() << RESET;
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
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

inline SE3 project_gravity_aligned_loop_correction(
    const Eigen::Matrix4f& full_correction,
    const V3& raw_later_position)
{
  // LiDAR-inertial odometry already observes roll and pitch from gravity.
  // Applying a small GICP roll/pitch correction about the world origin can
  // change Z by metres at a distant ramp and flatten real terrain.  Keep the
  // full GICP transform for acceptance validation, then project the graph
  // measurement to the standard gravity-aligned 4-DoF form: XYZ + yaw.
  //
  // Translation is recomputed about the later keyframe origin, so the
  // projected transform sends that origin to exactly the same XYZ selected
  // by full 6-DoF GICP.  Z remains fully optimized; only correction roll and
  // pitch are removed.
  const Eigen::Matrix3f full_rotation =
      full_correction.block<3, 3>(0, 0);
  const float yaw =
      std::atan2(full_rotation(1, 0), full_rotation(0, 0));
  const Eigen::Matrix3f yaw_rotation =
      Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
          .toRotationMatrix();
  const Eigen::Vector3f anchor =
      raw_later_position.cast<float>();
  const Eigen::Vector3f corrected_anchor =
      full_rotation * anchor +
      full_correction.block<3, 1>(0, 3);

  Eigen::Matrix4f projected =
      Eigen::Matrix4f::Identity();
  projected.block<3, 3>(0, 0) = yaw_rotation;
  projected.block<3, 1>(0, 3) =
      corrected_anchor - yaw_rotation * anchor;
  return SE3(projected);
}

inline Eigen::Matrix3f interpolate_correction_rotation(
    const Eigen::Matrix4f& correction,
    const float alpha)
{
  const float a = std::clamp(alpha, 0.0f, 1.0f);
  Eigen::Quaternionf q_target(correction.block<3, 3>(0, 0));
  q_target.normalize();
  Eigen::Quaternionf q = Eigen::Quaternionf::Identity().slerp(a, q_target);
  return q.toRotationMatrix();
}

struct PoseGraphEdge
{
  int from = -1;
  int to = -1;
  SE3 measurement;
  float weight = 1.0f;
  bool loop = false;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using PoseVector = std::vector<SE3, Eigen::aligned_allocator<SE3>>;
using PoseGraphEdgeVector =
  std::vector<PoseGraphEdge, Eigen::aligned_allocator<PoseGraphEdge>>;

inline V6 pose_graph_residual(
    const SE3& from,
    const SE3& to,
    const SE3& measurement)
{
  return (measurement.inverse() * from.inverse() * to).log_vee();
}

struct LoopConsistency
{
  float path_length = 0.0f;
  float translation_residual = 0.0f;
  float rotation_residual_deg = 0.0f;
  float weight = 0.0f;
};

inline LoopConsistency evaluate_loop_consistency(
    const PoseGraphEdge& edge,
    const PoseVector& reference_poses,
    const float path_length)
{
  LoopConsistency result;
  result.path_length = std::max(path_length, 1.0e-3f);
  const V6 residual = pose_graph_residual(
      reference_poses[edge.from],
      reference_poses[edge.to],
      edge.measurement);
  // BASIC::SE3::log_vee() stores rotation first and translation second.
  result.translation_residual = residual.tail<3>().norm();
  result.rotation_residual_deg =
      residual.head<3>().norm() * 180.0f / static_cast<float>(M_PI);

  // This is a trajectory uncertainty model, not an XYZ/Z distance gate.
  // Expected uncertainty grows continuously with travelled path length and
  // all translation axes are treated together.  A real ramp or floor change
  // therefore remains untouched; only a loop observation that demands an
  // implausibly different correction from the current graph is switched off.
  const float translation_scale =
      g_loop_translation_drift_ratio * result.path_length;
  const float rotation_scale =
      g_loop_rotation_drift_deg_per_m * result.path_length;
  const float normalized_translation =
      result.translation_residual /
      std::max(translation_scale, 1.0e-6f);
  const float normalized_rotation =
      result.rotation_residual_deg /
      std::max(rotation_scale, 1.0e-6f);
  result.weight = 1.0f /
      (1.0f + normalized_translation * normalized_translation +
       normalized_rotation * normalized_rotation);
  return result;
}

inline float pose_graph_robust_cost(
    const PoseVector& poses,
    const PoseGraphEdgeVector& edges)
{
  float cost = 0.0f;
  for (const auto& edge : edges) {
    V6 residual = pose_graph_residual(
        poses[edge.from], poses[edge.to], edge.measurement);
    residual.head<3>() *= 2.0f;
    residual *= edge.weight;
    const float norm = residual.norm();
    if (edge.loop && norm > 1.0f) {
      cost += 2.0f * norm - 1.0f;
    } else {
      cost += residual.squaredNorm();
    }
  }
  return cost;
}

inline bool optimize_pose_graph(
    PoseVector& poses,
    const PoseGraphEdgeVector& edges,
    const int max_iterations = 8)
{
  if (poses.size() < 2 || edges.empty()) {
    return false;
  }

  const int variable_count =
      static_cast<int>((poses.size() - 1) * 6);
  // Loop corrections are small rotations (validated before reaching here),
  // so solve the correction graph in se(3):
  //
  //   x_to - x_from = log(G_from_to)
  //
  // Identity measurements on consecutive nodes form a Laplacian smoothness
  // term.  Loop measurements jointly deform the whole trajectory instead of
  // applying a rigid jump at either endpoint.  Solving in double precision is
  // important for a chain with thousands of nodes; the previous float
  // absolute-pose Gauss-Newton system was ill-conditioned.
  std::vector<Eigen::Matrix<double, 6, 1>> measurements;
  measurements.reserve(edges.size());
  for (const auto& edge : edges) {
    measurements.push_back(
        edge.measurement.log_vee().cast<double>());
  }

  Eigen::VectorXd solution =
      Eigen::VectorXd::Zero(variable_count);
  for (int iteration = 0; iteration < max_iterations; ++iteration) {
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(edges.size() * 24 + variable_count);
    Eigen::VectorXd rhs =
        Eigen::VectorXd::Zero(variable_count);

    auto correction_component = [&](
        const int node, const int axis) -> double {
      if (node <= 0) {
        return 0.0;
      }
      return solution((node - 1) * 6 + axis);
    };

    for (std::size_t edge_index = 0;
         edge_index < edges.size();
         ++edge_index) {
      const auto& edge = edges[edge_index];
      const auto& measurement = measurements[edge_index];
      Eigen::Matrix<double, 6, 1> residual;
      for (int axis = 0; axis < 6; ++axis) {
        residual(axis) =
            correction_component(edge.to, axis) -
            correction_component(edge.from, axis) -
            measurement(axis);
      }

      Eigen::Matrix<double, 6, 1> scaled_residual =
          edge.weight * residual;
      scaled_residual.head<3>() *= 2.0;
      double robust_weight = 1.0;
      const double residual_norm = scaled_residual.norm();
      if (edge.loop && residual_norm > 1.0) {
        robust_weight = 1.0 / residual_norm;
      }

      for (int axis = 0; axis < 6; ++axis) {
        const double axis_scale =
            static_cast<double>(edge.weight) *
            (axis < 3 ? 2.0 : 1.0);
        const double information =
            robust_weight * axis_scale * axis_scale;
        const int from_offset =
            edge.from > 0 ? (edge.from - 1) * 6 + axis : -1;
        const int to_offset =
            edge.to > 0 ? (edge.to - 1) * 6 + axis : -1;

        if (from_offset >= 0) {
          triplets.emplace_back(
              from_offset, from_offset, information);
          rhs(from_offset) -= information * measurement(axis);
        }
        if (to_offset >= 0) {
          triplets.emplace_back(
              to_offset, to_offset, information);
          rhs(to_offset) += information * measurement(axis);
        }
        if (from_offset >= 0 && to_offset >= 0) {
          triplets.emplace_back(
              from_offset, to_offset, -information);
          triplets.emplace_back(
              to_offset, from_offset, -information);
        }
      }
    }

    for (int diagonal = 0;
         diagonal < variable_count;
         ++diagonal) {
      triplets.emplace_back(diagonal, diagonal, 1.0e-10);
    }
    Eigen::SparseMatrix<double> hessian(
        variable_count, variable_count);
    hessian.setFromTriplets(
        triplets.begin(), triplets.end());
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
    solver.compute(hessian);
    if (solver.info() != Eigen::Success) {
      return false;
    }
    const Eigen::VectorXd next_solution =
        solver.solve(rhs);
    if (solver.info() != Eigen::Success ||
        !next_solution.allFinite()) {
      return false;
    }
    const double update =
        (next_solution - solution).cwiseAbs().maxCoeff();
    solution = next_solution;
    if (update < 1.0e-7) {
      break;
    }
  }

  poses[0] = SE3();
  for (std::size_t i = 1; i < poses.size(); ++i) {
    const V6 correction =
        solution.segment<6>((i - 1) * 6).cast<float>();
    poses[i] = SE3(correction);
  }
  return true;
}

struct TimedPoseCorrection
{
  double timestamp = 0.0;
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using TimedPoseCorrectionVector = std::vector<
  TimedPoseCorrection,
  Eigen::aligned_allocator<TimedPoseCorrection>>;

inline bool save_loop_trajectory_safe(
    const std::string& filename,
    const TimedPoseCorrectionVector& corrections)
{
  const auto temporary_filename = filename + ".tmp";
  try {
    std::ofstream output(
        temporary_filename,
        std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      return false;
    }
    output << "NAV_LIO_POSE_GRAPH_V1\n";
    output << corrections.size() << "\n";
    output << std::setprecision(17);
    for (const auto& sample : corrections) {
      output << sample.timestamp;
      output << std::setprecision(9);
      for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
          output << " " << sample.correction(row, col);
        }
      }
      output << "\n" << std::setprecision(17);
    }
    output.flush();
    if (!output.good()) {
      output.close();
      std::filesystem::remove(temporary_filename);
      return false;
    }
    output.close();
    std::error_code ec;
    std::filesystem::rename(
        temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      return false;
    }
    return true;
  } catch (...) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    return false;
  }
}

inline bool save_loop_correction_safe(
    const std::string& filename,
    const double candidate_timestamp,
    const double deformation_start_timestamp,
    const double end_timestamp,
    const Eigen::Matrix4f& correction)
{
  const auto temporary_filename = filename + ".tmp";
  try {
    std::ofstream output(temporary_filename, std::ios::out | std::ios::trunc);
    if (!output.is_open()) {
      LOG(ERROR) << RED << " ---> 无法创建闭环校正元数据: "
                 << temporary_filename << RESET;
      return false;
    }

    output << "NAV_LIO_LOOP_CORRECTION_V2\n";
    output << std::setprecision(17)
           << candidate_timestamp << " "
           << deformation_start_timestamp << " "
           << end_timestamp << "\n";
    output << std::setprecision(9);
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (row != 0 || col != 0) {
          output << " ";
        }
        output << correction(row, col);
      }
    }
    output << "\n";
    output.flush();
    if (!output.good()) {
      output.close();
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 写入闭环校正元数据失败: "
                 << temporary_filename << RESET;
      return false;
    }
    output.close();

    std::error_code ec;
    std::filesystem::rename(temporary_filename, filename, ec);
    if (ec) {
      std::filesystem::remove(temporary_filename);
      LOG(ERROR) << RED << " ---> 提交闭环校正元数据失败: "
                 << filename << " error: " << ec.message() << RESET;
      return false;
    }
    return true;
  } catch (const std::exception& e) {
    std::error_code ec;
    std::filesystem::remove(temporary_filename, ec);
    LOG(ERROR) << RED << " ---> 保存闭环校正元数据失败: "
               << filename << " error: " << e.what() << RESET;
    return false;
  }
}

inline double wrap_period_signed(const double angle, const double period)
{
  return angle - period * std::floor(angle / period + 0.5);
}

inline double mean_manhattan_axis(const std::deque<double>& angles)
{
  double cosine_sum = 0.0;
  double sine_sum = 0.0;
  for (const double angle : angles) {
    cosine_sum += std::cos(4.0 * angle);
    sine_sum += std::sin(4.0 * angle);
  }
  return 0.25 * std::atan2(sine_sum, cosine_sum);
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
    map_preview_.reset(new PointCloudType());
    map_preview_pending_.reset(new PointCloudType());
    map_preview_effective_leaf_size_ = std::max(g_map_preview_ds_size, 0.01f);
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
  data_wrapper_->finish_measure();
}


bool SuperLIO::kf_init(){
  for(auto& imu: measures_.imu){
    imu_init_window_.push_back(imu);
    while (imu_init_window_.size() > static_cast<std::size_t>(g_imu_init_samples)) {
      imu_init_window_.pop_front();
    }
  }

  if (imu_init_window_.size() < static_cast<std::size_t>(g_imu_init_samples)) {
    return false;
  }

  V3 mean_gyro = V3::Zero();
  V3 mean_acce = V3::Zero();
  for (const auto& imu : imu_init_window_) {
    mean_gyro += imu.gyr;
    mean_acce += imu.acc;
  }
  const scalar sample_count = static_cast<scalar>(imu_init_window_.size());
  mean_gyro /= sample_count;
  mean_acce /= sample_count;

  V3 gyro_variance = V3::Zero();
  V3 accel_variance = V3::Zero();
  for (const auto& imu : imu_init_window_) {
    gyro_variance += (imu.gyr - mean_gyro).cwiseAbs2();
    accel_variance += (imu.acc - mean_acce).cwiseAbs2();
  }
  const scalar variance_denominator = std::max<scalar>(sample_count - 1.0, 1.0);
  const V3 gyro_stddev = (gyro_variance / variance_denominator).cwiseSqrt();
  const V3 accel_stddev = (accel_variance / variance_denominator).cwiseSqrt();
  const double accel_mean_norm = mean_acce.norm();
  const double accel_stddev_ratio =
      accel_mean_norm > 1e-6 ? accel_stddev.maxCoeff() / accel_mean_norm
                             : std::numeric_limits<double>::infinity();
  const bool stationary =
      mean_gyro.norm() <= g_imu_init_max_gyro_norm &&
      gyro_stddev.maxCoeff() <= g_imu_init_max_gyro_stddev &&
      accel_stddev_ratio <= g_imu_init_max_accel_stddev_ratio;
  if (!stationary) {
    const auto now = std::chrono::steady_clock::now();
    if (last_imu_motion_warning_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_imu_motion_warning_time_ > std::chrono::seconds(1)) {
      last_imu_motion_warning_time_ = now;
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: waiting for stationary IMU initialization. "
                   << "gyro_mean_norm=" << mean_gyro.norm()
                   << " gyro_stddev=" << gyro_stddev.transpose()
                   << " accel_stddev_ratio=" << accel_stddev_ratio << RESET;
    }
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
  kf_->SetInitialConditions(options, mean_gyro, V3::Zero(), imu_scale, ref_gravity);
  auto state = kf_->GetSysState();
  state.R = SO3(rot);
  state.p = g_odom_robo.t_;        // By default, the robot frame is used as the reference origin.
  state.timestamp = measures_.imu.back().secs;
  kf_->SetX(state);
  sys_init_pose_ = kf_->GetSE3();
  imu_reference_accel_norm_ = mean_acce.norm();
  gravity_direction_window_.clear();
  gravity_reference_world_ = V3::UnitZ();
  gravity_reference_valid_ = false;
  last_gravity_sample_time_ = -1.0;
  wall_yaw_reference_samples_.clear();
  wall_yaw_references_.clear();
  wall_yaw_constraint_accepted_count_ = 0;
  wall_yaw_constraint_rejected_count_ = 0;
  wall_yaw_constraint_gated_count_ = 0;
  wall_yaw_extraction_skipped_count_ = 0;
  LOG(INFO) << GREEN << " ---> [SuperLIO]: IMU initialized with "
            << imu_init_window_.size() << " stationary samples, gyro_bias="
            << mean_gyro.transpose() << " gyro_stddev=" << gyro_stddev.transpose()
            << " accel_mean=" << mean_acce.transpose()
            << " accel_stddev_ratio=" << accel_stddev_ratio << RESET;
  imu_init_window_.clear();
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
    last_pose_ = kf_->GetSE3();
    has_last_accepted_pose_ = true;
    last_accepted_state_ = kf_->GetSysState();
    last_accepted_covariance_ = kf_->GetCov();
    has_last_accepted_state_ = true;
    g_flg_map_init = false;
    return true;
  }
  return false;
}


void SuperLIO::stateProcess(){
  frame_num_++;
  if(g_time_eva){
    bool undistortion_valid = false;
    time_record_.Evaluate(
      [this, &undistortion_valid](){
        undistortion_valid = Propagation_Undistort();
      }, "[Undistort]");
    if (!undistortion_valid) {
      observation_valid_ = false;
      return;
    }
    time_record_.Evaluate([this]() { DownSample(); }, "[DownSample]");
    time_record_.Evaluate([this]() { Observe(); }, "[Observe]");
    time_record_.Evaluate([this]() { UpdateMap(); }, "[UpdateMap]");
    time_record_.Evaluate([this]() { Output(); }, "[Output]");
    time_record_.Evaluate([this]() { caceData(); }, "[CacheData]");
  }else{
    if (!Propagation_Undistort()) {
      observation_valid_ = false;
      return;
    }
    DownSample();
    Observe();
    UpdateMap();
    Output();
    caceData();
  }
}


void SuperLIO::caceData(){
  if(!g_save_map || !observation_valid_) return;
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

    updateMapPreview(state.timestamp);
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


void SuperLIO::updateMapPreview(const double timestamp){
  if (!map_preview_ || !map_preview_pending_ || !world_pc_ || world_pc_->empty()) {
    return;
  }

  *map_preview_pending_ += *world_pc_;
  map_preview_scan_count_++;
  if (map_preview_scan_count_ < g_map_preview_publish_interval) {
    return;
  }
  map_preview_scan_count_ = 0;

  CloudPtr merged(new PointCloudType());
  merged->reserve(map_preview_->size() + map_preview_pending_->size());
  *merged += *map_preview_;
  *merged += *map_preview_pending_;
  map_preview_pending_->clear();

  CloudPtr filtered(new PointCloudType());
  auto leaf_size = std::max(map_preview_effective_leaf_size_, 0.01f);
  const auto max_points = static_cast<std::size_t>(g_map_preview_max_points);

  if (g_if_filter) {
    for (int attempt = 0; attempt < 6; ++attempt) {
      make_map_pcd_cloud(merged, *filtered, leaf_size, "bounded preview", true);
      if (filtered->size() <= max_points) {
        break;
      }
      const double ratio = static_cast<double>(filtered->size()) /
                           static_cast<double>(max_points);
      leaf_size *= static_cast<float>(std::max(1.25, std::sqrt(ratio) * 1.05));
    }
  } else {
    *filtered = *merged;
  }

  if (filtered->size() > max_points) {
    CloudPtr limited(new PointCloudType());
    limited->reserve(max_points);
    const std::size_t stride =
      std::max<std::size_t>(1, (filtered->size() + max_points - 1) / max_points);
    for (std::size_t index = 0;
         index < filtered->size() && limited->size() < max_points;
         index += stride) {
      limited->push_back(filtered->points[index]);
    }
    normalize_cloud_layout(*limited);
    filtered = limited;
  }

  if (leaf_size > map_preview_effective_leaf_size_ + 1e-6f) {
    LOG(INFO) << GREEN << " ---> [SuperLIO]: bounded map preview leaf increased from "
              << map_preview_effective_leaf_size_ << " to " << leaf_size
              << ", points=" << filtered->size() << RESET;
  }
  map_preview_effective_leaf_size_ = leaf_size;
  map_preview_ = filtered;

  if (!map_preview_->empty()) {
    data_wrapper_->pub_map_accumulated(map_preview_, timestamp);
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

  const std::string loop_map_path = g_save_map_dir + "/" + g_loop_map_name;
  const std::string correction_path =
      g_save_map_dir + "/loop_correction.txt";
  const std::string trajectory_path =
      g_save_map_dir + "/loop_pose_graph.txt";
  const auto finalize_started = std::chrono::steady_clock::now();
  const auto finalize_deadline = finalize_started +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          std::chrono::duration<double>(g_loop_max_finalize_seconds));
  auto finalize_timed_out = [&]() {
    if (std::chrono::steady_clock::now() <= finalize_deadline) {
      return false;
    }
    LOG(WARNING) << YELLOW
                 << " ---> 闭环后端超过保存时间预算，保留已原子提交的原始 map.pcd。"
                 << " budget_seconds: " << g_loop_max_finalize_seconds
                 << RESET;
    return true;
  };
  {
    // A repeated save into the same scene must never reuse an older accepted
    // closure when the current run cannot validate one.
    std::error_code ec;
    std::filesystem::remove(loop_map_path, ec);
    ec.clear();
    std::filesystem::remove(correction_path, ec);
    ec.clear();
    std::filesystem::remove(trajectory_path, ec);
  }

  if (loop_keyframes_.size() <= static_cast<std::size_t>(g_loop_keyframe_min_gap + 2)) {
    LOG(WARNING) << YELLOW << " ---> 闭环关键帧数量不足，跳过 map_loop.pcd 生成。"
                 << " keyframes: " << loop_keyframes_.size() << RESET;
    return;
  }

  const int end_idx = static_cast<int>(loop_keyframes_.size()) - 1;
  const auto& end_keyframe = loop_keyframes_[end_idx];
  PoseVector raw_poses;
  raw_poses.reserve(loop_keyframes_.size());
  std::vector<float> cumulative_route_length(
      loop_keyframes_.size(), 0.0f);
  for (std::size_t i = 0; i < loop_keyframes_.size(); ++i) {
    raw_poses.push_back(loop_keyframes_[i].pose);
    if (i > 0) {
      cumulative_route_length[i] =
          cumulative_route_length[i - 1] +
          (raw_poses[i].t_ - raw_poses[i - 1].t_).norm();
    }
  }
  const float raw_route_length = cumulative_route_length.back();
  struct LoopCandidate {
    int index = -1;
    float horizontal_distance = std::numeric_limits<float>::max();
  };
  std::vector<LoopCandidate> candidates;

  for (int i = 0; i <= end_idx - g_loop_keyframe_min_gap; ++i) {
    const V3 delta = end_keyframe.pose.t_ - loop_keyframes_[i].pose.t_;
    const float horizontal_distance =
        static_cast<float>(std::hypot(delta.x(), delta.y()));
    if (horizontal_distance < g_loop_search_radius) {
      candidates.push_back({i, horizontal_distance});
    }
  }

  if (candidates.empty()) {
    LOG(INFO) << GREEN
              << " ---> 终点未进入任何历史关键帧的保守搜索半径；"
              << "不强制回起点，继续检查中途回环。"
              << " horizontal_search_radius: "
              << g_loop_search_radius << RESET;
  }

  // Always retain a few of the earliest candidates (the common "return to
  // start" workflow), then fill the remaining budget by horizontal proximity.
  // Full 3-D geometry, including Z, is validated by ICP below.
  std::vector<LoopCandidate> selected_candidates;
  const int early_candidate_count =
      std::min<int>(5, static_cast<int>(candidates.size()));
  selected_candidates.insert(
      selected_candidates.end(),
      candidates.begin(),
      candidates.begin() + early_candidate_count);
  std::sort(
      candidates.begin(), candidates.end(),
      [](const LoopCandidate& lhs, const LoopCandidate& rhs) {
        return lhs.horizontal_distance < rhs.horizontal_distance;
      });
  for (const auto& candidate : candidates) {
    const auto already_selected = std::find_if(
        selected_candidates.begin(), selected_candidates.end(),
        [&](const LoopCandidate& selected) {
          return selected.index == candidate.index;
        });
    if (already_selected == selected_candidates.end()) {
      selected_candidates.push_back(candidate);
    }
    if (static_cast<int>(selected_candidates.size()) >= g_loop_candidate_limit) {
      break;
    }
  }

  auto build_local_world_cloud = [&](int center_idx) {
    CloudPtr cloud(new PointCloudType());
    const int start_idx =
        std::max(0, center_idx - g_loop_local_window_size);
    const int stop_idx =
        std::min(end_idx, center_idx + g_loop_local_window_size);
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
  if (source->empty()) {
    LOG(WARNING) << YELLOW << " ---> 闭环 ICP 源点云为空，跳过 map_loop.pcd 生成。" << RESET;
    return;
  }

  struct EndpointHypothesis
  {
    int index = -1;
    float horizontal_distance = std::numeric_limits<float>::max();
    double score = std::numeric_limits<double>::max();
    float overlap_ratio = 0.0f;
    float rotation_deg = std::numeric_limits<float>::max();
    Eigen::Matrix4f full_correction = Eigen::Matrix4f::Identity();
    PoseGraphEdge edge;
    LoopConsistency consistency;
    double selection_cost = std::numeric_limits<double>::max();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
  using EndpointHypothesisVector = std::vector<
      EndpointHypothesis,
      Eigen::aligned_allocator<EndpointHypothesis>>;
  EndpointHypothesisVector endpoint_hypotheses;
  for (const auto& candidate : selected_candidates) {
    if (finalize_timed_out()) {
      return;
    }
    CloudPtr target = build_local_world_cloud(candidate.index);
    if (target->empty()) {
      continue;
    }

    // The odometry already expresses both scans in one world frame. Seed ICP
    // with the full XYZ origin offset so several metres of accumulated drift
    // can be recovered. This is an initial guess, not a Z lock: ICP still
    // estimates and validates all six degrees of freedom from the point clouds.
    double candidate_best_score = std::numeric_limits<double>::max();
    float candidate_best_overlap_ratio = 0.0f;
    float candidate_best_rotation_deg = std::numeric_limits<float>::max();
    float candidate_best_seed_yaw_deg = 0.0f;
    Eigen::Matrix4f candidate_best_correction = Eigen::Matrix4f::Identity();
    bool candidate_converged = false;

    // A long route may accumulate enough yaw drift that one ICP basin is not
    // sufficient. Search a bounded set of yaw seeds; roll, pitch, Z and the
    // final yaw remain free variables estimated by full 3-D ICP.
    const std::array<int, 7> seed_yaw_degrees{
        0, -15, 15, -30, 30, -45, 45};
    for (const int seed_yaw_deg : seed_yaw_degrees) {
      if (finalize_timed_out()) {
        return;
      }
      const float seed_yaw_rad =
          static_cast<float>(seed_yaw_deg) *
          static_cast<float>(M_PI) / 180.0f;
      Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
      initial_guess.block<3, 3>(0, 0) =
          Eigen::AngleAxisf(seed_yaw_rad, Eigen::Vector3f::UnitZ())
              .toRotationMatrix();
      initial_guess.block<3, 1>(0, 3) =
          loop_keyframes_[candidate.index].pose.t_.cast<float>() -
          initial_guess.block<3, 3>(0, 0) *
              end_keyframe.pose.t_.cast<float>();

      pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
      icp.setInputSource(source);
      icp.setInputTarget(target);
      icp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
      icp.setMaximumIterations(80);
      icp.setTransformationEpsilon(1e-6);
      icp.setEuclideanFitnessEpsilon(1e-5);

      PointCloudType aligned;
      icp.align(aligned, initial_guess);
      const Eigen::Matrix4f attempt_correction =
          icp.getFinalTransformation();
      if (!icp.hasConverged() || !attempt_correction.allFinite() ||
          aligned.empty()) {
        continue;
      }

      pcl::search::KdTree<PointType> target_tree;
      target_tree.setInputCloud(target);
      std::vector<int> nearest_index(1);
      std::vector<float> nearest_squared_distance(1);
      std::size_t overlap_count = 0;
      double squared_distance_sum = 0.0;
      const float max_squared_distance =
          g_loop_icp_max_distance * g_loop_icp_max_distance;
      for (const auto& point : aligned.points) {
        if (target_tree.nearestKSearch(
                point, 1, nearest_index,
                nearest_squared_distance) > 0 &&
            nearest_squared_distance[0] <= max_squared_distance) {
          ++overlap_count;
          squared_distance_sum += nearest_squared_distance[0];
        }
      }
      if (overlap_count == 0) {
        continue;
      }
      const double score =
          squared_distance_sum / static_cast<double>(overlap_count);
      const float overlap_ratio =
          static_cast<float>(overlap_count) /
          static_cast<float>(aligned.size());

      const Eigen::AngleAxisf rotation(
          attempt_correction.block<3, 3>(0, 0));
      const float rotation_deg =
          std::abs(rotation.angle()) * 180.0f /
          static_cast<float>(M_PI);
      if (score < candidate_best_score) {
        candidate_converged = true;
        candidate_best_score = score;
        candidate_best_overlap_ratio = overlap_ratio;
        candidate_best_rotation_deg = rotation_deg;
        candidate_best_seed_yaw_deg =
            static_cast<float>(seed_yaw_deg);
        candidate_best_correction = attempt_correction;
      }

      // Start with the odometry-aligned seed and avoid needless registration
      // work once an attempt already satisfies every acceptance gate.
      if (score <= g_loop_icp_score_threshold &&
          overlap_ratio >= g_loop_min_overlap_ratio &&
          rotation_deg <= g_loop_max_correction_rotation_deg) {
        break;
      }
    }

    LOG(INFO) << GREEN
              << " ---> 闭环候选配准结果。candidate_idx: "
              << candidate.index
              << " horizontal_distance: "
              << candidate.horizontal_distance
              << " converged: " << candidate_converged
              << " best_score: " << candidate_best_score
              << " overlap_ratio: " << candidate_best_overlap_ratio
              << " rotation_deg: " << candidate_best_rotation_deg
              << " seed_yaw_deg: " << candidate_best_seed_yaw_deg
              << RESET;

    if (!candidate_converged ||
        candidate_best_score > g_loop_icp_score_threshold ||
        candidate_best_overlap_ratio < g_loop_min_overlap_ratio ||
        candidate_best_rotation_deg >
            g_loop_max_correction_rotation_deg) {
      continue;
    }

    EndpointHypothesis hypothesis;
    hypothesis.index = candidate.index;
    hypothesis.horizontal_distance = candidate.horizontal_distance;
    hypothesis.score = candidate_best_score;
    hypothesis.overlap_ratio = candidate_best_overlap_ratio;
    hypothesis.rotation_deg = candidate_best_rotation_deg;
    hypothesis.full_correction = candidate_best_correction;
    hypothesis.edge.from = candidate.index;
    hypothesis.edge.to = end_idx;
    hypothesis.edge.measurement =
        project_gravity_aligned_loop_correction(
            candidate_best_correction, raw_poses[end_idx].t_);
    hypothesis.edge.weight = 2.0f;
    hypothesis.edge.loop = true;
    PoseVector identity_reference(
        loop_keyframes_.size(), SE3());
    const float segment_path_length =
        cumulative_route_length[end_idx] -
        cumulative_route_length[candidate.index];
    hypothesis.consistency = evaluate_loop_consistency(
        hypothesis.edge, identity_reference, segment_path_length);
    hypothesis.selection_cost =
        hypothesis.score +
        (1.0 - hypothesis.consistency.weight) +
        0.1 * (1.0 - hypothesis.overlap_ratio);
    endpoint_hypotheses.push_back(hypothesis);
  }

  // The final keyframe has no special semantic relationship with the first
  // keyframe. Treat each spatially distinct historical revisit as a normal
  // graph hypothesis. Temporally overlapping windows are one observation;
  // retain their earliest member to keep the selected edge stable.
  std::sort(
      endpoint_hypotheses.begin(), endpoint_hypotheses.end(),
      [](const EndpointHypothesis& lhs, const EndpointHypothesis& rhs) {
        return lhs.index < rhs.index;
      });
  EndpointHypothesisVector distinct_endpoint_hypotheses;
  for (const auto& hypothesis : endpoint_hypotheses) {
    if (distinct_endpoint_hypotheses.empty() ||
        hypothesis.index - distinct_endpoint_hypotheses.back().index >
            2 * g_loop_local_window_size) {
      distinct_endpoint_hypotheses.push_back(hypothesis);
    }
  }

  int candidate_idx = -1;
  float best_horizontal_distance = std::numeric_limits<float>::max();
  double best_score = std::numeric_limits<double>::max();
  double best_selection_cost = std::numeric_limits<double>::max();
  float best_consistency_weight = 0.0f;
  Eigen::Matrix4f correction = Eigen::Matrix4f::Identity();
  std::optional<PoseGraphEdge> endpoint_edge;
  for (const auto& hypothesis : distinct_endpoint_hypotheses) {
    const bool graph_consistent =
        hypothesis.consistency.weight >= g_loop_min_consistency_weight;
    LOG(INFO) << GREEN
              << " ---> 终点普通回环假设。candidate_idx: "
              << hypothesis.index
              << " path_length: " << hypothesis.consistency.path_length
              << " translation_residual: "
              << hypothesis.consistency.translation_residual
              << " rotation_residual_deg: "
              << hypothesis.consistency.rotation_residual_deg
              << " consistency_weight: "
              << hypothesis.consistency.weight
              << " selection_cost: " << hypothesis.selection_cost
              << " graph_consistent: " << graph_consistent << RESET;
    if (!graph_consistent) {
      continue;
    }
    if (candidate_idx < 0 ||
        (g_loop_prefer_earliest_candidate &&
         hypothesis.index < candidate_idx) ||
        (!g_loop_prefer_earliest_candidate &&
         hypothesis.selection_cost < best_selection_cost)) {
      candidate_idx = hypothesis.index;
      best_horizontal_distance = hypothesis.horizontal_distance;
      best_score = hypothesis.score;
      best_selection_cost = hypothesis.selection_cost;
      best_consistency_weight = hypothesis.consistency.weight;
      correction = hypothesis.full_correction;
      endpoint_edge = hypothesis.edge;
    }
  }

  if (candidate_idx < 0) {
    LOG(INFO) << GREEN << " ---> 终点回环未通过，继续检查中途回环。"
                 << " candidates_tested: " << selected_candidates.size()
                 << " threshold: " << g_loop_icp_score_threshold << RESET;
  } else {
    LOG(INFO) << GREEN << " ---> 找到终点回环候选。candidate_idx: "
              << candidate_idx
              << " end_idx: " << end_idx
              << " horizontal_pose_distance: " << best_horizontal_distance
              << " correction_translation: "
              << correction.block<3, 1>(0, 3).transpose()
              << " icp_score: " << best_score
              << " consistency_weight: " << best_consistency_weight
              << " selection_cost: " << best_selection_cost << RESET;
  }
  // Optimize a deformation graph of world-frame correction poses C_i rather
  // than the large absolute poses X_i directly:
  //
  //   X_i = C_i * T_i(raw)
  //
  // Consecutive C_i are connected by identity edges, so the correction is
  // spread smoothly over the route.  If G maps a later raw world cloud onto
  // an earlier raw world cloud, a loop edge measures C_earlier^-1*C_later=G.
  // This is the same global pose-graph idea, but avoids numerical
  // cancellation on 100 m absolute coordinates and makes the intended
  // deformation explicit.
  PoseVector correction_poses(
      loop_keyframes_.size(), SE3());
  PoseGraphEdgeVector pose_graph_edges;
  pose_graph_edges.reserve(loop_keyframes_.size() + 32);
  for (int i = 0; i < end_idx; ++i) {
    PoseGraphEdge edge;
    edge.from = i;
    edge.to = i + 1;
    edge.measurement = SE3();
    edge.weight = 1.0f;
    edge.loop = false;
    pose_graph_edges.push_back(edge);
  }

  if (endpoint_edge.has_value()) {
    pose_graph_edges.push_back(*endpoint_edge);
  }

  struct RevisitCandidate
  {
    int earlier = -1;
    int later = -1;
    float horizontal_distance =
        std::numeric_limits<float>::max();
  };
  std::vector<RevisitCandidate> revisit_candidates;
  // Candidate discovery grows continuously with route length instead of
  // switching at a hard map-size boundary. Final acceptance is still decided
  // by full 3-D registration and graph consistency below.
  const float internal_search_radius =
      std::min(
          g_loop_search_radius,
          g_loop_internal_search_radius +
              g_loop_translation_drift_ratio * raw_route_length);
  LOG(INFO) << GREEN
            << " ---> 中途回环搜索范围。raw_route_length: "
            << raw_route_length
            << " internal_search_radius: "
            << internal_search_radius << RESET;
  for (int later = g_loop_keyframe_min_gap;
       later < end_idx - g_loop_local_window_size;
       ++later) {
    int nearest_earlier = -1;
    float nearest_distance =
        std::numeric_limits<float>::max();
    const int latest_earlier =
        later - g_loop_keyframe_min_gap;
    for (int earlier = 0; earlier <= latest_earlier; ++earlier) {
      const V3 delta =
          loop_keyframes_[later].pose.t_ -
          loop_keyframes_[earlier].pose.t_;
      const float distance =
          static_cast<float>(std::hypot(delta.x(), delta.y()));
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest_earlier = earlier;
      }
    }
    if (nearest_earlier >= 0 &&
        nearest_distance < internal_search_radius) {
      revisit_candidates.push_back(
          {nearest_earlier, later, nearest_distance});
    }
  }
  std::sort(
      revisit_candidates.begin(),
      revisit_candidates.end(),
      [](const RevisitCandidate& lhs, const RevisitCandidate& rhs) {
        return lhs.horizontal_distance < rhs.horizontal_distance;
      });
  std::vector<RevisitCandidate> selected_revisits;
  const int revisit_index_separation =
      std::max(30, 3 * g_loop_local_window_size);
  for (const auto& candidate : revisit_candidates) {
    bool too_close = false;
    for (const auto& selected : selected_revisits) {
      if (std::abs(candidate.later - selected.later) <
          revisit_index_separation) {
        too_close = true;
        break;
      }
    }
    if (too_close) {
      continue;
    }
    selected_revisits.push_back(candidate);
    if (static_cast<int>(selected_revisits.size()) >=
        std::min(12, g_loop_candidate_limit)) {
      break;
    }
  }
  std::sort(
      selected_revisits.begin(),
      selected_revisits.end(),
      [](const RevisitCandidate& lhs, const RevisitCandidate& rhs) {
        return lhs.later < rhs.later;
      });

  // Endpoint-only graph is merely a reference hypothesis. It does not force
  // the endpoint to match any particular place. Internal edges are compared
  // against this smooth field and receive independent switch weights.
  PoseVector consistency_reference_poses(
      loop_keyframes_.size(), SE3());
  if (endpoint_edge.has_value()) {
    if (!optimize_pose_graph(
            consistency_reference_poses, pose_graph_edges)) {
      LOG(ERROR) << RED
                 << " ---> 终点回环参考图优化失败，忽略该终点假设。"
                 << RESET;
      pose_graph_edges.pop_back();
      endpoint_edge.reset();
      candidate_idx = -1;
      consistency_reference_poses.assign(
          loop_keyframes_.size(), SE3());
    }
  }

  int accepted_internal_loops = 0;
  for (const auto& revisit : selected_revisits) {
    if (finalize_timed_out()) {
      return;
    }
    // The endpoint closure already aligns local windows around candidate_idx
    // and end_idx.  An internal edge built from substantially the same points
    // is correlated evidence, not an independent loop.  Detect it here so it
    // can be retained at low weight without concentrating metres of
    // correction into the first/last handful of poses.
    const int endpoint_overlap_margin =
        2 * g_loop_local_window_size;
    const bool overlaps_endpoint_closure =
        candidate_idx >= 0 &&
        std::abs(revisit.earlier - candidate_idx) <=
            endpoint_overlap_margin &&
        end_idx - revisit.later <= endpoint_overlap_margin;
    if (overlaps_endpoint_closure) {
      LOG(INFO) << GREEN
                << " ---> 中途回环与终点回环局部窗口重叠，按相关观测降权。"
                << " earlier_idx: " << revisit.earlier
                << " later_idx: " << revisit.later
                << " endpoint_candidate_idx: " << candidate_idx
                << " end_idx: " << end_idx
                << " edge_weight: 0.25" << RESET;
    }

    CloudPtr revisit_source =
        build_local_world_cloud(revisit.later);
    CloudPtr revisit_target =
        build_local_world_cloud(revisit.earlier);
    if (revisit_source->empty() || revisit_target->empty()) {
      continue;
    }

    Eigen::Matrix4f initial_guess =
        Eigen::Matrix4f::Identity();
    initial_guess.block<3, 1>(0, 3) =
        loop_keyframes_[revisit.earlier].pose.t_.cast<float>() -
        loop_keyframes_[revisit.later].pose.t_.cast<float>();
    pcl::GeneralizedIterativeClosestPoint<PointType, PointType> icp;
    icp.setInputSource(revisit_source);
    icp.setInputTarget(revisit_target);
    icp.setMaxCorrespondenceDistance(g_loop_icp_max_distance);
    icp.setMaximumIterations(80);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-5);
    PointCloudType aligned;
    icp.align(aligned, initial_guess);
    const Eigen::Matrix4f revisit_correction =
        icp.getFinalTransformation();
    if (!icp.hasConverged() ||
        !revisit_correction.allFinite() ||
        aligned.empty()) {
      continue;
    }

    pcl::search::KdTree<PointType> target_tree;
    target_tree.setInputCloud(revisit_target);
    std::vector<int> nearest_index(1);
    std::vector<float> nearest_squared_distance(1);
    std::size_t overlap_count = 0;
    double squared_distance_sum = 0.0;
    const float max_squared_distance =
        g_loop_icp_max_distance * g_loop_icp_max_distance;
    for (const auto& point : aligned.points) {
      if (target_tree.nearestKSearch(
              point, 1, nearest_index,
              nearest_squared_distance) > 0 &&
          nearest_squared_distance[0] <= max_squared_distance) {
        ++overlap_count;
        squared_distance_sum += nearest_squared_distance[0];
      }
    }
    if (overlap_count == 0) {
      continue;
    }
    const double score =
        squared_distance_sum /
        static_cast<double>(overlap_count);
    const float overlap_ratio =
        static_cast<float>(overlap_count) /
        static_cast<float>(aligned.size());
    const Eigen::AngleAxisf rotation(
        revisit_correction.block<3, 3>(0, 0));
    const float rotation_deg =
        std::abs(rotation.angle()) * 180.0f /
        static_cast<float>(M_PI);
    const bool accepted =
        score <= g_loop_icp_score_threshold &&
        overlap_ratio >= g_loop_min_overlap_ratio &&
        rotation_deg <= g_loop_max_correction_rotation_deg;
    LOG(INFO) << GREEN
              << " ---> 中途回环候选。earlier_idx: "
              << revisit.earlier
              << " later_idx: " << revisit.later
              << " earlier_xyz: "
              << loop_keyframes_[revisit.earlier].pose.t_.transpose()
              << " later_xyz: "
              << loop_keyframes_[revisit.later].pose.t_.transpose()
              << " horizontal_distance: "
              << revisit.horizontal_distance
              << " score: " << score
              << " overlap_ratio: " << overlap_ratio
              << " rotation_deg: " << rotation_deg
              << " accepted: " << accepted << RESET;
    if (!accepted) {
      continue;
    }

    PoseGraphEdge edge;
    edge.from = revisit.earlier;
    edge.to = revisit.later;
    edge.measurement =
        project_gravity_aligned_loop_correction(
            revisit_correction,
            raw_poses[revisit.later].t_);
    const float segment_path_length =
        cumulative_route_length[revisit.later] -
        cumulative_route_length[revisit.earlier];
    const LoopConsistency consistency = evaluate_loop_consistency(
        edge, consistency_reference_poses, segment_path_length);
    const bool graph_consistent =
        consistency.weight >= g_loop_min_consistency_weight;
    LOG(INFO) << GREEN
              << " ---> 中途回环图一致性。earlier_idx: "
              << revisit.earlier
              << " later_idx: " << revisit.later
              << " path_length: " << consistency.path_length
              << " translation_residual: "
              << consistency.translation_residual
              << " rotation_residual_deg: "
              << consistency.rotation_residual_deg
              << " consistency_weight: " << consistency.weight
              << " graph_consistent: " << graph_consistent << RESET;
    if (!graph_consistent) {
      continue;
    }

    // Overlapping endpoint/internal windows reuse most of the same points.
    // Retain the extra observation as weak corroboration, but do not count it
    // as an independent full-strength constraint. The switch weight is
    // optimized from the whole XYZ+yaw residual and travelled path length.
    edge.weight =
        (overlaps_endpoint_closure ? 0.25f : 1.5f) *
        consistency.weight;
    edge.loop = true;
    pose_graph_edges.push_back(edge);
    ++accepted_internal_loops;
  }

  if (candidate_idx < 0 && accepted_internal_loops == 0) {
    LOG(INFO) << GREEN
              << " ---> 没有通过完整验证的终点或中途回环；"
              << "保留原始 map.pcd，不生成 map_loop.pcd。" << RESET;
    return;
  }

  if (finalize_timed_out()) {
    return;
  }

  const float initial_graph_cost =
      pose_graph_robust_cost(
          correction_poses, pose_graph_edges);
  if (!optimize_pose_graph(
          correction_poses, pose_graph_edges)) {
    LOG(ERROR) << RED
               << " ---> 多回环位姿图优化失败，拒绝生成闭环地图。"
               << RESET;
    return;
  }
  const float optimized_graph_cost =
      pose_graph_robust_cost(
          correction_poses, pose_graph_edges);

  float max_correction_translation = 0.0f;
  float max_correction_rotation_deg = 0.0f;
  float max_adjacent_translation = 0.0f;
  float max_adjacent_rotation_deg = 0.0f;
  for (std::size_t i = 0; i < correction_poses.size(); ++i) {
    max_correction_translation =
        std::max(
            max_correction_translation,
            correction_poses[i].t_.norm());
    max_correction_rotation_deg =
        std::max(
            max_correction_rotation_deg,
            correction_poses[i].so3().log_vee().norm() *
                180.0f / static_cast<float>(M_PI));
    if (i == 0) {
      continue;
    }
    const SE3 adjacent =
        correction_poses[i - 1].inverse() *
        correction_poses[i];
    max_adjacent_translation =
        std::max(max_adjacent_translation, adjacent.t_.norm());
    max_adjacent_rotation_deg =
        std::max(
            max_adjacent_rotation_deg,
            adjacent.so3().log_vee().norm() *
                180.0f / static_cast<float>(M_PI));
  }

  // A smooth deformation can still be globally wrong. Bound total XYZ
  // translation by a fraction of travelled route, never by a fixed Z or
  // fixed-metre ceiling. Reuse the per-loop rotation gate as the total yaw
  // correction gate.
  const float max_total_translation =
      0.05f * raw_route_length;
  if (!std::isfinite(optimized_graph_cost) ||
      optimized_graph_cost >= initial_graph_cost ||
      max_correction_translation > max_total_translation ||
      max_correction_rotation_deg >
          g_loop_max_correction_rotation_deg ||
      // A correction larger than 15 cm between neighbouring 0.5 m
      // keyframes is a local warp, even if the total graph deformation is
      // otherwise plausible.  Reject it instead of publishing a seam.
      max_adjacent_translation > 0.15f ||
      max_adjacent_rotation_deg > 2.0f) {
    LOG(ERROR) << RED
               << " ---> 位姿图修正不可信，拒绝生成闭环地图。"
               << " graph_cost: " << initial_graph_cost
               << " -> " << optimized_graph_cost
               << " raw_route_length: " << raw_route_length
               << " max_correction_translation: "
               << max_correction_translation
               << " max_total_translation: "
               << max_total_translation
               << " max_correction_rotation_deg: "
               << max_correction_rotation_deg
               << " max_total_rotation_deg: "
               << g_loop_max_correction_rotation_deg
               << " max_adjacent_translation: "
               << max_adjacent_translation
               << " max_adjacent_rotation_deg: "
               << max_adjacent_rotation_deg << RESET;
    return;
  }
  LOG(INFO) << GREEN
            << " ---> 多回环位姿图优化完成。nodes: "
            << correction_poses.size()
            << " internal_loops: " << accepted_internal_loops
            << " graph_cost: " << initial_graph_cost
            << " -> " << optimized_graph_cost
            << " raw_route_length: " << raw_route_length
            << " max_correction_translation: "
            << max_correction_translation
            << " max_correction_rotation_deg: "
            << max_correction_rotation_deg
            << " max_adjacent_translation: "
            << max_adjacent_translation
            << " max_adjacent_rotation_deg: "
            << max_adjacent_rotation_deg << RESET;

  CloudPtr loop_map(new PointCloudType());
  TimedPoseCorrectionVector pose_corrections;
  pose_corrections.reserve(loop_keyframes_.size());
  for (int i = 0; i <= end_idx; ++i) {
    if ((i % 64) == 0 && finalize_timed_out()) {
      return;
    }
    const SE3 optimized_pose =
        correction_poses[i] * raw_poses[i];
    CloudPtr transformed(new PointCloudType());
    pcl::transformPointCloud(
        *loop_keyframes_[i].cloud_body,
        *transformed,
        optimized_pose.T());
    *loop_map += *transformed;

    TimedPoseCorrection sample;
    sample.timestamp = loop_keyframes_[i].timestamp;
    sample.correction = correction_poses[i].T();
    pose_corrections.push_back(sample);
  }

  PointCloudType filtered_loop_map;
  if (finalize_timed_out()) {
    return;
  }
  make_map_pcd_cloud(loop_map, filtered_loop_map, g_loop_map_ds_size, "loop final");

  if (finalize_timed_out()) {
    return;
  }

  if (save_pcd_binary_safe(loop_map_path, filtered_loop_map)) {
    LOG(INFO) << GREEN << " ---> 闭环地图已保存: " << loop_map_path << RESET;
    LOG(INFO) << GREEN << " ---> 闭环地图点数: " << filtered_loop_map.size() << RESET;

    const std::filesystem::path canonical_map_path =
        std::filesystem::path(g_save_map_dir) / g_map_name;
    const std::filesystem::path raw_map_path =
        canonical_map_path.parent_path() /
        (canonical_map_path.stem().string() + "_raw" +
         canonical_map_path.extension().string());
    std::error_code copy_error;
    if (std::filesystem::exists(canonical_map_path)) {
      std::filesystem::copy_file(
          canonical_map_path,
          raw_map_path,
          std::filesystem::copy_options::overwrite_existing,
          copy_error);
      if (copy_error) {
        LOG(WARNING) << YELLOW << " ---> 原始未闭环地图备份失败: "
                     << raw_map_path.string()
                     << " error: " << copy_error.message() << RESET;
      }
    }

    if (!save_pcd_binary_safe(canonical_map_path.string(), filtered_loop_map)) {
      LOG(ERROR) << RED << " ---> 闭环地图无法提交为正式地图: "
                 << canonical_map_path.string() << RESET;
      return;
    }
    if (!save_loop_trajectory_safe(
            trajectory_path, pose_corrections)) {
      LOG(ERROR) << RED
                 << " ---> 正式地图已闭环，但位姿图轨迹保存失败。"
                 << RESET;
      return;
    }
    LOG(INFO) << GREEN << " ---> 闭环地图已提交为正式 map.pcd；"
              << "terrain 将按同一条优化轨迹重建（不锁定 Z）。"
              << RESET;
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


bool SuperLIO::Propagation_Undistort(){
  auto reject_undistortion = [this](const std::string& reason) {
    ++rejected_undistortion_count_;
    const auto now = std::chrono::steady_clock::now();
    if (last_undistortion_warning_time_ == std::chrono::steady_clock::time_point{} ||
        now - last_undistortion_warning_time_ > std::chrono::seconds(1)) {
      last_undistortion_warning_time_ = now;
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: reject LiDAR undistortion. " << reason
                   << " rejected=" << rejected_undistortion_count_ << RESET;
    }
    scan_undistort_full_->clear();
    ds_undistort_->clear();
    return false;
  };

  auto& raw_pc = measures_.lidar.pc;
  if (!raw_pc || raw_pc->empty()) {
    return reject_undistortion("empty point cloud");
  }

  constexpr double time_epsilon = 1e-6;
  const double coverage_tolerance =
      std::max(g_scan_boundary_tolerance, time_epsilon);
  double minimum_query_time = std::numeric_limits<double>::infinity();
  double maximum_query_time = -std::numeric_limits<double>::infinity();
  for (const auto& point : raw_pc->points) {
    if (!std::isfinite(point.offset_time) || point.offset_time < 0.0) {
      return reject_undistortion("invalid point offset time");
    }
    const double query_time = measures_.lidar.start_time + point.offset_time;
    minimum_query_time = std::min(minimum_query_time, query_time);
    maximum_query_time = std::max(maximum_query_time, query_time);
  }

  if (minimum_query_time < measures_.lidar.start_time - time_epsilon ||
      maximum_query_time > measures_.lidar.end_time + time_epsilon) {
    std::ostringstream reason;
    reason << "point time outside scan range: point=["
           << minimum_query_time << ", " << maximum_query_time
           << "] scan=[" << measures_.lidar.start_time
           << ", " << measures_.lidar.end_time << "]";
    return reject_undistortion(reason.str());
  }

  propagate_states_.clear();
  propagate_states_.emplace_back(kf_->GetDynamicState());
  kf_->SetObsTime(measures_.lidar.end_time);
  for (auto &imu : measures_.imu) {
    kf_->Predict(imu);
    propagate_states_.emplace_back(kf_->GetDynamicState());
  }

  // Keep a short gravity-direction average for validating local plane normals.
  // It is deliberately not used as a direct attitude correction: walking
  // acceleration can tilt an accelerometer-only estimate by several degrees.
  updateGravityReference();

  static const M3 TLI_R = g_lidar_imu.R_;
  static const V3 TLI_t = g_lidar_imu.t_;
  const SE3 T_end = kf_->GetSE3();
  const M3  R_inv = T_end.R_.transpose();
  const V3  T_end_t = T_end.t_;
  const double start_time = measures_.lidar.start_time;

  if (propagate_states_.size() < 2 ||
      propagate_states_.front().time > minimum_query_time + coverage_tolerance ||
      propagate_states_.back().time < maximum_query_time - coverage_tolerance) {
    std::ostringstream reason;
    reason << "state coverage incomplete: state=["
           << propagate_states_.front().time << ", "
           << propagate_states_.back().time << "] point=["
           << minimum_query_time << ", " << maximum_query_time << "]";
    return reject_undistortion(reason.str());
  }

  std::size_t ptsize = raw_pc->points.size();
  scan_undistort_full_->resize(ptsize); 

  tbb::parallel_for(
  tbb::blocked_range<size_t>(0, ptsize),
  [&](const tbb::blocked_range<size_t>& r) {
    M3 R_h, R_t; V3 p_h, v_h, acc_t;
    for (size_t idx = r.begin(); idx < r.end(); ++idx) {  
      auto& pt_full = scan_undistort_full_->points[idx];
      const auto& pt = raw_pc->points[idx];
      pt_full.intensity = pt.intensity;
      double query_time = start_time + pt.offset_time;

      auto match_iter_n = std::upper_bound(
        propagate_states_.begin(), propagate_states_.end(), query_time,
        [](double time, const DynamicState& state) { return time < state.time; });
      if (match_iter_n == propagate_states_.begin()) {
        match_iter_n = std::next(propagate_states_.begin());
      } else if (match_iter_n == propagate_states_.end()) {
        match_iter_n = std::prev(propagate_states_.end());
      }
      auto match_iter = std::prev(match_iter_n);
      double imu_dt = match_iter_n->time - match_iter->time;
      const double query_dt = std::clamp(
        query_time - match_iter->time, 0.0, std::max(imu_dt, 0.0));
      const double s = imu_dt > 1e-9 ? query_dt / imu_dt : 0.0;
      R_h = match_iter->R;
      R_t = match_iter_n->R;
      p_h = match_iter->p;
      v_h = match_iter->v;
      acc_t = match_iter_n->a;
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
  return true;
}


void SuperLIO::DownSample(){
  voxel_grid_fliter_.setInputCloud(scan_undistort_full_);
  voxel_grid_fliter_.filter(ds_undistort_);
}


void SuperLIO::updateGravityReference(){
  if (!g_level_constraint_enable || imu_reference_accel_norm_ <= 1e-6 ||
      measures_.imu.empty() || propagate_states_.size() < 2) {
    gravity_reference_valid_ = false;
    return;
  }

  const std::size_t sample_count = std::min(
      measures_.imu.size(), propagate_states_.size() - 1);
  for (std::size_t i = 0; i < sample_count; ++i) {
    const auto& imu = measures_.imu[i];
    if (imu.secs <= last_gravity_sample_time_ + 1e-9) {
      continue;
    }
    last_gravity_sample_time_ = imu.secs;

    const double accel_norm = imu.acc.norm();
    if (!std::isfinite(accel_norm) || accel_norm <= 1e-6) {
      continue;
    }
    const double norm_error = std::abs(
        accel_norm / imu_reference_accel_norm_ - 1.0);
    if (norm_error > g_level_max_accel_norm_ratio) {
      continue;
    }

    V3 up_world = propagate_states_[i + 1].R * (imu.acc / accel_norm);
    const scalar up_norm = up_world.norm();
    if (!up_world.allFinite() || up_norm <= 1e-6) {
      continue;
    }
    gravity_direction_window_.push_back(
        GravityDirectionSample{imu.secs, up_world / up_norm});
  }

  const double newest_time = measures_.imu.back().secs;
  while (!gravity_direction_window_.empty() &&
         newest_time - gravity_direction_window_.front().timestamp >
             g_level_gravity_window_sec) {
    gravity_direction_window_.pop_front();
  }

  const double minimum_duration = std::min(
      0.5, 0.5 * g_level_gravity_window_sec);
  if (gravity_direction_window_.size() < 20 ||
      gravity_direction_window_.back().timestamp -
              gravity_direction_window_.front().timestamp < minimum_duration) {
    gravity_reference_valid_ = false;
    return;
  }

  V3 mean_up = V3::Zero();
  for (const auto& sample : gravity_direction_window_) {
    mean_up += sample.up_world;
  }
  const scalar mean_norm = mean_up.norm();
  gravity_reference_valid_ = mean_up.allFinite() && mean_norm > 1e-6;
  if (gravity_reference_valid_) {
    gravity_reference_world_ = mean_up / mean_norm;
  }
}


SuperLIO::LevelPlaneObservation SuperLIO::estimateLevelPlane() const {
  LevelPlaneObservation observation;
  if (!g_level_constraint_enable || !gravity_reference_valid_ ||
      !ds_undistort_ || ds_undistort_->empty()) {
    return observation;
  }

  const SE3 predicted_pose = kf_->GetSE3();
  V3 up_body = predicted_pose.R_.transpose() * gravity_reference_world_;
  const scalar up_body_norm = up_body.norm();
  if (!up_body.allFinite() || up_body_norm <= 1e-6) {
    return observation;
  }
  up_body /= up_body_norm;

  std::vector<V3> candidates;
  candidates.reserve(ds_undistort_->size());
  const double max_range_squared =
      g_level_max_point_range * g_level_max_point_range;
  for (const auto& point : ds_undistort_->points) {
    const V3 p(point.x, point.y, point.z);
    if (!p.allFinite()) {
      continue;
    }
    const double range_squared = p.squaredNorm();
    if (range_squared < 0.25 || range_squared > max_range_squared) {
      continue;
    }
    const double down_projection = -up_body.dot(p);
    if (down_projection < g_level_min_down_distance ||
        down_projection > g_level_max_down_distance) {
      continue;
    }
    candidates.push_back(p);
  }

  observation.candidate_count = static_cast<int>(candidates.size());
  const int required_inliers = std::max(
      g_level_min_plane_inliers,
      static_cast<int>(std::ceil(
          g_level_min_plane_inlier_ratio * candidates.size())));
  if (static_cast<int>(candidates.size()) < required_inliers ||
      candidates.size() < 3) {
    return observation;
  }

  // The RANSAC seed is frame-derived, making bag replays deterministic while
  // avoiding a geometry bias from repeatedly choosing the same point triples.
  std::mt19937 generator(
      static_cast<std::uint32_t>(frame_num_) * 2654435761u + 0x9e3779b9u);
  std::uniform_int_distribution<std::size_t> sample_index(
      0, candidates.size() - 1);
  const double candidate_angle_deg = std::max(
      10.0, 2.0 * g_level_max_plane_gravity_angle_deg);
  const double candidate_cosine = std::cos(
      candidate_angle_deg * M_PI / 180.0);

  int best_count = 0;
  V3 best_normal = V3::UnitZ();
  double best_offset = 0.0;
  for (int iteration = 0; iteration < g_level_ransac_iterations; ++iteration) {
    const std::size_t i0 = sample_index(generator);
    const std::size_t i1 = sample_index(generator);
    const std::size_t i2 = sample_index(generator);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    V3 normal = (candidates[i1] - candidates[i0]).cross(
        candidates[i2] - candidates[i0]);
    const scalar normal_norm = normal.norm();
    if (!normal.allFinite() || normal_norm <= 1e-5) {
      continue;
    }
    normal /= normal_norm;
    if (normal.dot(up_body) < 0.0) {
      normal = -normal;
    }
    if (normal.dot(up_body) < candidate_cosine) {
      continue;
    }

    const double offset = -normal.dot(candidates[i0]);
    int inlier_count = 0;
    for (const auto& candidate : candidates) {
      if (std::abs(normal.dot(candidate) + offset) <=
          g_level_plane_distance_threshold) {
        ++inlier_count;
      }
    }
    if (inlier_count > best_count) {
      best_count = inlier_count;
      best_normal = normal;
      best_offset = offset;
    }
  }

  if (best_count < required_inliers) {
    return observation;
  }

  V3 centroid = V3::Zero();
  std::vector<const V3*> inliers;
  inliers.reserve(best_count);
  for (const auto& candidate : candidates) {
    if (std::abs(best_normal.dot(candidate) + best_offset) <=
        g_level_plane_distance_threshold) {
      inliers.push_back(&candidate);
      centroid += candidate;
    }
  }
  if (static_cast<int>(inliers.size()) < required_inliers) {
    return observation;
  }
  centroid /= static_cast<scalar>(inliers.size());

  M3 covariance = M3::Zero();
  for (const V3* point : inliers) {
    const V3 centered = *point - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<scalar>(inliers.size());
  Eigen::SelfAdjointEigenSolver<M3> eigen_solver(covariance);
  if (eigen_solver.info() != Eigen::Success) {
    return observation;
  }
  const V3 eigenvalues = eigen_solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues[1] < 0.04) {
    return observation;
  }

  V3 refined_normal = eigen_solver.eigenvectors().col(0);
  if (refined_normal.dot(up_body) < 0.0) {
    refined_normal = -refined_normal;
  }
  const double refined_offset = -refined_normal.dot(centroid);
  double squared_error_sum = 0.0;
  int refined_inlier_count = 0;
  for (const auto& candidate : candidates) {
    const double error = refined_normal.dot(candidate) + refined_offset;
    if (std::abs(error) <= g_level_plane_distance_threshold) {
      squared_error_sum += error * error;
      ++refined_inlier_count;
    }
  }
  if (refined_inlier_count < required_inliers) {
    return observation;
  }

  const double rms = std::sqrt(
      squared_error_sum / static_cast<double>(refined_inlier_count));
  if (!std::isfinite(rms) ||
      rms > 0.8 * g_level_plane_distance_threshold) {
    return observation;
  }

  const auto angle_deg = [](const V3& a, const V3& b) {
    return std::acos(std::clamp(
        static_cast<double>(a.dot(b)), -1.0, 1.0)) * 180.0 / M_PI;
  };
  const double gravity_angle_deg = angle_deg(refined_normal, up_body);
  if (gravity_angle_deg > g_level_max_plane_gravity_angle_deg) {
    return observation;
  }

  V3 plane_up_world = predicted_pose.R_ * refined_normal;
  plane_up_world.normalize();
  const double innovation_deg = angle_deg(plane_up_world, V3::UnitZ());
  if (innovation_deg > g_level_max_attitude_innovation_deg) {
    return observation;
  }

  observation.valid = true;
  observation.normal_body = refined_normal;
  observation.inlier_count = refined_inlier_count;
  observation.inlier_ratio = static_cast<double>(refined_inlier_count) /
      static_cast<double>(candidates.size());
  observation.rms = rms;
  observation.gravity_angle_deg = gravity_angle_deg;
  observation.innovation_deg = innovation_deg;
  return observation;
}


SuperLIO::WallYawObservation SuperLIO::estimateWallYaw() const {
  WallYawObservation observation;
  if (!g_wall_yaw_constraint_enable || !gravity_reference_valid_ ||
      !ds_undistort_ || ds_undistort_->empty()) {
    return observation;
  }
  if (frame_num_ % g_wall_yaw_extraction_interval_frames != 0) {
    return observation;
  }
  observation.extraction_attempted = true;

  const SE3 predicted_pose = kf_->GetSE3();
  V3 up_body = predicted_pose.R_.transpose() * gravity_reference_world_;
  const scalar up_body_norm = up_body.norm();
  if (!up_body.allFinite() || up_body_norm <= 1e-6) {
    return observation;
  }
  up_body /= up_body_norm;

  std::vector<V3> candidates;
  candidates.reserve(ds_undistort_->size());
  const double min_range_squared = 1.0;
  const double max_range_squared =
      g_wall_yaw_max_point_range * g_wall_yaw_max_point_range;
  for (const auto& point : ds_undistort_->points) {
    const V3 candidate(point.x, point.y, point.z);
    if (!candidate.allFinite()) {
      continue;
    }
    const double range_squared = candidate.squaredNorm();
    if (range_squared < min_range_squared ||
        range_squared > max_range_squared) {
      continue;
    }
    candidates.push_back(candidate);
  }

  observation.candidate_count = static_cast<int>(candidates.size());
  const int required_inliers = std::max(
      g_wall_yaw_min_plane_inliers,
      static_cast<int>(std::ceil(
          g_wall_yaw_min_plane_inlier_ratio * candidates.size())));
  if (static_cast<int>(candidates.size()) < required_inliers ||
      candidates.size() < 3) {
    return observation;
  }

  // 固定随机种子保证同一 rosbag 的墙面提取结果可重复。
  std::mt19937 generator(
      static_cast<std::uint32_t>(frame_num_) * 2654435761u + 0x6a09e667u);
  std::uniform_int_distribution<std::size_t> sample_index(
      0, candidates.size() - 1);
  const double candidate_vertical_angle_deg = std::max(
      15.0, 2.0 * g_wall_yaw_max_vertical_angle_deg);
  const double candidate_vertical_sine = std::sin(
      candidate_vertical_angle_deg * M_PI / 180.0);

  int best_count = 0;
  V3 best_normal = V3::UnitX();
  double best_offset = 0.0;
  for (int iteration = 0;
       iteration < g_wall_yaw_ransac_iterations;
       ++iteration) {
    const std::size_t i0 = sample_index(generator);
    const std::size_t i1 = sample_index(generator);
    const std::size_t i2 = sample_index(generator);
    if (i0 == i1 || i0 == i2 || i1 == i2) {
      continue;
    }

    V3 normal = (candidates[i1] - candidates[i0]).cross(
        candidates[i2] - candidates[i0]);
    const scalar normal_norm = normal.norm();
    if (!normal.allFinite() || normal_norm <= 1e-5) {
      continue;
    }
    normal /= normal_norm;
    if (std::abs(normal.dot(up_body)) > candidate_vertical_sine) {
      continue;
    }

    const double offset = -normal.dot(candidates[i0]);
    int inlier_count = 0;
    for (const auto& candidate : candidates) {
      if (std::abs(normal.dot(candidate) + offset) <=
          g_wall_yaw_plane_distance_threshold) {
        ++inlier_count;
      }
    }
    if (inlier_count > best_count) {
      best_count = inlier_count;
      best_normal = normal;
      best_offset = offset;
    }
  }

  if (best_count < required_inliers) {
    return observation;
  }

  V3 centroid = V3::Zero();
  std::vector<const V3*> inliers;
  inliers.reserve(best_count);
  for (const auto& candidate : candidates) {
    if (std::abs(best_normal.dot(candidate) + best_offset) <=
        g_wall_yaw_plane_distance_threshold) {
      inliers.push_back(&candidate);
      centroid += candidate;
    }
  }
  if (static_cast<int>(inliers.size()) < required_inliers) {
    return observation;
  }
  centroid /= static_cast<scalar>(inliers.size());

  M3 covariance = M3::Zero();
  for (const V3* point : inliers) {
    const V3 centered = *point - centroid;
    covariance.noalias() += centered * centered.transpose();
  }
  covariance /= static_cast<scalar>(inliers.size());
  Eigen::SelfAdjointEigenSolver<M3> eigen_solver(covariance);
  if (eigen_solver.info() != Eigen::Success) {
    return observation;
  }
  const V3 eigenvalues = eigen_solver.eigenvalues();
  if (!eigenvalues.allFinite() || eigenvalues[1] < 0.04) {
    return observation;
  }

  V3 refined_normal = eigen_solver.eigenvectors().col(0);
  if (refined_normal.dot(best_normal) < 0.0) {
    refined_normal = -refined_normal;
  }
  const double vertical_angle_deg = std::asin(std::clamp(
      std::abs(static_cast<double>(refined_normal.dot(up_body))),
      0.0, 1.0)) * 180.0 / M_PI;
  if (vertical_angle_deg > g_wall_yaw_max_vertical_angle_deg) {
    return observation;
  }

  const double refined_offset = -refined_normal.dot(centroid);
  std::vector<const V3*> refined_inliers;
  refined_inliers.reserve(inliers.size());
  double squared_error_sum = 0.0;
  for (const auto& candidate : candidates) {
    const double error = refined_normal.dot(candidate) + refined_offset;
    if (std::abs(error) <= g_wall_yaw_plane_distance_threshold) {
      refined_inliers.push_back(&candidate);
      squared_error_sum += error * error;
    }
  }
  if (static_cast<int>(refined_inliers.size()) < required_inliers) {
    return observation;
  }

  const double rms = std::sqrt(
      squared_error_sum / static_cast<double>(refined_inliers.size()));
  if (!std::isfinite(rms) ||
      rms > 0.8 * g_wall_yaw_plane_distance_threshold) {
    return observation;
  }

  V3 horizontal_tangent = up_body.cross(refined_normal);
  const scalar tangent_norm = horizontal_tangent.norm();
  if (!horizontal_tangent.allFinite() || tangent_norm <= 1e-6) {
    return observation;
  }
  horizontal_tangent /= tangent_norm;

  std::vector<double> vertical_coordinates;
  std::vector<double> horizontal_coordinates;
  vertical_coordinates.reserve(refined_inliers.size());
  horizontal_coordinates.reserve(refined_inliers.size());
  for (const V3* point : refined_inliers) {
    const V3 centered = *point - centroid;
    vertical_coordinates.push_back(centered.dot(up_body));
    horizontal_coordinates.push_back(centered.dot(horizontal_tangent));
  }
  std::sort(vertical_coordinates.begin(), vertical_coordinates.end());
  std::sort(horizontal_coordinates.begin(), horizontal_coordinates.end());
  const std::size_t lower_index = static_cast<std::size_t>(std::floor(
      0.05 * static_cast<double>(refined_inliers.size() - 1)));
  const std::size_t upper_index = static_cast<std::size_t>(std::ceil(
      0.95 * static_cast<double>(refined_inliers.size() - 1)));
  const double vertical_span =
      vertical_coordinates[upper_index] - vertical_coordinates[lower_index];
  const double horizontal_span =
      horizontal_coordinates[upper_index] - horizontal_coordinates[lower_index];
  if (!std::isfinite(vertical_span) || !std::isfinite(horizontal_span) ||
      vertical_span < g_wall_yaw_min_vertical_span ||
      horizontal_span < g_wall_yaw_min_horizontal_span) {
    return observation;
  }

  observation.plane_valid = true;
  observation.normal_body = refined_normal;
  observation.inlier_count = static_cast<int>(refined_inliers.size());
  observation.inlier_ratio = static_cast<double>(refined_inliers.size()) /
      static_cast<double>(candidates.size());
  observation.rms = rms;
  observation.vertical_angle_deg = vertical_angle_deg;
  observation.vertical_span = vertical_span;
  observation.horizontal_span = horizontal_span;

  return observation;
}


void SuperLIO::prepareWallYawConstraint(
    WallYawObservation& observation,
    const SE3& pose,
    const double lidar_yaw_information_ratio) {
  observation.lidar_yaw_information_ratio = std::clamp(
      lidar_yaw_information_ratio, 0.0, 1.0);
  observation.reference_valid = !wall_yaw_references_.empty();

  const double weak_ratio = g_wall_yaw_information_weak_ratio;
  const double strong_ratio = g_wall_yaw_information_strong_ratio;
  if (observation.lidar_yaw_information_ratio <= weak_ratio) {
    observation.observability_gate = 1.0;
  } else if (observation.lidar_yaw_information_ratio >= strong_ratio) {
    observation.observability_gate = 0.0;
  } else {
    const double linear_gate =
        (strong_ratio - observation.lidar_yaw_information_ratio) /
        (strong_ratio - weak_ratio);
    // Smoothstep avoids a discontinuous correction when geometry changes.
    observation.observability_gate =
        linear_gate * linear_gate * (3.0 - 2.0 * linear_gate);
  }

  if (!observation.plane_valid) {
    if (!wall_yaw_reference_samples_.empty() &&
        frame_num_ - wall_yaw_reference_samples_.back().frame >
            20 * g_wall_yaw_extraction_interval_frames) {
      wall_yaw_reference_samples_.clear();
    }
    return;
  }

  V3 normal_world = pose.R_ * observation.normal_body;
  normal_world.z() = 0.0;
  const scalar horizontal_normal_norm = normal_world.norm();
  if (!normal_world.allFinite() || horizontal_normal_norm <= 1e-6) {
    return;
  }
  normal_world /= horizontal_normal_norm;
  const double observed_angle = std::atan2(normal_world.y(), normal_world.x());

  auto target_for_axis = [&](const double axis_rad) {
    double best_dot = -std::numeric_limits<double>::infinity();
    V3 best_target = V3::UnitX();
    for (int axis_index = 0; axis_index < 4; ++axis_index) {
      const double target_angle = axis_rad + axis_index * M_PI_2;
      const V3 target(
          std::cos(target_angle), std::sin(target_angle), 0.0);
      const double dot = normal_world.dot(target);
      if (dot > best_dot) {
        best_dot = dot;
        best_target = target;
      }
    }
    return best_target;
  };
  auto signed_innovation_to = [&](const V3& target) {
    return std::atan2(
        normal_world.x() * target.y() - normal_world.y() * target.x(),
        std::clamp(static_cast<double>(normal_world.dot(target)), -1.0, 1.0));
  };

  int best_reference_index = -1;
  double best_reference_score = std::numeric_limits<double>::infinity();
  double best_reference_distance = 0.0;
  V3 best_target = V3::UnitX();
  double best_innovation_rad = 0.0;
  for (std::size_t index = 0; index < wall_yaw_references_.size(); ++index) {
    const auto& reference = wall_yaw_references_[index];
    const double distance = (pose.t_ - reference.center).norm();
    if (!std::isfinite(distance) || distance > g_wall_yaw_reference_radius_m) {
      continue;
    }
    const V3 target = target_for_axis(reference.axis_rad);
    const double innovation_rad = signed_innovation_to(target);
    const double innovation_deg = std::abs(innovation_rad) * 180.0 / M_PI;
    if (innovation_deg > g_wall_yaw_max_innovation_deg) {
      continue;
    }
    const double score = innovation_deg / g_wall_yaw_max_innovation_deg +
        0.25 * distance / g_wall_yaw_reference_radius_m;
    if (score < best_reference_score) {
      best_reference_score = score;
      best_reference_index = static_cast<int>(index);
      best_reference_distance = distance;
      best_target = target;
      best_innovation_rad = innovation_rad;
    }
  }

  if (best_reference_index >= 0) {
    auto& reference =
        wall_yaw_references_[static_cast<std::size_t>(best_reference_index)];
    reference.last_used_frame = static_cast<std::uint64_t>(frame_num_);
    ++reference.accepted_count;
    wall_yaw_reference_samples_.clear();
    observation.reference_index = best_reference_index;
    observation.reference_valid = true;
    observation.target_normal_world = best_target;
    observation.innovation_deg =
        std::abs(best_innovation_rad) * 180.0 / M_PI;
    observation.valid = true;

    // A fixed local radius is safe around turns and independently oriented
    // buildings, but by itself leaves a long degenerate corridor uncovered
    // after the robot walks out of the first radius. Once the same wall axis
    // is observed near the edge, copy the already-verified axis to a new
    // local anchor. Never derive the copied axis from the weak current pose.
    const double extension_distance =
        g_wall_yaw_reference_extension_ratio * g_wall_yaw_reference_radius_m;
    if (best_reference_distance >= extension_distance) {
      WallYawReference extension;
      extension.axis_rad = reference.axis_rad;
      extension.center = pose.t_;
      extension.last_used_frame = static_cast<std::uint64_t>(frame_num_);
      extension.accepted_count = 1;

      if (wall_yaw_references_.size() >=
          static_cast<std::size_t>(g_wall_yaw_max_references)) {
        const auto oldest = std::min_element(
            wall_yaw_references_.begin(), wall_yaw_references_.end(),
            [](const WallYawReference& lhs, const WallYawReference& rhs) {
              return lhs.last_used_frame < rhs.last_used_frame;
            });
        if (oldest != wall_yaw_references_.end()) {
          wall_yaw_references_.erase(oldest);
        }
      }
      wall_yaw_references_.push_back(extension);
      observation.reference_index =
          static_cast<int>(wall_yaw_references_.size() - 1);
      LOG(INFO) << GREEN
                << " ---> [SuperLIO]: 续接长走廊墙面航向锚点。index="
                << observation.reference_index
                << " axis=" << extension.axis_rad * 180.0 / M_PI
                << "deg center=" << extension.center.transpose()
                << " source_distance=" << best_reference_distance
                << "m total_references=" << wall_yaw_references_.size()
                << RESET;
    }
    return;
  }

  // A new local reference may only be learned while normal LiDAR matching has
  // sufficient yaw information. In a degenerate corridor we use an existing
  // reference, but never turn the already-drifting pose into a new truth.
  if (observation.lidar_yaw_information_ratio <
      g_wall_yaw_reference_min_yaw_information_ratio) {
    if (!wall_yaw_reference_samples_.empty() &&
        frame_num_ - wall_yaw_reference_samples_.back().frame >
            20 * g_wall_yaw_extraction_interval_frames) {
      wall_yaw_reference_samples_.clear();
    }
    return;
  }

  if (!wall_yaw_reference_samples_.empty()) {
    std::deque<double> sample_angles;
    V3 sample_center = V3::Zero();
    for (const auto& sample : wall_yaw_reference_samples_) {
      sample_angles.push_back(sample.axis_rad);
      sample_center += sample.position;
    }
    sample_center /= static_cast<scalar>(wall_yaw_reference_samples_.size());
    const double candidate_axis = mean_manhattan_axis(sample_angles);
    const double deviation_deg = std::abs(wrap_period_signed(
        observed_angle - candidate_axis, M_PI_2)) * 180.0 / M_PI;
    const double spatial_deviation = (pose.t_ - sample_center).norm();
    const bool stale =
        frame_num_ - wall_yaw_reference_samples_.back().frame >
        20 * g_wall_yaw_extraction_interval_frames;
    if (stale ||
        spatial_deviation > 0.5 * g_wall_yaw_reference_radius_m) {
      wall_yaw_reference_samples_.clear();
    } else if (deviation_deg >
               g_wall_yaw_reference_max_deviation_deg) {
      // The largest plane can briefly switch from a building wall to a
      // cabinet, escalator side or vehicle. Ignore that observation instead
      // of letting one outlier erase an otherwise stable local direction.
      return;
    }
  }

  wall_yaw_reference_samples_.push_back(
      WallYawReferenceSample{observed_angle, pose.t_, frame_num_});
  if (wall_yaw_reference_samples_.size() <
      static_cast<std::size_t>(g_wall_yaw_reference_min_frames)) {
    return;
  }

  std::deque<double> sample_angles;
  V3 reference_center = V3::Zero();
  for (const auto& sample : wall_yaw_reference_samples_) {
    sample_angles.push_back(sample.axis_rad);
    reference_center += sample.position;
  }
  reference_center /= static_cast<scalar>(wall_yaw_reference_samples_.size());
  WallYawReference reference;
  reference.axis_rad = mean_manhattan_axis(sample_angles);
  reference.center = reference_center;
  reference.last_used_frame = static_cast<std::uint64_t>(frame_num_);
  reference.accepted_count = 1;

  if (wall_yaw_references_.size() >=
      static_cast<std::size_t>(g_wall_yaw_max_references)) {
    const auto oldest = std::min_element(
        wall_yaw_references_.begin(), wall_yaw_references_.end(),
        [](const WallYawReference& lhs, const WallYawReference& rhs) {
          return lhs.last_used_frame < rhs.last_used_frame;
        });
    if (oldest != wall_yaw_references_.end()) {
      wall_yaw_references_.erase(oldest);
    }
  }
  wall_yaw_references_.push_back(reference);
  wall_yaw_reference_samples_.clear();

  observation.reference_index =
      static_cast<int>(wall_yaw_references_.size() - 1);
  observation.reference_valid = true;
  observation.target_normal_world = target_for_axis(reference.axis_rad);
  const double innovation_rad =
      signed_innovation_to(observation.target_normal_world);
  observation.innovation_deg = std::abs(innovation_rad) * 180.0 / M_PI;
  observation.valid =
      observation.innovation_deg <= g_wall_yaw_max_innovation_deg;

  LOG(INFO) << GREEN
            << " ---> [SuperLIO]: 新建局部墙面航向参考。index="
            << observation.reference_index
            << " axis=" << reference.axis_rad * 180.0 / M_PI
            << "deg center=" << reference.center.transpose()
            << " yaw_information_ratio="
            << observation.lidar_yaw_information_ratio
            << " total_references=" << wall_yaw_references_.size()
            << RESET;
}


struct ThreadACC{
  M6d HTVH = M6d::Zero();
  V6d HTVr = V6d::Zero();
  ThreadACC(): HTVH(M6d::Zero()), HTVr(V6d::Zero()) {}
};


void SuperLIO::Observe(){
  // 迭代观测会原地修改名义状态，因此保留 IMU 预测状态，避免错误匹配污染后续帧。
  const SysState predicted_state = kf_->GetSysState();
  const ESKF::COV predicted_covariance = kf_->GetCov();
  const LevelPlaneObservation level_observation = estimateLevelPlane();
  WallYawObservation wall_yaw_observation = estimateWallYaw();
  if (level_observation.valid) {
    ++level_constraint_accepted_count_;
  } else {
    ++level_constraint_rejected_count_;
  }

  const double inlier_confidence = level_observation.valid
      ? std::clamp(
          static_cast<double>(level_observation.inlier_count) /
              static_cast<double>(2 * g_level_min_plane_inliers),
          0.25, 1.0)
      : 0.0;
  const double ratio_confidence = level_observation.valid
      ? std::clamp(
          level_observation.inlier_ratio /
              (2.0 * g_level_min_plane_inlier_ratio),
          0.25, 1.0)
      : 0.0;
  const double rms_confidence = level_observation.valid
      ? std::clamp(
          1.0 - level_observation.rms /
              g_level_plane_distance_threshold,
          0.25, 1.0)
      : 0.0;
  const double gravity_confidence = level_observation.valid
      ? std::clamp(
          1.0 - level_observation.gravity_angle_deg /
              g_level_max_plane_gravity_angle_deg,
          0.25, 1.0)
      : 0.0;
  const double level_confidence = level_observation.valid
      ? std::clamp(
          0.25 * (inlier_confidence + ratio_confidence +
                  rms_confidence + gravity_confidence),
          0.25, 1.0)
      : 0.0;
  const double level_sigma_rad =
      g_level_attitude_stddev_deg * M_PI / 180.0;
  const double level_information = level_observation.valid
      ? level_confidence / (level_sigma_rad * level_sigma_rad)
      : 0.0;
  const double wall_inlier_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          static_cast<double>(wall_yaw_observation.inlier_count) /
              static_cast<double>(2 * g_wall_yaw_min_plane_inliers),
          0.25, 1.0)
      : 0.0;
  const double wall_ratio_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          wall_yaw_observation.inlier_ratio /
              (2.0 * g_wall_yaw_min_plane_inlier_ratio),
          0.25, 1.0)
      : 0.0;
  const double wall_rms_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          1.0 - wall_yaw_observation.rms /
              g_wall_yaw_plane_distance_threshold,
          0.25, 1.0)
      : 0.0;
  const double wall_vertical_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          1.0 - wall_yaw_observation.vertical_angle_deg /
              g_wall_yaw_max_vertical_angle_deg,
          0.25, 1.0)
      : 0.0;
  const double wall_span_confidence = wall_yaw_observation.plane_valid
      ? 0.5 * (
          std::clamp(
              wall_yaw_observation.vertical_span /
                  (2.0 * g_wall_yaw_min_vertical_span),
              0.25, 1.0) +
          std::clamp(
              wall_yaw_observation.horizontal_span /
                  (2.0 * g_wall_yaw_min_horizontal_span),
              0.25, 1.0))
      : 0.0;
  const double wall_yaw_confidence = wall_yaw_observation.plane_valid
      ? std::clamp(
          0.2 * (wall_inlier_confidence + wall_ratio_confidence +
                 wall_rms_confidence + wall_vertical_confidence +
                 wall_span_confidence),
          0.25, 1.0)
      : 0.0;
  const double wall_yaw_sigma_rad =
      g_wall_yaw_stddev_deg * M_PI / 180.0;
  const double wall_yaw_base_information = wall_yaw_observation.plane_valid
      ? wall_yaw_confidence /
          (wall_yaw_sigma_rad * wall_yaw_sigma_rad)
      : 0.0;
  double wall_yaw_information = 0.0;
  bool wall_yaw_prepared = false;
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
  V3d lidar_rotation_information_eigenvalues = V3d::Zero();

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

    Eigen::SelfAdjointEigenSolver<M3d> lidar_information_solver(
        sum_HTVH.block<3, 3>(0, 0));
    if (lidar_information_solver.info() == Eigen::Success) {
      lidar_rotation_information_eigenvalues =
          lidar_information_solver.eigenvalues();
    }

    const M3d rotation = pose.R_.cast<double>();
    V3d yaw_axis_body = rotation.transpose() * V3d::UnitZ();
    yaw_axis_body.normalize();
    const M3d lidar_rotation_information =
        sum_HTVH.block<3, 3>(0, 0);
    const double lidar_yaw_information = std::max(
        yaw_axis_body.dot(lidar_rotation_information * yaw_axis_body), 0.0);
    const double strongest_rotation_information = std::max(
        lidar_rotation_information_eigenvalues.maxCoeff(), 1e-12);
    const double lidar_yaw_information_ratio = std::clamp(
        lidar_yaw_information / strongest_rotation_information, 0.0, 1.0);
    if (!wall_yaw_prepared) {
      prepareWallYawConstraint(
          wall_yaw_observation, pose, lidar_yaw_information_ratio);
      wall_yaw_prepared = true;
    }
    wall_yaw_information = wall_yaw_observation.valid
        ? wall_yaw_base_information *
            wall_yaw_observation.observability_gate
        : 0.0;

    if (level_observation.valid) {
      // z 为世界竖直方向，h(R)=R*n_body。右乘姿态误差下
      // dh/d(delta_theta)=-R*hat(n_body)。该雅可比秩为二，只增加
      // roll/pitch 信息，不观测 yaw、位置或高度。
      const V3d normal_body = level_observation.normal_body.cast<double>();
      const V3d predicted_up = rotation * normal_body;
      const V3d residual = V3d::UnitZ() - predicted_up;
      const M3d H = -rotation *
          SO3::hat(level_observation.normal_body).cast<double>();
      sum_HTVH.block<3, 3>(0, 0).noalias() +=
          level_information * H.transpose() * H;
      sum_HTVr.head<3>().noalias() +=
          level_information * H.transpose() * residual;
    }
    if (wall_yaw_observation.valid && wall_yaw_information > 0.0) {
      V3d normal_world =
          rotation * wall_yaw_observation.normal_body.cast<double>();
      normal_world.z() = 0.0;
      const double normal_norm = normal_world.norm();
      if (std::isfinite(normal_norm) && normal_norm > 1e-6) {
        normal_world /= normal_norm;
        const V3d target_normal_world =
            wall_yaw_observation.target_normal_world.cast<double>();
        const double raw_residual = std::atan2(
            normal_world.x() * target_normal_world.y() -
                normal_world.y() * target_normal_world.x(),
            std::clamp(
                normal_world.dot(target_normal_world), -1.0, 1.0));
        const double residual_limit =
            g_wall_yaw_max_correction_per_frame_deg * M_PI / 180.0;
        const double residual = std::clamp(
            raw_residual, -residual_limit, residual_limit);

        // 右乘误差中 R^T*z_world 对应绕世界竖直轴的纯旋转。
        // 该 rank-1 观测不会向 XY、Z、roll 或 pitch 注入信息。
        sum_HTVH.block<3, 3>(0, 0).noalias() +=
            wall_yaw_information *
            yaw_axis_body * yaw_axis_body.transpose();
        sum_HTVr.head<3>().noalias() +=
            wall_yaw_information * yaw_axis_body * residual;
      }
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

  if (!wall_yaw_observation.extraction_attempted) {
    ++wall_yaw_extraction_skipped_count_;
  } else if (!wall_yaw_observation.valid) {
    ++wall_yaw_constraint_rejected_count_;
  } else if (wall_yaw_observation.observability_gate <= 1e-6) {
    ++wall_yaw_constraint_gated_count_;
  } else {
    ++wall_yaw_constraint_accepted_count_;
  }

  effective_match_count_ = 0;
  for (std::size_t i = 0; i < effect_knn_num_; ++i) {
    const int index = effect_knn_idxs_[i];
    if (index >= 0 && static_cast<std::size_t>(index) < effect_mask_.size() &&
        effect_mask_[index]) {
      effective_match_count_++;
    }
  }

  const auto level_log_time = std::chrono::steady_clock::now();
  if (last_level_constraint_log_time_ ==
          std::chrono::steady_clock::time_point{} ||
      level_log_time - last_level_constraint_log_time_ >
          std::chrono::seconds(5)) {
    last_level_constraint_log_time_ = level_log_time;
    const SE3 diagnostic_pose = kf_->GetSE3();
    double post_update_innovation_deg = 0.0;
    if (level_observation.valid) {
      V3 post_update_plane_up =
          diagnostic_pose.R_ * level_observation.normal_body;
      post_update_plane_up.normalize();
      post_update_innovation_deg = std::acos(std::clamp(
          static_cast<double>(post_update_plane_up.dot(V3::UnitZ())),
          -1.0, 1.0)) * 180.0 / M_PI;
    }
    double wall_post_update_innovation_deg = 0.0;
    if (wall_yaw_observation.valid) {
      V3 wall_normal_world =
          diagnostic_pose.R_ * wall_yaw_observation.normal_body;
      wall_normal_world.z() = 0.0;
      const scalar wall_normal_norm = wall_normal_world.norm();
      if (wall_normal_world.allFinite() && wall_normal_norm > 1e-6) {
        wall_normal_world /= wall_normal_norm;
        wall_post_update_innovation_deg = std::abs(std::atan2(
            wall_normal_world.x() *
                    wall_yaw_observation.target_normal_world.y() -
                wall_normal_world.y() *
                    wall_yaw_observation.target_normal_world.x(),
            std::clamp(
                static_cast<double>(wall_normal_world.dot(
                    wall_yaw_observation.target_normal_world)),
                -1.0, 1.0))) * 180.0 / M_PI;
      }
    }
    LOG(INFO) << GREEN
              << " ---> [SuperLIO]: level constraint health. current_valid="
              << level_observation.valid
              << " accepted=" << level_constraint_accepted_count_
              << " rejected=" << level_constraint_rejected_count_
              << " candidates=" << level_observation.candidate_count
              << " inliers=" << level_observation.inlier_count
              << " ratio=" << level_observation.inlier_ratio
              << " rms=" << level_observation.rms
              << " plane_gravity_angle="
              << level_observation.gravity_angle_deg
              << "deg attitude_innovation="
              << level_observation.innovation_deg
              << "deg post_update_innovation="
              << post_update_innovation_deg
              << "deg information=" << level_information
              << " lidar_rotation_information_eigenvalues="
              << lidar_rotation_information_eigenvalues.transpose()
              << " position=" << diagnostic_pose.t_.transpose() << RESET;
    LOG(INFO) << GREEN
              << " ---> [SuperLIO]: 墙面航向约束健康状态。当前有效="
              << wall_yaw_observation.valid
              << " 平面有效=" << wall_yaw_observation.plane_valid
              << " 参考有效=" << wall_yaw_observation.reference_valid
              << " 接受=" << wall_yaw_constraint_accepted_count_
              << " 拒绝=" << wall_yaw_constraint_rejected_count_
              << " 可观测性门控=" << wall_yaw_constraint_gated_count_
              << " 降频跳过=" << wall_yaw_extraction_skipped_count_
              << " 参考样本=" << wall_yaw_reference_samples_.size()
              << " 局部参考=" << wall_yaw_references_.size()
              << " 当前参考=" << wall_yaw_observation.reference_index
              << " 候选点=" << wall_yaw_observation.candidate_count
              << " 内点=" << wall_yaw_observation.inlier_count
              << " 内点比例=" << wall_yaw_observation.inlier_ratio
              << " RMS=" << wall_yaw_observation.rms
              << " 垂直角=" << wall_yaw_observation.vertical_angle_deg
              << "deg 竖直跨度=" << wall_yaw_observation.vertical_span
              << "m 水平跨度=" << wall_yaw_observation.horizontal_span
              << "m 创新=" << wall_yaw_observation.innovation_deg
              << "deg 更新后创新=" << wall_post_update_innovation_deg
              << "deg yaw相对信息强度="
              << wall_yaw_observation.lidar_yaw_information_ratio
              << " 门控权重=" << wall_yaw_observation.observability_gate
              << " 信息量=" << wall_yaw_information << RESET;
  }

  const SE3 current_pose = kf_->GetSE3();
  const SE3 predicted_pose = predicted_state.GetSE3();
  const bool pose_finite = current_pose.t_.allFinite() && current_pose.R_.allFinite();
  double frame_translation = 0.0;
  double frame_rotation_deg = 0.0;
  double predicted_translation = 0.0;
  double predicted_rotation_deg = 0.0;
  bool prediction_valid =
      predicted_pose.t_.allFinite() && predicted_pose.R_.allFinite();
  bool motion_valid = pose_finite;
  if (pose_finite && has_last_accepted_pose_) {
    frame_translation = (current_pose.t_ - last_pose_.t_).norm();
    const M3 relative_rotation = last_pose_.R_.transpose() * current_pose.R_;
    frame_rotation_deg = std::abs(
      Eigen::AngleAxis<scalar>(relative_rotation).angle() * 180.0 / M_PI);
    motion_valid = frame_translation <= g_max_frame_translation &&
                   frame_rotation_deg <= g_max_frame_rotation_deg;
  }
  if (prediction_valid && has_last_accepted_pose_) {
    predicted_translation = (predicted_pose.t_ - last_pose_.t_).norm();
    const M3 predicted_relative_rotation =
        last_pose_.R_.transpose() * predicted_pose.R_;
    predicted_rotation_deg = std::abs(
      Eigen::AngleAxis<scalar>(predicted_relative_rotation).angle() * 180.0 / M_PI);
    prediction_valid =
        predicted_translation <= g_max_frame_translation &&
        predicted_rotation_deg <= g_max_frame_rotation_deg;
  }

  observation_valid_ =
    effective_match_count_ >= static_cast<std::size_t>(g_min_effective_points) &&
    motion_valid;
  if (!observation_valid_) {
    const char* rollback_mode = "imu_prediction";
    if (prediction_valid || !has_last_accepted_state_) {
      kf_->SetX(predicted_state);
      kf_->SetCov(predicted_covariance);
    } else {
      SysState recovered_state = last_accepted_state_;
      recovered_state.timestamp = predicted_state.timestamp;
      recovered_state.v.setZero();
      kf_->SetX(recovered_state);
      kf_->SetCov(last_accepted_covariance_);
      rollback_mode = "last_accepted_state";
    }
    consecutive_invalid_observations_++;
    if (consecutive_invalid_observations_ == 1 ||
        consecutive_invalid_observations_ % 10 == 0) {
      LOG(WARNING) << YELLOW
                   << " ---> [SuperLIO]: reject unsafe map update. effective_matches="
                   << effective_match_count_ << " min=" << g_min_effective_points
                   << " frame_translation=" << frame_translation
                   << "m frame_rotation=" << frame_rotation_deg
                   << "deg predicted_translation=" << predicted_translation
                   << "m predicted_rotation=" << predicted_rotation_deg
                   << "deg finite=" << pose_finite
                   << " state_rollback=" << rollback_mode
                   << " consecutive=" << consecutive_invalid_observations_ << RESET;
    }
  } else if (consecutive_invalid_observations_ > 0) {
    LOG(INFO) << GREEN << " ---> [SuperLIO]: observation recovered after "
              << consecutive_invalid_observations_ << " rejected frames." << RESET;
    consecutive_invalid_observations_ = 0;
  }

  frame_num_++;
}

void SuperLIO::UpdateMap() {
  if (!observation_valid_) return;
  const size_t ptsize = ds_undistort_->size();
  if (ptsize == 0) return;
  
  last_pose_ = kf_->GetSE3();
  has_last_accepted_pose_ = true;
  last_accepted_state_ = kf_->GetSysState();
  last_accepted_covariance_ = kf_->GetCov();
  has_last_accepted_state_ = true;
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

  if (!observation_valid_) return;

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
