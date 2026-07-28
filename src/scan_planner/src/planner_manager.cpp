// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <plan_manage/reference_guide.h>
#include <thread>

namespace scan_planner
{
  namespace
  {
    template <typename T>
    void getParam(const rclcpp::Node::SharedPtr &node, const std::string &name, T &value, const T &default_value)
    {
      if (!node->has_parameter(name))
        node->declare_parameter<T>(name, default_value);
      node->get_parameter(name, value);
    }

    void applyLinearZReference(std::vector<Eigen::Vector3d> &points, const double start_z, const double target_z)
    {
      if (points.empty())
        return;

      if (points.size() == 1)
      {
        points.front()(2) = start_z;
        return;
      }

      std::vector<double> accumulated_xy_length(points.size(), 0.0);
      for (size_t i = 1; i < points.size(); ++i)
      {
        accumulated_xy_length[i] = accumulated_xy_length[i - 1] +
                                   (points[i].head<2>() - points[i - 1].head<2>()).norm();
      }

      const double total_xy_length = accumulated_xy_length.back();
      for (size_t i = 0; i < points.size(); ++i)
      {
        const double ratio = total_xy_length > 1e-6
                                 ? accumulated_xy_length[i] / total_xy_length
                                 : static_cast<double>(i) / static_cast<double>(points.size() - 1);
        points[i](2) = start_z + ratio * (target_z - start_z);
      }

      points.front()(2) = start_z;
      points.back()(2) = target_z;
    }
  } // namespace

  // SECTION interfaces for setup and query

  SCANPlannerManager::SCANPlannerManager() {}

  SCANPlannerManager::~SCANPlannerManager() { std::cout << "des manager" << std::endl; }

  void SCANPlannerManager::initPlanModules(const rclcpp::Node::SharedPtr &nh, PlanningVisualization::Ptr vis)
  {
    node_ = nh;

    /* read algorithm parameters */

    getParam(nh, "manager/max_vel", pp_.max_vel_, -1.0);
    getParam(nh, "manager/max_acc", pp_.max_acc_, -1.0);
    getParam(nh, "manager/max_jerk", pp_.max_jerk_, -1.0);
    getParam(nh, "optimization/vel_tolerance", pp_.vel_tolerance_, 1.0);
    getParam(nh, "optimization/acc_tolerance", pp_.acc_tolerance_, 1.0);
    getParam(nh, "manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
    getParam(nh, "manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
    getParam(nh, "manager/planning_horizon", pp_.planning_horizon_, 5.0);
    getParam(nh, "manager/planar_motion", planar_motion_, false);

    local_data_.traj_id_ = 0;
    grid_map_.reset(new GridMap);
    grid_map_->initMap(nh);

    bspline_optimizer_rebound_.reset(new BsplineOptimizer);
    bspline_optimizer_rebound_->setParam(nh);
    bspline_optimizer_rebound_->setEnvironment(grid_map_);
    bspline_optimizer_rebound_->a_star_.reset(new AStar);
    bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

    visualization_ = vis;
  }

  // !SECTION

  // SECTION rebond replanning

  bool SCANPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                        Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                        Eigen::Vector3d local_target_vel,
                                        bool flag_polyInit,
                                        bool flag_randomPolyTraj,
                                        const std::vector<Eigen::Vector3d> *reference_guide,
                                        bool allow_short_verified_recovery_leg)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    const double minimum_plan_distance =
        allow_short_verified_recovery_leg ? 0.05 : 0.20;
    if ((start_pt - local_target_pt).norm() < minimum_plan_distance)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    rclcpp::Time t_start = node_->now();
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    bool using_reference_guide = false;
    const bool have_reference_guide =
        reference_guide != nullptr && reference_guide->size() >= 2;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (have_reference_guide ||
          flag_first_call || flag_polyInit || flag_force_polynomial
          /*|| ( start_pt - local_target_pt ).norm() < 1.0*/)
      {
        flag_first_call = false;
        flag_force_polynomial = false;
        using_reference_guide = have_reference_guide;

        if (using_reference_guide)
        {
          const double max_guide_spacing =
              std::min(0.20, std::max(0.05, pp_.ctrl_pt_dist * 0.5));
          point_set = resampleReferenceGuide(
              *reference_guide,
              start_pt,
              local_target_pt,
              max_guide_spacing,
              7);

          double guide_length = 0.0;
          for (size_t index = 1; index < point_set.size(); ++index)
            guide_length +=
                (point_set[index] - point_set[index - 1]).norm();
          const double mean_spacing =
              guide_length /
              std::max<double>(1.0, point_set.size() - 1.0);
          ts = std::max(
              0.05,
              mean_spacing / std::max(0.05, pp_.max_vel_) * 1.2);

          start_end_derivatives.push_back(start_vel);
          start_end_derivatives.push_back(local_target_vel);
          start_end_derivatives.push_back(start_acc);
          start_end_derivatives.push_back(Eigen::Vector3d::Zero());
        }
        else
        {

          PolynomialTraj gl_traj;

          double dist = (start_pt - local_target_pt).norm();
          double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

          if (!flag_randomPolyTraj)
          {
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
          }
          else
          {
            Eigen::Vector3d horizon_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
            Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizon_dir)).normalized();
            Eigen::Vector3d vertical_perturbation = Eigen::Vector3d::Zero();
            if (!planar_motion_)
            {
              vertical_perturbation =
                  (((double)rand()) / RAND_MAX - 0.5) *
                  (start_pt - local_target_pt).norm() * vertical_dir * 0.4 *
                  (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
            }
            Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                 (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                                 vertical_perturbation;
            Eigen::MatrixXd pos(3, 3);
            pos.col(0) = start_pt;
            pos.col(1) = random_inserted_pt;
            pos.col(2) = local_target_pt;
            Eigen::VectorXd t(2);
            t(0) = t(1) = time / 2;
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
          }

          double t;
          bool flag_too_far;
          ts *= 1.5; // ts will be divided by 1.5 in the next
          do
          {
            ts /= 1.5;
            point_set.clear();
            flag_too_far = false;
            Eigen::Vector3d last_pt = gl_traj.evaluate(0);
            for (t = 0; t < time; t += ts)
            {
              Eigen::Vector3d pt = gl_traj.evaluate(t);
              if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
              {
                flag_too_far = true;
                break;
              }
              last_pt = pt;
              point_set.push_back(pt);
            }
          } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
          t -= ts;
          start_end_derivatives.push_back(gl_traj.evaluateVel(0));
          start_end_derivatives.push_back(local_target_vel);
          start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
          start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
        }
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();

        vector<double> pseudo_arc_length;
        vector<Eigen::Vector3d> segment_point;
        pseudo_arc_length.push_back(0.0);
        for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
        {
          segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
          if (t > t_cur)
          {
            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
          }
        }
        t -= ts;

        double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;
        if (poly_time > ts)
        {
          PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                        local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                        local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                        local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

          for (t = ts; t < poly_time; t += ts)
          {
            if (!pseudo_arc_length.empty())
            {
              segment_point.push_back(gl_traj.evaluate(t));
              pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
            }
            else
            {
              ROS_ERROR("pseudo_arc_length is empty, return!");
              continuous_failures_count_++;
              return false;
            }
          }
        }

        double sample_length = 0;
        double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
        size_t id = 0;
        do
        {
          cps_dist /= 1.5;
          point_set.clear();
          sample_length = 0;
          id = 0;
          while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
          {
            if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
            {
              point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                  (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
              sample_length += cps_dist;
            }
            else
              id++;
          }
          point_set.push_back(local_target_pt);
        } while (point_set.size() < 7); // If the start point is very close to end point, this will help

        start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
        start_end_derivatives.push_back(Eigen::Vector3d::Zero());

        if (point_set.size() > pp_.planning_horizon_ / pp_.ctrl_pt_dist * 3) // The initial path is abnormally too long!
        {
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
      }
    } while (flag_regenerate);

    // The reference path already carries the terrain-following body height.
    // Replacing it with a straight z interpolation would flatten ramps and can
    // jump between floors. Legacy endpoint planning still needs its original
    // linear z profile.
    if (!using_reference_guide)
      applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

    bool reference_is_free = using_reference_guide;
    if (reference_is_free)
    {
      const double sample_step =
          std::max(0.02, grid_map_->getResolution());
      for (size_t index = 0;
           reference_is_free && index + 1 < point_set.size();
           ++index)
      {
        const Eigen::Vector3d delta =
            point_set[index + 1] - point_set[index];
        const double length = delta.norm();
        const int samples = std::max(
            1, static_cast<int>(std::ceil(length / sample_step)));
        const double yaw = std::atan2(delta(1), delta(0));
        for (int sample = 0; sample <= samples; ++sample)
        {
          const Eigen::Vector3d point =
              point_set[index] +
              (static_cast<double>(sample) /
               static_cast<double>(samples)) *
                  delta;
          if (grid_map_->getInflateOccupancy(point, yaw) != 0)
          {
            reference_is_free = false;
            break;
          }
        }
      }
    }

    // In free space the lateral reference term prevents smoothness from
    // turning an L-shaped verified ground route into its endpoint chord. If
    // any obstacle occupies the guide, remove that term completely: local
    // A*/rebound retains full authority to leave the global reference.
    if (reference_is_free)
      bspline_optimizer_rebound_->setReboundReference(point_set);
    else
      bspline_optimizer_rebound_->clearReboundReference();

    vector<vector<Eigen::Vector3d>> a_star_paths;
    a_star_paths = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

    t_init = node_->now() - t_start;

    static int vis_id = 0;
    visualization_->displayInitPathList(point_set, 0.2, 0);
    visualization_->displayAStarList(a_star_paths, vis_id);

    t_start = node_->now();

    /*** STEP 2: OPTIMIZE ***/
    bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
    cout << "first_optimize_step_success=" << flag_step_1_success << endl;
    if (!flag_step_1_success)
    {
      // visualization_->displayOptimalList( ctrl_pts, vis_id );
      continuous_failures_count_++;
      return false;
    }
    //visualization_->displayOptimalList( ctrl_pts, vis_id );

    t_opt = node_->now() - t_start;
    t_start = node_->now();

    /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
    UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

    double ratio;
    bool flag_step_2_success = true;
    if (!pos.checkFeasibility(ratio, false))
    {
      cout << "Need to reallocate time." << endl;

      Eigen::MatrixXd optimal_control_points;
      flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
      if (flag_step_2_success)
        pos = UniformBspline(optimal_control_points, 3, ts);
    }

    if (!flag_step_2_success || !checkDynamicFeasibility(pos))
    {
      printf("\033[34mThis refined trajectory is unsafe or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      return false;
    }

    t_refine = node_->now() - t_start;

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << endl;

    // success. YoY
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::planVerifiedB2RecoveryLeg(
      const Eigen::Vector3d &start_pt,
      const Eigen::Vector3d &target_pt)
  {
    if (!start_pt.allFinite() || !target_pt.allFinite())
      return false;

    const double distance = (target_pt - start_pt).norm();
    if (distance < 0.02)
      return false;

    // A clamped cubic Bezier with P0=P1 and P2=P3 follows the verified
    // straight chord exactly and stops at both ends. The normal rebound
    // optimizer is intentionally bypassed here: smoothing a short steering
    // primitive can leave the already checked obstacle/ground corridor.
    Eigen::MatrixXd control_points(3, 4);
    control_points.col(0) = start_pt;
    control_points.col(1) = start_pt;
    control_points.col(2) = target_pt;
    control_points.col(3) = target_pt;

    const double velocity_duration =
        1.5 * distance / std::max(0.05, pp_.max_vel_);
    const double acceleration_duration =
        std::sqrt(
            6.0 * distance / std::max(0.05, pp_.max_acc_));
    const double duration =
        1.10 * std::max(
            0.40,
            std::max(velocity_duration, acceleration_duration));

    UniformBspline trajectory(control_points, 3, duration);
    Eigen::VectorXd knots(8);
    knots << 0.0, 0.0, 0.0, 0.0,
        duration, duration, duration, duration;
    trajectory.setKnot(knots);

    if (!checkDynamicFeasibility(trajectory))
      return false;

    updateTrajInfo(trajectory, node_->now());
    continuous_failures_count_ = 0;
    return true;
  }

  bool SCANPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
  {
    Eigen::MatrixXd control_points(3, 6);
    for (int i = 0; i < 6; i++)
    {
      control_points.col(i) = stop_pos;
    }

    updateTrajInfo(UniformBspline(control_points, 3, 1.0), node_->now());

    return true;
  }

  bool SCANPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                  const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    if (waypoints.empty())
      return false;

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);

    for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
    {
      points.push_back(waypoints[wp_i]);
    }

    double total_len = 0;
    for (size_t i = 0; i < points.size() - 1; i++)
    {
      total_len += (points[i + 1] - points[i]).norm();
    }

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    double dist_thresh = max(total_len / 8, 4.0);

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // for ( int i=0; i<inter_points.size(); i++ )
    // {
    //   cout << inter_points[i].transpose() << endl;
    // }

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {

    // generate global reference trajectory

    vector<Eigen::Vector3d> points;
    points.push_back(start_pos);
    points.push_back(end_pos);

    // insert intermediate points if too far
    vector<Eigen::Vector3d> inter_points;
    const double dist_thresh = 4.0;

    for (size_t i = 0; i < points.size() - 1; ++i)
    {
      inter_points.push_back(points.at(i));
      double dist = (points.at(i + 1) - points.at(i)).norm();

      if (dist > dist_thresh)
      {
        int id_num = floor(dist / dist_thresh) + 1;

        for (int j = 1; j < id_num; ++j)
        {
          Eigen::Vector3d inter_pt =
              points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
          inter_points.push_back(inter_pt);
        }
      }
    }

    inter_points.push_back(points.back());

    // write position matrix
    int pt_num = inter_points.size();
    Eigen::MatrixXd pos(3, pt_num);
    for (int i = 0; i < pt_num; ++i)
      pos.col(i) = inter_points[i];

    Eigen::Vector3d zero(0, 0, 0);
    Eigen::VectorXd time(pt_num - 1);
    for (int i = 0; i < pt_num - 1; ++i)
    {
      time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
    }

    time(0) *= 2.0;
    time(time.rows() - 1) *= 2.0;

    PolynomialTraj gl_traj;
    if (pos.cols() >= 3)
      gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
    else if (pos.cols() == 2)
      gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
    else
      return false;

    auto time_now = node_->now();
    global_data_.setGlobalTraj(gl_traj, time_now);

    return true;
  }

  bool SCANPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
  {
    double t_inc;

    Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

    // std::cout << "ratio: " << ratio << std::endl;
    reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

    traj = UniformBspline(ctrl_pts, 3, ts);

    double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
    bspline_optimizer_rebound_->ref_pts_.clear();
    for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
      bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

    bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

    return success;
  }

  void SCANPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const rclcpp::Time time_now)
  {
    local_data_.start_time_ = time_now;
    local_data_.position_traj_ = position_traj;
    local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
    local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
    local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
    local_data_.duration_ = local_data_.position_traj_.getTimeSum();
    local_data_.traj_id_ += 1;
  }

  bool SCANPlannerManager::checkDynamicFeasibility(UniformBspline position_traj)
  {
    UniformBspline vel_traj = position_traj.getDerivative();
    UniformBspline acc_traj = vel_traj.getDerivative();
    const double duration = position_traj.getTimeSum();
    const double sample_dt = std::max(0.01, std::min(0.05, duration / 50.0));
    const double vel_limit = pp_.max_vel_ + pp_.vel_tolerance_;
    const double acc_limit = pp_.max_acc_ + pp_.acc_tolerance_;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      Eigen::Vector3d vel = vel_traj.evaluateDeBoorT(tc);
      if (vel.norm() > vel_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: velocity limit exceeded at t="
                        << tc << ", |v|=" << vel.norm() << " > " << vel_limit);
        return false;
      }

      Eigen::Vector3d acc = acc_traj.evaluateDeBoorT(tc);
      if (acc.norm() > acc_limit)
      {
        ROS_WARN_STREAM("Dynamic feasibility check failed: acceleration limit exceeded at t="
                        << tc << ", |a|=" << acc.norm() << " > " << acc_limit);
        return false;
      }
    }

    return true;
  }

  void SCANPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                         Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
  {
    double time_origin = bspline.getTimeSum();
    int seg_num = bspline.getControlPoint().cols() - 3;
    // double length = bspline.getLength(0.1);
    // int seg_num = ceil(length / pp_.ctrl_pt_dist);

    bspline.lengthenTime(ratio);
    double duration = bspline.getTimeSum();
    dt = duration / double(seg_num);
    time_inc = duration - time_origin;

    vector<Eigen::Vector3d> point_set;
    for (double time = 0.0; time <= duration + 1e-4; time += dt)
    {
      point_set.push_back(bspline.evaluateDeBoorT(time));
    }
    UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
  }

} // namespace scan_planner
