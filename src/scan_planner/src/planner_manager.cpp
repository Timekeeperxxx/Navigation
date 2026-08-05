// #include <fstream>
#include <plan_manage/planner_manager.h>
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
    getParam(nh, "manager/collision_check_corridor_radius", collision_check_corridor_radius_, 0.4);
    getParam(nh, "manager/collision_check_z_tolerance", collision_check_z_tolerance_, 0.35);
    getParam(nh, "manager/collision_check_z_tolerance_down", collision_check_z_tolerance_down_,
             collision_check_z_tolerance_);
    getParam(nh, "manager/collision_check_z_tolerance_up", collision_check_z_tolerance_up_,
             collision_check_z_tolerance_);
    getParam(nh, "manager/collision_check_start_clear_radius", collision_check_start_clear_radius_, 0.5);
    collision_check_corridor_radius_ = std::max(0.0, collision_check_corridor_radius_);
    collision_check_z_tolerance_ = std::max(0.0, collision_check_z_tolerance_);
    collision_check_z_tolerance_down_ = std::max(0.0, collision_check_z_tolerance_down_);
    collision_check_z_tolerance_up_ = std::max(0.0, collision_check_z_tolerance_up_);
    collision_check_start_clear_radius_ = std::max(0.0, collision_check_start_clear_radius_);

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
                                        Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
  {

    static int count = 0;
    std::cout << endl
              << "[rebo replan]: -------------------------------------" << count++ << std::endl;
    cout.precision(3);
    cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
         << endl;

    if ((start_pt - local_target_pt).norm() < 0.2)
    {
      cout << "Close to goal" << endl;
      continuous_failures_count_++;
      return false;
    }

    const rclcpp::Time t_total_start = node_->now();
    rclcpp::Time t_start = t_total_start;
    rclcpp::Duration t_init(0, 0), t_opt(0, 0), t_refine(0, 0);
    rclcpp::Duration t_feasibility(0, 0), t_collision(0, 0);

    /*** STEP 1: INIT ***/
    double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
    if (!std::isfinite(ts) || ts <= 1e-4)
    {
      ROS_ERROR("Invalid B-spline initialization sample time ts=%f (ctrl_dist=%f max_vel=%f).",
                ts, pp_.ctrl_pt_dist, pp_.max_vel_);
      continuous_failures_count_++;
      return false;
    }
    vector<Eigen::Vector3d> point_set, start_end_derivatives;
    static bool flag_first_call = true, flag_force_polynomial = false;
    bool flag_regenerate = false;
    do
    {
      point_set.clear();
      start_end_derivatives.clear();
      flag_regenerate = false;

      if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
      {
        flag_first_call = false;
        flag_force_polynomial = false;

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
          Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizon_dir * 0.8 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989) +
                                               (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continuous_failures_count_ + 0.989) + 0.989);
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
        int polynomial_resample_attempts = 0;
        do
        {
          polynomial_resample_attempts++;
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
        } while ((flag_too_far || point_set.size() < 7) &&
                 polynomial_resample_attempts < 48); // To make sure the initial path has enough points.
        if (flag_too_far || point_set.size() < 7)
        {
          ROS_ERROR("Unable to sample polynomial initial path after %d attempts: points=%zu ts=%f too_far=%d.",
                    polynomial_resample_attempts, point_set.size(), ts, flag_too_far);
          continuous_failures_count_++;
          return false;
        }
        t -= ts;
        start_end_derivatives.push_back(gl_traj.evaluateVel(0));
        start_end_derivatives.push_back(local_target_vel);
        start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
        start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
      }
      else // Initial path generated from previous trajectory.
      {

        double t;
        double t_cur = (node_->now() - local_data_.start_time_).seconds();
        t_cur = std::max(0.0, std::min(t_cur, local_data_.duration_));

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

        // EmergencyStop() creates a stationary B-spline whose remaining arc
        // length is exactly zero. Reusing that trajectory as an initial path
        // can make the legacy arc-length resampler spin forever. A stopped
        // trajectory carries no useful geometric information, so restart from
        // a fresh polynomial at the frozen robot pose.
        const double remaining_arc_length =
            pseudo_arc_length.empty() ? 0.0 : pseudo_arc_length.back();
        const double minimum_reusable_arc_length =
            std::max(1e-3, pp_.ctrl_pt_dist * 0.05);
        if (pseudo_arc_length.size() < 2 ||
            segment_point.size() < 2 ||
            !std::isfinite(remaining_arc_length) ||
            remaining_arc_length < minimum_reusable_arc_length)
        {
          ROS_WARN("[SCAN replan] Remaining trajectory is degenerate: samples=%zu arc_length=%.6f threshold=%.6f; regenerate polynomial initial path.",
                   segment_point.size(), remaining_arc_length, minimum_reusable_arc_length);
          flag_force_polynomial = true;
          flag_regenerate = true;
          continue;
        }

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

        if (pseudo_arc_length.size() < 2 || segment_point.size() < 2 ||
            !std::isfinite(pseudo_arc_length.back()) ||
            pseudo_arc_length.back() < minimum_reusable_arc_length)
        {
          ROS_WARN("Previous trajectory has too few remaining samples; regenerate polynomial initial path.");
          flag_force_polynomial = true;
          flag_regenerate = true;
        }
        else
        {
          double sample_length = 0;
          double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next
          size_t id = 0;
          int previous_resample_attempts = 0;
          do
          {
            previous_resample_attempts++;
            cps_dist /= 1.5;
            point_set.clear();
            sample_length = 0;
            id = 0;
            while ((id + 1 < pseudo_arc_length.size()) && sample_length <= pseudo_arc_length.back())
            {
              const double segment_len = pseudo_arc_length[id + 1] - pseudo_arc_length[id];
              if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1] && segment_len > 1e-6)
              {
                point_set.push_back((sample_length - pseudo_arc_length[id]) / segment_len * segment_point[id + 1] +
                                    (pseudo_arc_length[id + 1] - sample_length) / segment_len * segment_point[id]);
                sample_length += cps_dist;
              }
              else
                id++;
            }
            point_set.push_back(local_target_pt);
          } while (point_set.size() < 7 && previous_resample_attempts < 48); // If the start point is very close to end point, this will help

          if (point_set.size() < 7)
          {
            ROS_WARN("[SCAN replan] Previous-trajectory resampling did not converge after %d attempts: points=%zu arc_length=%.6f; regenerate polynomial initial path.",
                     previous_resample_attempts, point_set.size(), pseudo_arc_length.back());
            flag_force_polynomial = true;
            flag_regenerate = true;
            continue;
          }

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
      }
    } while (flag_regenerate);

    applyLinearZReference(point_set, start_pt(2), local_target_pt(2));

    Eigen::MatrixXd ctrl_pts;
    UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

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
      RCLCPP_WARN(
          node_->get_logger(),
          "[SCAN timing] reboundReplan failed stage=optimize total=%.3f init=%.3f optimize=%.3f "
          "start=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f) poly_init=%d random_poly=%d.",
          (node_->now() - t_total_start).seconds(),
          t_init.seconds(),
          (node_->now() - t_start).seconds(),
          start_pt(0), start_pt(1), start_pt(2),
          local_target_pt(0), local_target_pt(1), local_target_pt(2),
          flag_polyInit,
          flag_randomPolyTraj);
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

    bool dynamic_feasible = false;
    bool collision_free = false;
    if (flag_step_2_success)
    {
      const rclcpp::Time t_feasibility_start = node_->now();
      dynamic_feasible = checkDynamicFeasibility(pos);
      t_feasibility = node_->now() - t_feasibility_start;

      if (dynamic_feasible)
      {
        const rclcpp::Time t_collision_start = node_->now();
        collision_free = checkCollisionFree(pos);
        t_collision = node_->now() - t_collision_start;
      }
    }

    if (!flag_step_2_success || !dynamic_feasible || !collision_free)
    {
      printf("\033[34mThis refined trajectory is unsafe, colliding, or dynamically infeasible. Skip publishing it.\n\033[0m");
      continuous_failures_count_++;
      RCLCPP_WARN(
          node_->get_logger(),
          "[SCAN timing] reboundReplan failed stage=validate total=%.3f init=%.3f optimize=%.3f refine_validate=%.3f "
          "feasibility=%.3f collision=%.3f refine_success=%d dynamic_feasible=%d collision_free=%d "
          "start=(%.2f,%.2f,%.2f) target=(%.2f,%.2f,%.2f).",
          (node_->now() - t_total_start).seconds(),
          t_init.seconds(),
          t_opt.seconds(),
          (node_->now() - t_start).seconds(),
          t_feasibility.seconds(),
          t_collision.seconds(),
          flag_step_2_success,
          dynamic_feasible,
          collision_free,
          start_pt(0), start_pt(1), start_pt(2),
          local_target_pt(0), local_target_pt(1), local_target_pt(2));
      return false;
    }

    t_refine = node_->now() - t_start;

    // save planned results
    updateTrajInfo(pos, node_->now());

    cout << "total time:\033[42m" << (t_init + t_opt + t_refine).seconds() << "\033[0m,optimize:" << (t_init + t_opt).seconds() << ",refine:" << t_refine.seconds() << endl;
    RCLCPP_INFO(
        node_->get_logger(),
        "[SCAN timing] reboundReplan success total=%.3f init=%.3f optimize=%.3f refine_validate=%.3f "
        "feasibility=%.3f collision=%.3f duration=%.3f ctrl_pts=%ld start=(%.2f,%.2f,%.2f) "
        "target=(%.2f,%.2f,%.2f) poly_init=%d random_poly=%d failures_before=%d.",
        (node_->now() - t_total_start).seconds(),
        t_init.seconds(),
        t_opt.seconds(),
        t_refine.seconds(),
        t_feasibility.seconds(),
        t_collision.seconds(),
        local_data_.duration_,
        pos.getControlPoint().cols(),
        start_pt(0), start_pt(1), start_pt(2),
        local_target_pt(0), local_target_pt(1), local_target_pt(2),
        flag_polyInit,
        flag_randomPolyTraj,
        continuous_failures_count_);

    // success. YoY
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
    const rclcpp::Time t_total_start = node_->now();

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
    RCLCPP_INFO(
        node_->get_logger(),
        "[SCAN timing] planGlobalTrajWaypoints success total=%.3f input_waypoints=%zu inter_points=%d duration=%.3f "
        "start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f).",
        (node_->now() - t_total_start).seconds(),
        waypoints.size(),
        pt_num,
        global_data_.global_duration_,
        start_pos(0), start_pos(1), start_pos(2),
        waypoints.back()(0), waypoints.back()(1), waypoints.back()(2));

    return true;
  }

  bool SCANPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                         const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
  {
    const rclcpp::Time t_total_start = node_->now();

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
    RCLCPP_INFO(
        node_->get_logger(),
        "[SCAN timing] planGlobalTraj success total=%.3f inter_points=%d duration=%.3f "
        "start=(%.2f,%.2f,%.2f) end=(%.2f,%.2f,%.2f).",
        (node_->now() - t_total_start).seconds(),
        pt_num,
        global_data_.global_duration_,
        start_pos(0), start_pos(1), start_pos(2),
        end_pos(0), end_pos(1), end_pos(2));

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

  bool SCANPlannerManager::checkCollisionFree(UniformBspline position_traj)
  {
    if (!grid_map_)
      return true;

    const double duration = position_traj.getTimeSum();
    if (duration <= 1e-6)
      return true;

    const double resolution = std::max(0.02, grid_map_->getResolution());
    const double sample_dt = std::max(0.01, std::min(0.05, duration / std::max(1.0, position_traj.getLength(0.05) / resolution)));
    const Eigen::Vector3d start_pos = position_traj.evaluateDeBoorT(0.0);
    int checked_centers = 0;
    int skipped_start_centers = 0;
    int skipped_start_occupied_centers = 0;
    int checked_samples = 0;

    for (double t = 0.0; t < duration + 1e-6; t += sample_dt)
    {
      const double tc = std::min(t, duration);
      const double tn = std::min(tc + sample_dt, duration);
      const Eigen::Vector3d pos = position_traj.evaluateDeBoorT(tc);
      const Eigen::Vector3d pos_next = position_traj.evaluateDeBoorT(tn);
      const double start_distance_xy = (pos.head<2>() - start_pos.head<2>()).norm();
      const Eigen::Vector2d diff = pos_next.head<2>() - pos.head<2>();
      const double yaw = diff.squaredNorm() > 1e-8 ? std::atan2(diff(1), diff(0)) : 0.0;

      if (start_distance_xy <= collision_check_start_clear_radius_)
      {
        skipped_start_centers++;
        const int start_occ = grid_map_->getInflateOccupancy(pos, yaw);
        if (start_occ != 0)
        {
          skipped_start_occupied_centers++;
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(),
              *node_->get_clock(),
              1000,
              "Corridor collision diagnostic: skipped start-clear sample is occupied "
              "but ignored before publishing. region=start_near occ=%d t=%.3f "
              "start_distance_xy=%.3f start_clear_radius=%.3f pos=(%.3f, %.3f, %.3f) yaw=%.3f.",
              start_occ,
              tc,
              start_distance_xy,
              collision_check_start_clear_radius_,
              pos(0), pos(1), pos(2),
              yaw);
        }
        continue;
      }
      checked_centers++;

      const Eigen::Vector3d lateral(-std::sin(yaw), std::cos(yaw), 0.0);
      const double lateral_step = std::max(resolution, 0.05);
      const double z_step = std::max(resolution, 0.05);

      for (double lateral_offset = -collision_check_corridor_radius_;
           lateral_offset <= collision_check_corridor_radius_ + 1e-6;
           lateral_offset += lateral_step)
      {
        for (double z_offset = -collision_check_z_tolerance_down_;
             z_offset <= collision_check_z_tolerance_up_ + 1e-6;
             z_offset += z_step)
        {
          Eigen::Vector3d sample = pos + lateral * lateral_offset;
          sample(2) += z_offset;
          checked_samples++;

          const int occ = grid_map_->getInflateOccupancy(sample, yaw);
          if (occ != 0)
          {
            ROS_WARN_STREAM("Corridor collision check failed before publishing: t="
                            << tc << ", center=" << pos.transpose()
                            << ", sample=" << sample.transpose()
                            << ", region=path_ahead"
                            << ", occ=" << occ
                            << ", start_distance_xy=" << start_distance_xy
                            << ", corridor_radius=" << collision_check_corridor_radius_
                            << ", z_tolerance_down=" << collision_check_z_tolerance_down_
                            << ", z_tolerance_up=" << collision_check_z_tolerance_up_
                            << ", start_clear_radius=" << collision_check_start_clear_radius_);
            return false;
          }
        }
      }
    }

    RCLCPP_INFO_THROTTLE(
        node_->get_logger(),
        *node_->get_clock(),
        1000,
        "Corridor collision check passed before publishing: duration=%.3f, sample_dt=%.3f, "
        "checked_centers=%d, skipped_start_centers=%d, skipped_start_occupied_centers=%d, checked_samples=%d, "
        "corridor_radius=%.3f, z_tolerance_down=%.3f, z_tolerance_up=%.3f, "
        "start_clear_radius=%.3f, resolution=%.3f.",
        duration,
        sample_dt,
        checked_centers,
        skipped_start_centers,
        skipped_start_occupied_centers,
        checked_samples,
        collision_check_corridor_radius_,
        collision_check_z_tolerance_down_,
        collision_check_z_tolerance_up_,
        collision_check_start_clear_radius_,
        resolution);

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
