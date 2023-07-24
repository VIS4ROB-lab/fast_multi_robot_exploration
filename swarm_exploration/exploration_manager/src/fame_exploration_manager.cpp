// #include <fstream>
#include <exploration_manager/fame_exploration_manager.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <boost/filesystem.hpp>
#include <active_perception/graph_node.h>
#include <active_perception/graph_search.h>
#include <active_perception/perception_utils.h>
#include <active_perception/frontier_finder.h>
#include <active_perception/hgrid.h>
#include <plan_env/raycast.h>
#include <plan_env/sdf_map.h>
#include <plan_env/edt_environment.h>
#include <plan_manage/planner_manager.h>
#include <lkh_tsp_solver/SolveTSP.h>
#include <lkh_mtsp_solver/SolveMTSP.h>

#include <exploration_manager/expl_data.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

using namespace Eigen;

namespace fast_planner {
// SECTION interfaces for setup and query

FameExplorationManager::FameExplorationManager() {
}

FameExplorationManager::~FameExplorationManager() {
  ViewNode::astar_.reset();
  ViewNode::caster_.reset();
  ViewNode::map_.reset();
}

void FameExplorationManager::initialize(ros::NodeHandle& nh) {
  planner_manager_.reset(new FastPlannerManager);
  planner_manager_->initPlanModules(nh);

  edt_environment_ = planner_manager_->edt_environment_;
  sdf_map_ = edt_environment_->sdf_map_;
  frontier_finder_.reset(new FrontierFinder(edt_environment_, nh));
  // uniform_grid_.reset(new UniformGrid(edt_environment_, nh));
  hgrid_.reset(new HGrid(edt_environment_, nh));
  role_assigner_.reset(new RoleAssigner(nh));
  // view_finder_.reset(new ViewFinder(edt_environment_, nh));

  ed_.reset(new ExplorationData);
  ep_.reset(new ExplorationParam);

  nh.param("exploration/refine_local", ep_->refine_local_, true);
  nh.param("exploration/refined_num", ep_->refined_num_, -1);
  nh.param("exploration/refined_radius", ep_->refined_radius_, -1.0);
  nh.param("exploration/top_view_num", ep_->top_view_num_, -1);
  nh.param("exploration/max_decay", ep_->max_decay_, -1.0);
  nh.param("exploration/tsp_dir", ep_->tsp_dir_, string("null"));
  nh.param("exploration/mtsp_dir", ep_->mtsp_dir_, string("null"));
  nh.param("exploration/relax_time", ep_->relax_time_, 1.0);
  nh.param("exploration/drone_num", ep_->drone_num_, 1);
  nh.param("exploration/drone_id", ep_->drone_id_, 1);
  nh.param("exploration/init_plan_num", ep_->init_plan_num_, 2);

  // Explorer Parameters
  explorer_params_.reset(new ExplorerParams);
  nh.param("explorer/ftr_max_distance", explorer_params_->ftr_max_distance, 15.0);
  nh.param("explorer/max_ang_dist", explorer_params_->max_ang_dist, M_PI / 4.0);
  nh.param("explorer/label_penalty", explorer_params_->label_penalty, 10.0);
  nh.param("explorer/w_distance", explorer_params_->w_distance, 1.0);
  nh.param("explorer/w_direction", explorer_params_->w_direction, 1.0);
  nh.param("explorer/w_others", explorer_params_->w_others, 1.0);
  nh.param("explorer/w_previous_goal", explorer_params_->w_previous_goal, 1.0);

  // Collector Parameters
  collector_params_.reset(new CollectorParams);
  nh.param("collector/min_vel", collector_params_->min_vel, 1.0);
  nh.param("collector/label_penalty", collector_params_->label_penalty, 15.0);
  nh.param("collector/velocity_factor", collector_params_->velocity_factor, 2.0);
  nh.param("collector/w_distance", collector_params_->w_distance, 1.0);
  nh.param("collector/w_direction", collector_params_->w_direction, 1.0);
  nh.param("collector/w_others", collector_params_->w_others, 1.0);
  nh.param("collector/w_previous_goal", collector_params_->w_previous_goal, 1.0);

  // Use the same parameter as in role assigner
  nh.param("role_assigner/region_size", collector_params_->ftr_max_distance, 8.0);

  // Potential Field Parameters
  pf_params_.reset(new PotentialFieldParams);
  nh.param("potential_field/ka", pf_params_->ka, 1.0);
  nh.param("potential_field/kr", pf_params_->kr, 1.0);
  nh.param("potential_field/d0", pf_params_->d0, 10.0);
  nh.param("potential_field/df", pf_params_->df, 12.0);
  nh.param("potential_field/dc", pf_params_->dc, 5.0);

  assert(pf_params_->ka >= 0.);
  assert(pf_params_->kr >= 0.);

  // Initial role
  role_ = ROLE::UNKNOWN;

  ed_->swarm_state_.resize(ep_->drone_num_);
  ed_->pair_opt_stamps_.resize(ep_->drone_num_);
  ed_->pair_opt_res_stamps_.resize(ep_->drone_num_);
  for (int i = 0; i < ep_->drone_num_; ++i) {
    ed_->swarm_state_[i].stamp_ = 0.0;
    ed_->pair_opt_stamps_[i] = 0.0;
    ed_->pair_opt_res_stamps_[i] = 0.0;
  }
  planner_manager_->swarm_traj_data_.init(ep_->drone_id_, ep_->drone_num_);

  nh.param("exploration/vm", ViewNode::vm_, -1.0);
  nh.param("exploration/am", ViewNode::am_, -1.0);
  nh.param("exploration/yd", ViewNode::yd_, -1.0);
  nh.param("exploration/ydd", ViewNode::ydd_, -1.0);
  nh.param("exploration/w_dir", ViewNode::w_dir_, -1.0);

  ViewNode::astar_.reset(new Astar);
  ViewNode::astar_->init(nh, edt_environment_);
  ViewNode::map_ = sdf_map_;

  double resolution_ = sdf_map_->getResolution();
  Eigen::Vector3d origin, size;
  sdf_map_->getRegion(origin, size);
  ViewNode::caster_.reset(new RayCaster);
  ViewNode::caster_->setParams(resolution_, origin);

  planner_manager_->path_finder_->lambda_heu_ = 1.0;
  // planner_manager_->path_finder_->max_search_time_ = 0.05;
  planner_manager_->path_finder_->max_search_time_ = 1.0;

  tsp_client_ =
      nh.serviceClient<lkh_mtsp_solver::SolveMTSP>("/solve_tsp_" + to_string(ep_->drone_id_), true);
  acvrp_client_ = nh.serviceClient<lkh_mtsp_solver::SolveMTSP>(
      "/solve_acvrp_" + to_string(ep_->drone_id_), true);

  // Swarm
  for (auto& state : ed_->swarm_state_) {
    state.stamp_ = 0.0;
    state.recent_interact_time_ = 0.0;
    state.recent_attempt_time_ = 0.0;
  }
  ed_->last_grid_ids_ = {};
  ed_->reallocated_ = true;
  ed_->pair_opt_stamp_ = 0.0;
  ed_->wait_response_ = false;
  ed_->plan_num_ = 0;

  // Analysis
  // ofstream fout;
  // fout.open("/home/boboyu/Desktop/RAL_Time/frontier.txt");
  // fout.close();
}

int FameExplorationManager::planExploreMotion(
    const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw) {
  ros::Time t1 = ros::Time::now();
  auto t2 = t1;

  std::cout << "start pos: " << pos.transpose() << ", vel: " << vel.transpose()
            << ", acc: " << acc.transpose() << std::endl;

  Vector3d next_pos;
  double next_yaw;

  auto start_time = chrono::high_resolution_clock::now();

  const ROLE updated_role = role_assigner_->assignRole(
      pos, ep_->drone_id_, ed_->swarm_state_, frontier_finder_->getFrontiers());
  updateRoleAndVelocities(updated_role);

  auto end_time = chrono::high_resolution_clock::now();
  int elapsed_time = chrono::duration_cast<chrono::milliseconds>(end_time - start_time).count();
  ROS_INFO_STREAM("Time to assign role: " << elapsed_time << " ms");
  ROS_INFO("Current role: %s", roleToString(role_).c_str());

  bool success;
  if (role_ == ROLE::EXPLORER) {
    success = explorerPlan(pos, vel, yaw, next_pos, next_yaw);
  } else if (role_ == ROLE::GARBAGE_COLLECTOR) {
    success = collectorPlan(pos, vel, yaw, next_pos, next_yaw);
  }

  // If we haven't found a goal, then get the first viewpoint (this happens
  // when the goal is farther away)
  if (!success) {
    // ROS_ERROR("Falling back to closest greedy frontier");
    success = closestGreedyFrontier(pos, yaw, next_pos, next_yaw);
  }

  // Update goal
  if (success) {
    std::cout << "Next view: " << next_pos.transpose() << ", " << next_yaw << std::endl;

    // Check if we have been trying to access them too often
    const double kMinDistGoals = 0.2;
    if ((next_pos - ed_->next_pos_).norm() < kMinDistGoals) {
      ++ed_->num_attempts_;
    } else {
      ed_->num_attempts_ = 0;
    }

    const size_t kMaxAttempts = 10;
    if (ed_->num_attempts_ > kMaxAttempts) {
      bool force_different = true;
      closestGreedyFrontier(pos, yaw, next_pos, next_yaw, force_different);
    }

    ed_->next_pos_ = next_pos;
    ed_->next_yaw_ = next_yaw;

  } else {
    next_pos = ed_->next_pos_;
    next_yaw = ed_->next_yaw_;
    return FAIL;
  }

  if (planTrajToView(pos, vel, acc, yaw, next_pos, next_yaw) == FAIL) {
    return FAIL;
  }

  double total = (ros::Time::now() - t2).toSec();
  ROS_INFO("Total time: %lf", total);
  ROS_ERROR_COND(total > 0.1, "Total time too long!!!");

  return SUCCEED;
}

bool FameExplorationManager::isPositionReachable(const Vector3d& from, const Vector3d& to) const {
  /*if (sdf_map_->getInflateOccupancy(to) != SDFMap::OCCUPANCY::FREE) {
    // End position in occupied or unknown space
    return false;
  } else if (planner_manager_->path_finder_->search(from, to, false) != Astar::REACH_END) {
    // Dynamics-aware path finder doesn't find a solution
    return false;
  }*/

  if (!ViewNode::validPathExists(from, to)) {
    // Discreate A* cannot find a solution
    return false;
  } else {
    // Position is cool
    return true;
  }
}

int FameExplorationManager::planTrajToView(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& acc, const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw) {

  // Plan trajectory (position and yaw) to the next viewpoint
  auto t1 = ros::Time::now();

  // Compute time lower bound of yaw and use in trajectory generation
  double diff0 = next_yaw - yaw[0];
  double diff1 = fabs(diff0);
  double time_lb = min(diff1, 2 * M_PI - diff1) / ViewNode::yd_;

  // Generate trajectory of x,y,z
  bool goal_unknown = (edt_environment_->sdf_map_->getOccupancy(next_pos) == SDFMap::UNKNOWN);
  // bool start_unknown = (edt_environment_->sdf_map_->getOccupancy(pos) == SDFMap::UNKNOWN);
  bool optimistic = ed_->plan_num_ < ep_->init_plan_num_;
  planner_manager_->path_finder_->reset();
  if (planner_manager_->path_finder_->search(pos, next_pos, optimistic) != Astar::REACH_END) {
    ROS_ERROR("No path to next viewpoint");
    return FAIL;
  }
  ed_->path_next_goal_ = planner_manager_->path_finder_->getPath();
  shortenPath(ed_->path_next_goal_);
  ed_->kino_path_.clear();

  const double radius_far = 7.0;
  const double radius_close = 1.5;
  const double len = Astar::pathLength(ed_->path_next_goal_);
  if (len < radius_close || optimistic) {
    // Next viewpoint is very close, no need to search kinodynamic path, just use waypoints-based
    // optimization
    planner_manager_->planExploreTraj(ed_->path_next_goal_, vel, acc, time_lb);
    ed_->next_goal_ = next_pos;
    // std::cout << "Close goal." << std::endl;
    if (ed_->plan_num_ < ep_->init_plan_num_) {
      ed_->plan_num_++;
      ROS_WARN("init plan.");
    }
  } else if (len > radius_far) {
    // Next viewpoint is far away, select intermediate goal on geometric path (this also deal with
    // dead end)
    // std::cout << "Far goal." << std::endl;
    double len2 = 0.0;
    vector<Eigen::Vector3d> truncated_path = { ed_->path_next_goal_.front() };
    for (int i = 1; i < ed_->path_next_goal_.size() && len2 < radius_far; ++i) {
      auto cur_pt = ed_->path_next_goal_[i];
      len2 += (cur_pt - truncated_path.back()).norm();
      truncated_path.push_back(cur_pt);
    }
    ed_->next_goal_ = truncated_path.back();
    planner_manager_->planExploreTraj(truncated_path, vel, acc, time_lb);
  } else {
    // Search kino path to exactly next viewpoint and optimize
    // std::cout << "Mid goal" << std::endl;
    ed_->next_goal_ = next_pos;

    if (!planner_manager_->kinodynamicReplan(
            pos, vel, acc, ed_->next_goal_, Vector3d(0, 0, 0), time_lb))
      return FAIL;
    ed_->kino_path_ = planner_manager_->kino_path_finder_->getKinoTraj(0.02);
  }

  if (planner_manager_->local_data_.position_traj_.getTimeSum() < time_lb - 0.5)
    ROS_ERROR("Lower bound not satified!");

  double traj_plan_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  planner_manager_->planYawExplore(yaw, next_yaw, true, ep_->relax_time_);
  double yaw_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Traj: %lf, yaw: %lf", traj_plan_time, yaw_time);

  return SUCCEED;
}

int FameExplorationManager::updateFrontierStruct(
    const Eigen::Vector3d& pos, double yaw, const Eigen::Vector3d& vel) {

  auto t1 = ros::Time::now();
  auto t2 = t1;
  ed_->views_.clear();

  // Search frontiers and group them into clusters
  frontier_finder_->searchFrontiers();
  double frontier_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Find viewpoints (x,y,z,yaw) for all clusters; find the informative ones
  frontier_finder_->computeFrontiersToVisit();

  // Binary classification of frontiers
  frontier_finder_->binaryClassify();

  // Get frontiers in front
  frontier_finder_->updateFrontiersInFront(
      pos, vel, yaw, explorer_params_->ftr_max_distance, explorer_params_->max_ang_dist);

  // Retrieve the updated info
  frontier_finder_->getFrontiers(ed_->frontiers_);
  frontier_finder_->getFrontiersIds(ed_->fronters_ids_);
  frontier_finder_->getDormantFrontiers(ed_->dead_frontiers_);
  frontier_finder_->getFrontierBoxes(ed_->frontier_boxes_);
  frontier_finder_->getLabeledFrontiers(ed_->labeled_frontiers_);
  frontier_finder_->getInFrontFrontiers(ed_->infront_frontiers_);

  frontier_finder_->getTopViewpointsInfo(pos, ed_->points_, ed_->yaws_, ed_->averages_);
  for (int i = 0; i < ed_->points_.size(); ++i)
    ed_->views_.push_back(
        ed_->points_[i] + 2.0 * Vector3d(cos(ed_->yaws_[i]), sin(ed_->yaws_[i]), 0));

  if (ed_->frontiers_.empty()) {
    ROS_WARN("No coverable frontier.");
    return 0;
  }

  double view_time = (ros::Time::now() - t1).toSec();

  t1 = ros::Time::now();
  frontier_finder_->updateFrontierCostMatrix();

  double mat_time = (ros::Time::now() - t1).toSec();
  double total_time = frontier_time + view_time + mat_time;
  ROS_INFO("Drone %d: frontier t: %lf, viewpoint t: %lf, mat: %lf", ep_->drone_id_, frontier_time,
      view_time, mat_time);

  ROS_INFO("Total t: %lf", (ros::Time::now() - t2).toSec());
  return ed_->frontiers_.size();
}

void FameExplorationManager::updateRoleAndVelocities(const ROLE updated_role) {
  // Update velocities
  const double inv_factor = 1. / collector_params_->velocity_factor;

  if ((role_ == ROLE::EXPLORER || role_ == ROLE::UNKNOWN) &&
      updated_role == ROLE::GARBAGE_COLLECTOR) {
    // Explorer / Unknown -> GC
    planner_manager_->pp_.max_vel_ *= collector_params_->velocity_factor;
    double curr_vmax = planner_manager_->getKinodynamicAstarVMax();
    planner_manager_->setKinodynamicAstarVMax(collector_params_->velocity_factor * curr_vmax);
    curr_vmax = planner_manager_->getMaxVelBsplineOpt();
    planner_manager_->setMaxVelBsplineOpt(collector_params_->velocity_factor * curr_vmax);

    planner_manager_->pp_.max_acc_ *= collector_params_->velocity_factor;
    double curr_amax = planner_manager_->getKinodynamicAstarAMax();
    planner_manager_->setKinodynamicAstarAMax(collector_params_->velocity_factor * curr_amax);
    curr_amax = planner_manager_->getMaxAccBsplineOpt();
    planner_manager_->setMaxAccBsplineOpt(collector_params_->velocity_factor * curr_amax);

    planner_manager_->pp_.accept_vel_ = planner_manager_->pp_.max_vel_ + 0.5;
    planner_manager_->pp_.accept_acc_ = planner_manager_->pp_.max_acc_ + 0.5;

    ViewNode::vm_ *= collector_params_->velocity_factor;
    ViewNode::am_ *= collector_params_->velocity_factor;
  } else if (role_ == ROLE::GARBAGE_COLLECTOR && updated_role == ROLE::EXPLORER) {
    // GC -> Explorer
    planner_manager_->pp_.max_vel_ *= inv_factor;
    double curr_vmax = planner_manager_->getKinodynamicAstarVMax();
    planner_manager_->setKinodynamicAstarVMax(inv_factor * curr_vmax);
    curr_vmax = planner_manager_->getMaxVelBsplineOpt();
    planner_manager_->setMaxVelBsplineOpt(inv_factor * curr_vmax);

    planner_manager_->pp_.max_acc_ *= inv_factor;
    double curr_amax = planner_manager_->getKinodynamicAstarAMax();
    planner_manager_->setKinodynamicAstarAMax(inv_factor * curr_amax);
    curr_amax = planner_manager_->getMaxAccBsplineOpt();
    planner_manager_->setMaxAccBsplineOpt(inv_factor * curr_amax);

    planner_manager_->pp_.accept_vel_ = planner_manager_->pp_.max_vel_ + 0.5;
    planner_manager_->pp_.accept_acc_ = planner_manager_->pp_.max_acc_ + 0.5;

    ViewNode::vm_ *= inv_factor;
    ViewNode::am_ *= inv_factor;
  }

  // Update role
  role_ = updated_role;
}

bool FameExplorationManager::explorerPlan(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

  // If don't have frontiers in front, then go the closest frontier
  if (ed_->infront_frontiers_.empty()) {
    // ROS_ERROR("No frontal frontiers - back to greedy");
    return findPathClosestFrontier(pos, vel, yaw, next_pos, next_yaw);
  }

  // Iterate over frontal frontiers
  auto distanceCost = [&](const Vector3d& target_pos) {
    std::vector<Vector3d> path;
    return explorer_params_->w_distance * ViewNode::searchPath(pos, target_pos, path);
  };

  auto angularCost = [&](const Vector3d& target_pos) {
    Vector3d direction(target_pos - pos);
    return explorer_params_->w_direction *
           vel.head(2).normalized().dot(direction.head(2).normalized());
  };

  auto labelCost = [&](LABEL label) {
    if (role_ == ROLE::EXPLORER && label == LABEL::TRAIL)
      return explorer_params_->label_penalty;
    else if (role_ == ROLE::GARBAGE_COLLECTOR && label == LABEL::FRONTIER)
      return explorer_params_->label_penalty;
    else
      return 0.0;
  };

  auto totalCost = [&](const Vector3d& target_pos, LABEL label, bool verbose = true) {
    double distance_cost = distanceCost(target_pos);
    double angular_cost = angularCost(target_pos);
    double label_cost = labelCost(label);
    double formation_cost = explorer_params_->w_others * formationCost(target_pos);
    double previous_goal_cost = explorer_params_->w_previous_goal * previousGoalCost(target_pos);

    if (verbose) {
      std::cout << std::endl;
      std::cout << "Distance cost: " << distance_cost << std::endl;
      std::cout << "Angular cost: " << angular_cost << std::endl;
      std::cout << "Label cost: " << label_cost << std::endl;
      std::cout << "Formation cost: " << formation_cost << std::endl;
      std::cout << "Previous goal cost: " << previous_goal_cost << std::endl;
      std::cout << std::endl;
    }

    return distance_cost + angular_cost + label_cost + formation_cost + previous_goal_cost;
  };

  double min_cost = std::numeric_limits<double>::max();
  bool found_goal = false;
  for (const auto& ftr : ed_->infront_frontiers_) {
    // Re-compute centroid
    // FIXME This has already been calculated
    Eigen::Vector3d centroid(Eigen::Vector3d::Zero());
    for (const auto& cell : ftr.second) centroid += cell;
    centroid /= ftr.second.size();

    // Check that the position is valid
    if (!isPositionReachable(pos, centroid)) {
      continue;
    }

    // Total cost
    double total_cost = totalCost(centroid, ftr.first);

    if (total_cost < min_cost) {

      min_cost = total_cost;
      // Compute next position
      Vector3d diff_vec = centroid - pos;
      Vector3d direction = diff_vec.normalized();
      next_yaw = atan2(direction.y(), direction.x());

      if (diff_vec.head(2).norm() < explorer_params_->ftr_max_distance &&
          diff_vec.head(2).norm() >= 0.5) {
        next_pos = centroid;
      } else {

        double range = std::min(explorer_params_->ftr_max_distance, diff_vec.norm());
        next_pos = pos + range * direction;
        while (sdf_map_->getOccupancy(next_pos) != SDFMap::OCCUPANCY::FREE && range >= 0.5) {
          range -= 0.5;
          next_pos = pos + range * direction;
        }
      }

      // Found a path
      found_goal = true;
    }
  }

  return found_goal;

  // If we haven't found a goal, then go to greedy approach
  // if (!found_goal) {
  //   //ROS_ERROR("Goal not found - back to greedy");
  //   return findPathClosestFrontier(pos, vel, yaw, next_pos, next_yaw);
  // } else {
  //   return true;
  // }
}

bool FameExplorationManager::findPathClosestFrontier(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) const {

  // Iterate over the frontiers, and compute the path to the top viewpoint of
  // each of them. Then grab the best one
  double min_cost = std::numeric_limits<double>::max();
  bool found_ftr = false;
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
    // Get the best viewpoint for the current frontier
    const auto& vp = ftr.viewpoints_.front();

    // Check that the position is valid
    if (!isPositionReachable(pos, vp.pos_)) {
      continue;
    }

    double dist = (pos - vp.pos_).norm();
    bool valid_dist = dist <= explorer_params_->ftr_max_distance && dist >= 0.5;
    if (!valid_dist) {
      continue;
    }

    vector<Vector3d> path;
    double distance = std::max(1., explorer_params_->w_distance) *
                      ViewNode::computeCost(pos, vp.pos_, yaw[0], vp.yaw_, vel, yaw[1], path);
    double formation_cost = explorer_params_->w_others * formationCost(vp.pos_);
    double previous_goal_cost = explorer_params_->w_previous_goal * previousGoalCost(vp.pos_);
    double cost = distance + formation_cost + previous_goal_cost;

    if (cost < min_cost) {
      // Update flag
      found_ftr = true;
      // Target
      min_cost = cost;
      next_pos = vp.pos_;
      next_yaw = vp.yaw_;
    }
  }

  return found_ftr;
}

bool FameExplorationManager::closestGreedyFrontier(const Vector3d& pos, const Vector3d& yaw,
    Vector3d& next_pos, double& next_yaw, bool force_different) const {

  double min_dist = std::numeric_limits<double>::max();
  bool found_ftr = false;
  for (const auto& ftr : frontier_finder_->getFrontiers()) {
    // Find the viewpoint that will be evaluated by iterating over viewpoints (sorted)
    for (const auto& vp : ftr.viewpoints_) {
      // Check that the position is valid
      if (!isPositionReachable(pos, vp.pos_)) {
        continue;
      }

      std::vector<Vector3d> path;
      double distance = ViewNode::searchPath(pos, vp.pos_, path);
      if (distance < min_dist) {
        // Check if we need to force a new goal
        const double kMinDistGoals = 1.0;
        if (force_different && (vp.pos_ - ed_->next_pos_).norm() < kMinDistGoals) {
          continue;
        }

        // Update flag
        found_ftr = true;
        // Target
        min_dist = distance;
        next_pos = vp.pos_;
        next_yaw = vp.yaw_;
      }
    }
  }

  return found_ftr;
}

double FameExplorationManager::attractivePotentialField(double distance) const {
  // NOTE: Here function returns a negative value (the lower the distance, the better),
  // since we are selecting the target with LOWEST cost
  if (distance >= 0 && distance <= pf_params_->d0) {
    return 0.0;
  } else if (distance > pf_params_->d0 && distance <= pf_params_->df) {
    return -(distance - pf_params_->d0) * (distance - pf_params_->d0);
  } else {
    double lambda = (pf_params_->df - pf_params_->d0) * (pf_params_->df - pf_params_->d0) /
                    atan(0.1 * pf_params_->df);
    return -lambda * atan(distance - 0.9 * pf_params_->df);
  }
}

double FameExplorationManager::repulsivePotentialField(double distance) const {
  if (distance <= pf_params_->dc) {
    double lambda0 =
        pow((pf_params_->dc - pf_params_->d0), 2.) * sqrt(pf_params_->dc * pf_params_->d0);
    lambda0 *= 1. / (sqrt(pf_params_->d0) - sqrt(pf_params_->dc));
    return lambda0 * (1. / sqrt(distance) - 1. / sqrt(pf_params_->d0));
  } else if (distance >= pf_params_->dc && distance <= pf_params_->d0) {
    return (distance - pf_params_->d0) * (distance - pf_params_->d0);
  } else {
    return 0.;
  }
}

double FameExplorationManager::previousGoalCost(const Eigen::Vector3d& target_pos) const {
  double distance = (ed_->next_pos_ - target_pos).norm();
  return attractivePotentialField(distance);
}

double FameExplorationManager::formationCost(const Eigen::Vector3d& target_pos) const {
  double formation_cost = 0.0;

  // Iterate over the drones
  for (int i = 0; i < ed_->swarm_state_.size(); ++i) {
    // Skip oneself
    if (i == ep_->drone_id_ - 1) continue;

    const auto& drone_state = ed_->swarm_state_[i];

    double distance_goals = (target_pos - drone_state.goal_pos_).norm();
    formation_cost += pf_params_->kr * repulsivePotentialField(distance_goals);

    double distance_pos = (target_pos - drone_state.pos_).norm();
    formation_cost += pf_params_->kr * repulsivePotentialField(distance_pos);
  }

  // Push drone towards useful area for collaboration
  if (ed_->swarm_state_.size() > 1) {
    double distance = (target_pos - ed_->next_pos_).norm();
    formation_cost += pf_params_->ka * attractivePotentialField(distance);
  }

  return formation_cost;
}

bool FameExplorationManager::collectorPlan(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

  if (!findTourOfTrails(pos, vel, yaw, next_pos, next_yaw)) {
    return greedyPlan(pos, vel, yaw, next_pos, next_yaw);
  } else {
    return true;
  }

  // bool success_line_path = linePlan(pos, vel, yaw, next_pos, next_yaw);
  // if (!success_line_path) {
  //   return greedyPlan(pos, vel, yaw, next_pos, next_yaw);
  // } else {
  //   return true;
  // }
}

bool FameExplorationManager::linePlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
    Vector3d& next_pos, double& next_yaw) {

  bool success = false;

  // Get all the most promising viewpoint for each frontier
  vector<pair<LABEL, Vector3d>> vp_pos;
  vector<Vector3d> clusters_avg;
  vector<double> vp_yaws;
  vector<int> ids;
  frontier_finder_->getTopViewpointsInfo(pos, vp_pos, vp_yaws, clusters_avg, ids);

  auto areaAlreadyAssigned = [&](const Eigen::Vector3d& candidate_pos) {
    for (int i = 0; i < ed_->swarm_state_.size(); ++i) {
      if (i == ep_->drone_id_ - 1) {
        continue;
      } else if ((candidate_pos - ed_->swarm_state_[i].goal_pos_).norm() < 8.) {
        return true;
      }
    }
    return false;
  };

  double min_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < vp_pos.size(); ++i) {
    Vector3d view_pos = vp_pos[i].second;
    LABEL ftr_label = vp_pos[i].first;
    bool valid_ftr = (ftr_label == LABEL::TRAIL && role_ == ROLE::GARBAGE_COLLECTOR) ||
                     (ftr_label == LABEL::FRONTIER && role_ == ROLE::EXPLORER);

    if (valid_ftr && ViewNode::straightLineFeasible(pos, view_pos) &&
        !areaAlreadyAssigned(view_pos)) {

      // Check that the position is valid
      if (!isPositionReachable(pos, view_pos)) {
        continue;
      }

      // Distance
      double distance_cost = (view_pos - pos).head(2).norm();

      // Direction
      auto direction = (view_pos - pos).head(2).normalized();
      double direction_cost = ViewNode::w_dir_ * acos(vel.head(2).normalized().dot(direction));

      // Other agents
      double others_cost = collector_params_->w_others * formationCost(view_pos);

      // Label penalty
      double label_cost = 0.0;
      if (role_ == ROLE::GARBAGE_COLLECTOR && ftr_label == LABEL::FRONTIER) {
        label_cost = collector_params_->label_penalty;
      }

      double total_cost = distance_cost + direction_cost + others_cost + label_cost;
      if (total_cost < min_cost) {
        success = true;
        next_pos = view_pos;
        next_yaw = vp_yaws[i];
      }
    }
  }

  return success;
}

bool FameExplorationManager::greedyPlan(const Vector3d& pos, const Vector3d& vel,
    const Vector3d& yaw, Vector3d& next_pos, double& next_yaw) {

  ed_->trails_tour_.clear();

  // Get all the most promising viewpoint for each frontier
  vector<pair<LABEL, Vector3d>> vp_pos;
  vector<Vector3d> clusters_avg;
  vector<double> vp_yaws;
  vector<int> ids;
  frontier_finder_->getTopViewpointsInfo(pos, vp_pos, vp_yaws, clusters_avg, ids);

  // Calculate the cost for each viewpoint
  // Iterate over viewpoints
  double min_cost = std::numeric_limits<double>::max();
  bool goal_found = false;
  for (size_t i = 0; i < vp_pos.size(); ++i) {
    Vector3d vp_position = vp_pos[i].second;
    LABEL vp_label = vp_pos[i].first;

    // Check that the position is valid
    if (!isPositionReachable(pos, vp_position)) {
      continue;
    }

    // Calculate path from current position to viewpoint
    double path_length;
    if (!ViewNode::validPathExists(pos, vp_position, path_length)) {
      continue;
    }
    double time_to_dest = collector_params_->w_distance * path_length / ViewNode::vm_;

    // Calculate velocity change
    double vel_change = 0.0;
    if (vel.norm() > collector_params_->min_vel) {
      Vector3d dir = (vp_position - pos).normalized();
      double diff = acos(vel.normalized().dot(dir));
      vel_change = ViewNode::w_dir_ * diff;
    } else {
      // Calculate yaw change
      double diff = fabs(vp_yaws[i] - yaw[0]);
      vel_change = min(diff, 2 * M_PI - diff) / ViewNode::yd_;
    }
    vel_change *= collector_params_->w_direction;

    // Iterate over others
    double formation_cost = collector_params_->w_others * formationCost(vp_position);

    // Label associated cost (incentivate to explore trails)
    double label_penalty = 0.0;
    if (role_ == ROLE::EXPLORER && vp_label == LABEL::TRAIL) {
      label_penalty = collector_params_->label_penalty;
    } else if (role_ == ROLE::GARBAGE_COLLECTOR && vp_label == LABEL::FRONTIER) {
      label_penalty = collector_params_->label_penalty;
    }

    // Previous goal influence
    double previous_goal_cost = collector_params_->w_previous_goal * previousGoalCost(vp_position);

    // Compute final cost and update
    double total_cost =
        time_to_dest + vel_change + formation_cost + label_penalty + previous_goal_cost;

    if (total_cost < min_cost) {
      min_cost = total_cost;
      next_pos = vp_position;
      next_yaw = vp_yaws[i];
      goal_found = true;
    }
  }

  return goal_found;
}

void FameExplorationManager::shortenPath(vector<Vector3d>& path) {
  if (path.empty()) {
    ROS_ERROR("Empty path to shorten");
    return;
  }
  // Shorten the tour, only critical intermediate points are reserved.
  const double dist_thresh = 3.0;
  vector<Vector3d> short_tour = { path.front() };
  for (int i = 1; i < path.size() - 1; ++i) {
    if ((path[i] - short_tour.back()).norm() > dist_thresh)
      short_tour.push_back(path[i]);
    else {
      // Add waypoints to shorten path only to avoid collision
      ViewNode::caster_->input(short_tour.back(), path[i + 1]);
      Eigen::Vector3i idx;
      while (ViewNode::caster_->nextId(idx) && ros::ok()) {
        if (edt_environment_->sdf_map_->getInflateOccupancy(idx) == 1 ||
            edt_environment_->sdf_map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
          short_tour.push_back(path[i]);
          break;
        }
      }
    }
  }
  if ((path.back() - short_tour.back()).norm() > 1e-3) short_tour.push_back(path.back());

  // Ensure at least three points in the path
  if (short_tour.size() == 2)
    short_tour.insert(short_tour.begin() + 1, 0.5 * (short_tour[0] + short_tour[1]));
  path = short_tour;
}

void FameExplorationManager::findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d cur_yaw, vector<int>& indices) {
  auto t1 = ros::Time::now();

  // Get cost matrix for current state and clusters
  Eigen::MatrixXd cost_mat;
  frontier_finder_->getFullCostMatrix(cur_pos, cur_vel, cur_yaw, cost_mat);
  const int dimension = cost_mat.rows();
  // std::cout << "mat:   " << cost_mat.rows() << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Initialize TSP par file
  ofstream par_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par");
  par_file << "PROBLEM_FILE = " << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp\n";
  par_file << "GAIN23 = NO\n";
  par_file << "OUTPUT_TOUR_FILE ="
           << ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) +
                  ".tou"
                  "r\n";
  par_file << "RUNS = 1\n";
  par_file.close();

  // Write params and cost matrix to problem file
  ofstream prob_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tsp");
  // Problem specification part, follow the format of TSPLIB
  string prob_spec;
  prob_spec = "NAME : single\nTYPE : ATSP\nDIMENSION : " + to_string(dimension) +
              "\nEDGE_WEIGHT_TYPE : "
              "EXPLICIT\nEDGE_WEIGHT_FORMAT : FULL_MATRIX\nEDGE_WEIGHT_SECTION\n";
  prob_file << prob_spec;
  // prob_file << "TYPE : TSP\n";
  // prob_file << "EDGE_WEIGHT_FORMAT : LOWER_ROW\n";
  // Problem data part
  const int scale = 100;
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = cost_mat(i, j) * scale;
      prob_file << int_cost << " ";
    }
    prob_file << "\n";
  }
  prob_file << "EOF";
  prob_file.close();

  // solveTSPLKH((ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".par").c_str());
  lkh_tsp_solver::SolveTSP srv;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve TSP.");
    return;
  }

  // Read optimal tour from the tour section of result file
  ifstream res_file(ep_->tsp_dir_ + "/drone_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  while (getline(res_file, res)) {
    // Go to tour section
    if (res.compare("TOUR_SECTION") == 0) break;
  }

  // Read path for ATSP formulation
  while (getline(res_file, res)) {
    // Read indices of frontiers in optimal tour
    int id = stoi(res);
    if (id == 1)  // Ignore the current state
      continue;
    if (id == -1) break;
    indices.push_back(id - 2);  // Idx of solver-2 == Idx of frontier
  }

  res_file.close();

  std::cout << "Tour " << ep_->drone_id_ << ": ";
  for (auto id : indices) std::cout << id << ", ";
  std::cout << "" << std::endl;

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);

  double tsp_time = (ros::Time::now() - t1).toSec();
  ROS_INFO("Cost mat: %lf, TSP: %lf", mat_time, tsp_time);

  // if (tsp_time > 0.1) ROS_BREAK();
}

void FameExplorationManager::refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<vector<Vector3d>>& n_points,
    const vector<vector<double>>& n_yaws, vector<Vector3d>& refined_pts,
    vector<double>& refined_yaws) {
  double create_time, search_time, parse_time;
  auto t1 = ros::Time::now();

  // Create graph for viewpoints selection
  GraphSearch<ViewNode> g_search;
  vector<ViewNode::Ptr> last_group, cur_group;

  // Add the current state
  ViewNode::Ptr first(new ViewNode(cur_pos, cur_yaw[0]));
  first->vel_ = cur_vel;
  g_search.addNode(first);
  last_group.push_back(first);
  ViewNode::Ptr final_node;

  // Add viewpoints
  std::cout << "Local refine graph size: 1, ";
  for (int i = 0; i < n_points.size(); ++i) {
    // Create nodes for viewpoints of one frontier
    for (int j = 0; j < n_points[i].size(); ++j) {
      ViewNode::Ptr node(new ViewNode(n_points[i][j], n_yaws[i][j]));
      g_search.addNode(node);
      // Connect a node to nodes in last group
      for (auto nd : last_group) g_search.addEdge(nd->id_, node->id_);
      cur_group.push_back(node);

      // Only keep the first viewpoint of the last local frontier
      if (i == n_points.size() - 1) {
        final_node = node;
        break;
      }
    }
    // Store nodes for this group for connecting edges
    std::cout << cur_group.size() << ", ";
    last_group = cur_group;
    cur_group.clear();
  }
  std::cout << "" << std::endl;
  create_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Search optimal sequence
  vector<ViewNode::Ptr> path;
  g_search.DijkstraSearch(first->id_, final_node->id_, path);

  search_time = (ros::Time::now() - t1).toSec();
  t1 = ros::Time::now();

  // Return searched sequence
  for (int i = 1; i < path.size(); ++i) {
    refined_pts.push_back(path[i]->pos_);
    refined_yaws.push_back(path[i]->yaw_);
  }

  // Extract optimal local tour (for visualization)
  ed_->refined_tour_.clear();
  ed_->refined_tour_.push_back(cur_pos);
  ViewNode::astar_->lambda_heu_ = 1.0;
  ViewNode::astar_->setResolution(0.2);
  for (auto pt : refined_pts) {
    vector<Vector3d> path;
    if (ViewNode::searchPath(ed_->refined_tour_.back(), pt, path))
      ed_->refined_tour_.insert(ed_->refined_tour_.end(), path.begin(), path.end());
    else
      ed_->refined_tour_.push_back(pt);
  }
  ViewNode::astar_->lambda_heu_ = 10000;

  parse_time = (ros::Time::now() - t1).toSec();
  // ROS_WARN("create: %lf, search: %lf, parse: %lf", create_time, search_time, parse_time);
}

void FameExplorationManager::allocateGrids(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
    const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
    vector<int>& other_ids) {
  // ROS_INFO("Allocate grid.");

  auto t1 = ros::Time::now();
  auto t2 = t1;

  if (grid_ids.size() == 1) {  // Only one grid, no need to run ACVRP
    auto pt = hgrid_->getCenter(grid_ids.front());
    // double d1 = (positions[0] - pt).norm();
    // double d2 = (positions[1] - pt).norm();
    vector<Eigen::Vector3d> path;
    double d1 = ViewNode::computeCost(positions[0], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    double d2 = ViewNode::computeCost(positions[1], pt, 0, 0, Eigen::Vector3d(0, 0, 0), 0, path);
    if (d1 < d2) {
      ego_ids = grid_ids;
      other_ids = {};
    } else {
      ego_ids = {};
      other_ids = grid_ids;
    }
    return;
  }

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, prev_first_ids, grid_ids, mat);
  hgrid_->getCostMatrix(positions, velocities, first_ids, second_ids, grid_ids, mat);

  // int unknown = hgrid_->getTotalUnknwon();
  int unknown;

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through AmTSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = positions.size();

  vector<int> unknown_nums;
  int capacity = 0;
  for (int i = 0; i < grid_ids.size(); ++i) {
    int unum = hgrid_->getUnknownCellsNum(grid_ids[i]);
    unknown_nums.push_back(unum);
    capacity += unum;
    // std::cout << "Grid " << i << ": " << unum << std::endl;
  }
  // std::cout << "Total: " << capacity << std::endl;
  capacity = capacity * 0.75 * 0.1;

  // int prob_type;
  // if (grid_ids.size() >= 3)
  //   prob_type = 2;  // Use ACVRP
  // else
  //   prob_type = 1;  // Use AmTSP

  const int prob_type = 2;

  // Create problem file--------------------------
  ofstream file(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : pairopt\n";

  if (prob_type == 1)
    file << "TYPE : ATSP\n";
  else if (prob_type == 2)
    file << "TYPE : ACVRP\n";

  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";

  if (prob_type == 2) {
    file << "CAPACITY : " + to_string(capacity) + "\n";   // ACVRP
    file << "VEHICLES : " + to_string(drone_num) + "\n";  // ACVRP
  }

  // Cost matrix
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }

  if (prob_type == 2) {  // Demand section, ACVRP only
    file << "DEMAND_SECTION\n";
    file << "1 0\n";
    for (int i = 0; i < drone_num; ++i) {
      file << to_string(i + 2) + " 0\n";
    }
    for (int i = 0; i < grid_ids.size(); ++i) {
      int grid_unknown = unknown_nums[i] * 0.1;
      file << to_string(i + 2 + drone_num) + " " + to_string(grid_unknown) + "\n";
    }
    file << "DEPOT_SECTION\n";
    file << "1\n";
    file << "EOF";
  }

  file.close();

  // Create par file------------------------------------------
  int min_size = int(grid_ids.size()) / 2;
  int max_size = ceil(int(grid_ids.size()) / 2.0);
  file.open(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp\n";
  if (prob_type == 1) {
    file << "SALESMEN = " << to_string(drone_num) << "\n";
    file << "MTSP_OBJECTIVE = MINSUM\n";
    // file << "MTSP_OBJECTIVE = MINMAX\n";
    file << "MTSP_MIN_SIZE = " << to_string(min_size) << "\n";
    file << "MTSP_MAX_SIZE = " << to_string(max_size) << "\n";
    file << "TRACE_LEVEL = 0\n";
  } else if (prob_type == 2) {
    file << "TRACE_LEVEL = 1\n";  // ACVRP
    file << "SEED = 0\n";         // ACVRP
  }
  file << "RUNS = 1\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour\n";

  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 3;
  // if (!tsp_client_.call(srv)) {
  if (!acvrp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ACVRP.");
    return;
  }
  // system("/home/boboyu/software/LKH-3.0.6/LKH
  // /home/boboyu/workspaces/hkust_swarm_ws/src/swarm_exploration/utils/lkh_mtsp_solver/resource/amtsp3_1.par");

  double mtsp_time = (ros::Time::now() - t1).toSec();
  std::cout << "Allocation time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp3_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }
  // // Print tour ids
  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }

  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      ego_ids.insert(ego_ids.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      other_ids.insert(other_ids.end(), tours[i].begin() + 1, tours[i].end());
    }
  }
  for (auto& id : ego_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  for (auto& id : other_ids) {
    id = grid_ids[id - 1 - drone_num];
  }
  // // Remove repeated grid
  // unordered_map<int, int> ego_map, other_map;
  // for (auto id : ego_ids) ego_map[id] = 1;
  // for (auto id : other_ids) other_map[id] = 1;

  // ego_ids.clear();
  // other_ids.clear();
  // for (auto p : ego_map) ego_ids.push_back(p.first);
  // for (auto p : other_map) other_ids.push_back(p.first);

  // sort(ego_ids.begin(), ego_ids.end());
  // sort(other_ids.begin(), other_ids.end());
}

double FameExplorationManager::computeGridPathCost(const Eigen::Vector3d& pos,
    const vector<int>& grid_ids, const vector<int>& first, const vector<vector<int>>& firsts,
    const vector<vector<int>>& seconds, const double& w_f) {
  if (grid_ids.empty()) return 0.0;

  double cost = 0.0;
  vector<Eigen::Vector3d> path;
  cost += hgrid_->getCostDroneToGrid(pos, grid_ids[0], first);
  for (int i = 0; i < grid_ids.size() - 1; ++i) {
    cost += hgrid_->getCostGridToGrid(grid_ids[i], grid_ids[i + 1], firsts, seconds, firsts.size());
  }
  return cost;
}

bool FameExplorationManager::findTourOfTrails(const Vector3d& cur_pos,
    const Eigen::Vector3d& cur_yaw, const Vector3d& cur_vel, Eigen::Vector3d& next_pos,
    double& next_yaw) {
  // 1. Get all trails within an area
  list<Frontier> close_by_trails;
  frontier_finder_->getTrailCentroidsAroundPosition(
      close_by_trails, cur_pos, collector_params_->ftr_max_distance);

  // Limit case: we don't have trails around or we have only one
  const size_t num_trails = close_by_trails.size();
  if (num_trails == 0) {
    return false;
  } else if (num_trails == 1) {
    // Target pose
    next_pos = close_by_trails.front().viewpoints_.front().pos_;
    next_yaw = close_by_trails.front().viewpoints_.front().yaw_;

    // For visualization
    ed_->trails_tour_ = { { cur_pos, cur_yaw[0] }, { next_pos, next_yaw } };

    return true;
  }

  // Create cost matrix
  const auto dimension = num_trails + 1;
  Eigen::MatrixXd cost_mat = Eigen::MatrixXd::Zero(dimension, dimension);

  auto updateCostFrontiers = [&](const list<Frontier>::iterator& it1,
                                 const list<Frontier>::iterator& it2) {
    // Search path from old cluster's top viewpoint to new cluster'
    const Viewpoint& vui = it1->viewpoints_.front();
    const Viewpoint& vuj = it2->viewpoints_.front();
    vector<Vector3d> path_ij;
    return collector_params_->w_distance * ViewNode::searchPath(vui.pos_, vuj.pos_, path_ij) +
           collector_params_->w_others * (formationCost(vui.pos_) - formationCost(vuj.pos_));
  };

  auto updateCostFromState = [&](const list<Frontier>::iterator& it) {
    // Search path from old cluster's top viewpoint to new cluster'
    const Viewpoint& v = it->viewpoints_.front();
    vector<Vector3d> path;
    return collector_params_->w_distance * ViewNode::searchPath(cur_pos, v.pos_, path) +
           collector_params_->w_others * formationCost(v.pos_);  // +
    // collector_params_->w_previous_goal * previousGoalCost(v.pos_);
  };

  auto start_time = ros::Time::now();

  // cost between trails
  int i = 1, j = 1;
  for (auto it1 = close_by_trails.begin(); it1 != close_by_trails.end(); ++it1) {
    for (auto it2 = close_by_trails.begin(); it2 != close_by_trails.end(); ++it2) {
      cost_mat(i, j++) = it1 == it2 ? 0. : updateCostFrontiers(it1, it2);
    }
    ++i;
    j = 1;
  }

  // cost between current position and trails
  j = 1;
  for (auto it = close_by_trails.begin(); it != close_by_trails.end(); ++it) {
    cost_mat(0, j++) = updateCostFromState(it);
  }

  // ROS_WARN_STREAM("Time to build problem: " << (ros::Time::now() - start_time).toSec() << " s");

  // Create problem to solve
  start_time = ros::Time::now();

  const int scale = 100;
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = scale * cost_mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(close_by_trails.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(close_by_trails.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  // ROS_WARN_STREAM("Time to solve problem: " << (ros::Time::now() - start_time).toSec() << " s");

  // Read results
  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  ed_->trails_tour_ = { { cur_pos, cur_yaw[0] } };
  for (auto id : ids) {
    if (id <= 0) continue;

    auto it = close_by_trails.begin();
    std::advance(it, id - 1);
    ed_->trails_tour_.push_back({ it->viewpoints_.front().pos_, it->viewpoints_.front().yaw_ });
  }

  // Target pose
  next_pos = ed_->trails_tour_[1].first;
  next_yaw = ed_->trails_tour_[1].second;
  return true;
}

bool FameExplorationManager::findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
    const vector<Eigen::Vector3d>& velocities, vector<int>& indices, vector<vector<int>>& others,
    bool init) {

  ROS_INFO("Find grid tour---------------");

  auto t1 = ros::Time::now();

  auto& grid_ids = ed_->swarm_state_[ep_->drone_id_ - 1].grid_ids_;

  // hgrid_->updateBaseCoor();  // Use the latest basecoor transform of swarm

  vector<int> first_ids, second_ids;
  hgrid_->inputFrontiers(ed_->averages_);

  hgrid_->updateGridData(
      ep_->drone_id_, grid_ids, ed_->reallocated_, ed_->last_grid_ids_, first_ids, second_ids);

  if (grid_ids.empty()) {
    ROS_WARN("Empty dominance.");
    ed_->grid_tour_.clear();
    return false;
  }

  // std::cout << "Allocated grid: ";
  // for (auto id : grid_ids) std::cout << id << ", ";
  // std::cout << "" << std::endl;

  Eigen::MatrixXd mat;
  // uniform_grid_->getCostMatrix(positions, velocities, first_ids, grid_ids, mat);
  if (!init)
    hgrid_->getCostMatrix(positions, velocities, { first_ids }, { second_ids }, grid_ids, mat);
  else
    hgrid_->getCostMatrix(positions, velocities, { {} }, { {} }, grid_ids, mat);

  double mat_time = (ros::Time::now() - t1).toSec();

  // Find optimal path through ATSP
  t1 = ros::Time::now();
  const int dimension = mat.rows();
  const int drone_num = 1;

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  file.open(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  // file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) <<
  // "\n"; file << "MTSP_MAX_SIZE = "
  //      << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 2;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return false;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // std::cout << "AmTSP time: " << mtsp_time << std::endl;

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp2_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour of grid
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  // for (auto tr : tours) {
  //   std::cout << "tour: ";
  //   for (auto id : tr) std::cout << id << ", ";
  //   std::cout << "" << std::endl;
  // }
  others.resize(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    } else {
      others[tours[i][0] - 2].insert(
          others[tours[i][0] - 2].end(), tours[i].begin(), tours[i].end());
    }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  for (auto& other : others) {
    for (auto& id : other) id -= 1 + drone_num;
  }
  // std::cout << "Grid tour: ";
  for (auto& id : indices) {
    id = grid_ids[id];
    // std::cout << id << ", ";
  }
  // std::cout << "" << std::endl;

  // uniform_grid_->getGridTour(indices, ed_->grid_tour_);
  grid_ids = indices;
  hgrid_->getGridTour(grid_ids, positions[0], ed_->grid_tour_, ed_->grid_tour2_);

  ed_->last_grid_ids_ = grid_ids;
  ed_->reallocated_ = false;

  // hgrid_->checkFirstGrid(grid_ids.front());

  return true;
}

void FameExplorationManager::findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel,
    const Vector3d& cur_yaw, const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos,
    vector<int>& indices) {

  auto t1 = ros::Time::now();

  vector<Eigen::Vector3d> positions = { cur_pos };
  vector<Eigen::Vector3d> velocities = { cur_vel };
  vector<double> yaws = { cur_yaw[0] };

  // frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, mat);
  Eigen::MatrixXd mat;
  frontier_finder_->getSwarmCostMatrix(positions, velocities, yaws, ftr_ids, grid_pos, mat);
  const int dimension = mat.rows();
  // std::cout << "dim of frontier TSP mat: " << dimension << std::endl;

  double mat_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("mat time: %lf", mat_time);

  // Find optimal allocation through AmTSP
  t1 = ros::Time::now();

  // Create problem file
  ofstream file(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp");
  file << "NAME : amtsp\n";
  file << "TYPE : ATSP\n";
  file << "DIMENSION : " + to_string(dimension) + "\n";
  file << "EDGE_WEIGHT_TYPE : EXPLICIT\n";
  file << "EDGE_WEIGHT_FORMAT : FULL_MATRIX\n";
  file << "EDGE_WEIGHT_SECTION\n";
  for (int i = 0; i < dimension; ++i) {
    for (int j = 0; j < dimension; ++j) {
      int int_cost = 100 * mat(i, j);
      file << int_cost << " ";
    }
    file << "\n";
  }
  file.close();

  // Create par file
  const int drone_num = 1;

  file.open(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".par");
  file << "SPECIAL\n";
  file << "PROBLEM_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp\n";
  file << "SALESMEN = " << to_string(drone_num) << "\n";
  file << "MTSP_OBJECTIVE = MINSUM\n";
  file << "MTSP_MIN_SIZE = " << to_string(min(int(ed_->frontiers_.size()) / drone_num, 4)) << "\n";
  file << "MTSP_MAX_SIZE = "
       << to_string(max(1, int(ed_->frontiers_.size()) / max(1, drone_num - 1))) << "\n";
  file << "RUNS = 1\n";
  file << "TRACE_LEVEL = 0\n";
  file << "TOUR_FILE = " + ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour\n";
  file.close();

  auto par_dir = ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".atsp";
  t1 = ros::Time::now();

  lkh_mtsp_solver::SolveMTSP srv;
  srv.request.prob = 1;
  if (!tsp_client_.call(srv)) {
    ROS_ERROR("Fail to solve ATSP.");
    return;
  }

  double mtsp_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("AmTSP time: %lf", mtsp_time);

  // Read results
  t1 = ros::Time::now();

  ifstream fin(ep_->mtsp_dir_ + "/amtsp_" + to_string(ep_->drone_id_) + ".tour");
  string res;
  vector<int> ids;
  while (getline(fin, res)) {
    if (res.compare("TOUR_SECTION") == 0) break;
  }
  while (getline(fin, res)) {
    int id = stoi(res);
    ids.push_back(id - 1);
    if (id == -1) break;
  }
  fin.close();

  // Parse the m-tour
  vector<vector<int>> tours;
  vector<int> tour;
  for (auto id : ids) {
    if (id > 0 && id <= drone_num) {
      tour.clear();
      tour.push_back(id);
    } else if (id >= dimension || id <= 0) {
      tours.push_back(tour);
    } else {
      tour.push_back(id);
    }
  }

  vector<vector<int>> others(drone_num - 1);
  for (int i = 1; i < tours.size(); ++i) {
    if (tours[i][0] == 1) {
      indices.insert(indices.end(), tours[i].begin() + 1, tours[i].end());
    }
    // else {
    //   others[tours[i][0] - 2].insert(
    //       others[tours[i][0] - 2].end(), tours[i].begin() + 1, tours[i].end());
    // }
  }
  for (auto& id : indices) {
    id -= 1 + drone_num;
  }
  // for (auto& other : others) {
  //   for (auto& id : other)
  //     id -= 1 + drone_num;
  // }

  if (ed_->grid_tour_.size() > 2) {  // Remove id for next grid, since it is considered in the TSP
    indices.pop_back();
  }
  // Subset of frontier inside first grid
  for (int i = 0; i < indices.size(); ++i) {
    indices[i] = ftr_ids[indices[i]];
  }

  // Get the path of optimal tour from path matrix
  frontier_finder_->getPathForTour(cur_pos, indices, ed_->frontier_tour_);
  if (!grid_pos.empty()) {
    ed_->frontier_tour_.push_back(grid_pos[0]);
  }

  // ed_->other_tours_.clear();
  // for (int i = 1; i < positions.size(); ++i) {
  //   ed_->other_tours_.push_back({});
  //   frontier_finder_->getPathForTour(positions[i], others[i - 1], ed_->other_tours_[i - 1]);
  // }

  double parse_time = (ros::Time::now() - t1).toSec();
  // ROS_INFO("Cost mat: %lf, TSP: %lf, parse: %f, %d frontiers assigned.", mat_time, mtsp_time,
  //     parse_time, indices.size());
}

}  // namespace fast_planner
