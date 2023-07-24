#ifndef _FAME_EXPLORATION_FSM_H_
#define _FAME_EXPLORATION_FSM_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>

#include <exploration_manager/role_assigner.h>

using Eigen::Vector3d;
using std::pair;
using std::shared_ptr;
using std::tuple;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class EDTEnvironment;
class SDFMap;
class FastPlannerManager;
// class UniformGrid;
class HGrid;
class FrontierFinder;
struct ExplorationParam;
struct ExplorationData;

enum EXPL_RESULT { NO_FRONTIER, FAIL, SUCCEED, NO_GRID };

struct ExplorerParams {
  double ftr_max_distance;
  double max_ang_dist;
  double label_penalty;

  // Weigths
  double w_distance;
  double w_direction;
  double w_others;
  double w_previous_goal;
};

struct CollectorParams {
  double ftr_max_distance;
  double min_vel;
  double label_penalty;
  double velocity_factor;

  // Weigths
  double w_distance;
  double w_direction;
  double w_others;
  double w_previous_goal;
};

struct PotentialFieldParams {
  // Attrative
  double ka;
  double df;  // min distance at which attraction begins

  // Repulsive
  double kr;
  double dc;  // min distance to have collision
  double d0;  // distance after which we should not approach closer
};

class FameExplorationManager {
public:
  FameExplorationManager();
  ~FameExplorationManager();

  void initialize(ros::NodeHandle& nh);

  int planExploreMotion(
      const Vector3d& pos, const Vector3d& vel, const Vector3d& acc, const Vector3d& yaw);

  bool isPositionReachable(const Vector3d& from, const Vector3d& to) const;

  int planTrajToView(const Vector3d& pos, const Vector3d& vel, const Vector3d& acc,
      const Vector3d& yaw, const Vector3d& next_pos, const double& next_yaw);

  int updateFrontierStruct(const Eigen::Vector3d& pos, double yaw, const Eigen::Vector3d& vel);

  void allocateGrids(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
      const vector<vector<int>>& second_ids, const vector<int>& grid_ids, vector<int>& ego_ids,
      vector<int>& other_ids);

  // Find optimal tour visiting unknown grid
  bool findGlobalTourOfGrid(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, vector<int>& ids, vector<vector<int>>& others,
      bool init = false);

  void calcMutualCosts(const Eigen::Vector3d& pos, const double& yaw, const Eigen::Vector3d& vel,
      const vector<pair<Eigen::Vector3d, double>>& views, vector<float>& costs);

  double computeGridPathCost(const Eigen::Vector3d& pos, const vector<int>& grid_ids,
      const vector<int>& first, const vector<vector<int>>& firsts,
      const vector<vector<int>>& seconds, const double& w_f);

  shared_ptr<ExplorationData> ed_;
  shared_ptr<ExplorationParam> ep_;
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FrontierFinder> frontier_finder_;
  shared_ptr<HGrid> hgrid_;
  shared_ptr<SDFMap> sdf_map_;
  shared_ptr<RoleAssigner> role_assigner_;
  // shared_ptr<UniformGrid> uniform_grid_;

  // Parameters
  shared_ptr<ExplorerParams> explorer_params_;
  shared_ptr<CollectorParams> collector_params_;
  shared_ptr<PotentialFieldParams> pf_params_;

  // Current role
  ROLE role_;

private:
  shared_ptr<EDTEnvironment> edt_environment_;
  ros::ServiceClient tsp_client_, acvrp_client_;

  // Find optimal tour for coarse viewpoints of all frontiers
  void findGlobalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
      vector<int>& indices);

  // Refine local tour for next few frontiers, using more diverse viewpoints
  void refineLocalTour(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<vector<Vector3d>>& n_points, const vector<vector<double>>& n_yaws,
      vector<Vector3d>& refined_pts, vector<double>& refined_yaws);

  // Update role and velocities
  void updateRoleAndVelocities(const ROLE updated_role);

  // Explorer
  bool explorerPlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
      Vector3d& next_pos, double& next_yaw);

  bool findPathClosestFrontier(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
      Vector3d& next_pos, double& next_yaw) const;

  bool closestGreedyFrontier(const Vector3d& pos, const Vector3d& yaw, Vector3d& next_pos,
      double& next_yaw, bool force_different = false) const;

  double attractivePotentialField(double distance) const;

  double repulsivePotentialField(double distance) const;

  double previousGoalCost(const Eigen::Vector3d& target_pos) const;

  double formationCost(const Eigen::Vector3d& target_pos) const;

  // Garbage Collector
  bool collectorPlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw,
      Vector3d& next_pos, double& next_yaw);

  bool linePlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw, Vector3d& next_pos,
      double& next_yaw);

  bool greedyPlan(const Vector3d& pos, const Vector3d& vel, const Vector3d& yaw, Vector3d& next_pos,
      double& next_yaw);

  bool findTourOfTrails(const Vector3d& cur_pos, const Eigen::Vector3d& cur_yaw,
      const Vector3d& cur_vel, Eigen::Vector3d& next_pos, double& next_yaw);

  void shortenPath(vector<Vector3d>& path);

  void findTourOfFrontier(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d& cur_yaw,
      const vector<int>& ftr_ids, const vector<Eigen::Vector3d>& grid_pos, vector<int>& ids);

public:
  typedef shared_ptr<FameExplorationManager> Ptr;
};

}  // namespace fast_planner

#endif