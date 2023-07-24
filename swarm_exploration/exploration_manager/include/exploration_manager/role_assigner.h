#ifndef ROLE_ASSIGNER_H
#define ROLE_ASSIGNER_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <active_perception/frontier_finder.h>
#include <exploration_manager/expl_data.h>

using namespace std;

namespace fast_planner {

enum ROLE : size_t { EXPLORER = 0, GARBAGE_COLLECTOR = 1, UNKNOWN = 2 };

struct RoleParams {
  double region_size;
  double cluster_xy_size;
  int min_num_neighbours;
  int min_num_islands;
  int min_num_trails;
};

static string roleToString(ROLE role) {
  if (role == EXPLORER)
    return "EXPLORER";
  else if (role == GARBAGE_COLLECTOR)
    return "GARBAGE_COLLECTOR";
  else
    return "UNKNOWN";
}

class RoleAssigner {
public:
  RoleAssigner(const ros::NodeHandle& nh);
  ~RoleAssigner();

  ROLE assignRoleCloseByFrontiers(
      const Eigen::Vector3d& position, const list<Frontier>& frontiers) const;
  ROLE assignRole(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
      const list<Frontier>& frontiers) const;
  ROLE assignRole(const Eigen::Vector3d& position, int ego_id,
      const std::vector<DroneState>& swarm_states, const list<Frontier>& frontiers);

private:
  shared_ptr<RoleParams> params_;

  // Used for ablation studies
  bool fixed_;
  ROLE fix_role_;

public:
  typedef shared_ptr<RoleAssigner> Ptr;
};

}  // namespace fast_planner

#endif