#ifndef COLLABORATION_ASSIGNER_H
#define COLLABORATION_ASSIGNER_H

#include <ros/ros.h>
#include <Eigen/Dense>

using namespace std;

namespace fast_planner {

enum COLLAB_STATE : size_t { INDEPENDENT = 0, COLLABORATE = 1, UNDEFINED = 2 };

struct CollaborationParams {
  double dist_range;
  double dist_collision;

  double w_range;
  double w_collision;
};

static string stateToString(COLLAB_STATE state) {
  if (state == INDEPENDENT)
    return "INDEPENDENT";
  else if (state == COLLABORATE)
    return "COLLABORATE";
  else
    return "UNDEFINED";
}

class CollaborationAssigner {
public:
  CollaborationAssigner(const ros::NodeHandle& nh);
  ~CollaborationAssigner();

  bool optimizePositions(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2,
      Eigen::Vector3d& x1_opt, Eigen::Vector3d& x2_opt);

  static double distanceCost(
      const std::vector<double>& x, std::vector<double>& grad, void* func_data);

  inline bool isActive() const {
    return active_;
  }

private:
  shared_ptr<CollaborationParams> params_;
  double d_star_;  // cached
  bool active_;

public:
  typedef shared_ptr<CollaborationAssigner> Ptr;
};

}  // end namespace fast_planner

#endif