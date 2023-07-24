#ifndef _FRONTIER_FINDER_H_
#define _FRONTIER_FINDER_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::vector;

class RayCaster;

namespace fast_planner {
class EDTEnvironment;
class PerceptionUtils;

// Viewpoint to cover a frontier cluster
struct Viewpoint {
  // Position and heading
  Vector3d pos_;
  double yaw_;
  // Fraction of the cluster that can be covered
  // double fraction_;
  int visib_num_;
};

// Type of frontier
enum LABEL : int { TRAIL = 0, FRONTIER = 1, UNLABELED = 2 };

// A frontier cluster, the viewpoints to cover it
struct Frontier {
  // Complete voxels belonging to the cluster
  vector<pair<LABEL, Vector3d>> cells_;
  // down-sampled voxels filtered by voxel grid filter
  vector<Vector3d> filtered_cells_;
  // Average position of all voxels
  Vector3d average_;
  // Idx of cluster
  int id_;
  // Viewpoints that can cover the cluster
  vector<Viewpoint> viewpoints_;
  // Bounding box of cluster, center & 1/2 side length
  Vector3d box_min_, box_max_;
  // Path and cost from this cluster to other clusters
  list<vector<Vector3d>> paths_;
  list<double> costs_;
  // Type of frontier
  LABEL label_{ UNLABELED };
};

class FrontierFinder {
public:
  FrontierFinder(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~FrontierFinder();

  void searchFrontiers();
  void computeFrontiersToVisit();

  inline list<Frontier> getFrontiers() const {
    return frontiers_;
  }
  void getFrontiers(vector<vector<Vector3d>>& clusters);
  void getFrontiersIds(vector<int>& ids);
  void getLabeledFrontiers(vector<pair<LABEL, vector<Vector3d>>>& labeled_clusters);
  void getDormantFrontiers(vector<vector<Vector3d>>& clusters);
  void getInFrontFrontiers(vector<pair<LABEL, vector<Eigen::Vector3d>>>& infront_clusters) const;
  void getTrailCentroidsAroundPosition(
      list<Frontier>& trails, const Eigen::Vector3d& position, double max_distance) const;
  void getFrontierBoxes(vector<pair<Vector3d, Vector3d>>& boxes);

  // Utilities
  bool isFrontierInFront(const Eigen::Vector3d& ftr_pos, const Eigen::Vector3d& pos,
      const Eigen::Vector3d& vel, const double& yaw, double max_dist, double max_ang_dist) const;

  list<Frontier> updateFrontiersInFront(const Eigen::Vector3d& pos, const Eigen::Vector3d& vel,
      const double& yaw, double max_dist, double max_ang_dist);

  // Get viewpoint with highest coverage for each frontier
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<Vector3d>& points, vector<double>& yaws,
      vector<Vector3d>& averages);
  void getTopViewpointsInfo(const Vector3d& cur_pos, vector<pair<LABEL, Eigen::Vector3d>>& points,
      vector<double>& yaws, vector<Eigen::Vector3d>& averages, vector<int>& ftr_ids);
  // Get several viewpoints for a subset of frontiers
  void getViewpointsInfo(const Vector3d& cur_pos, const vector<int>& ids, const int& view_num,
      const double& max_decay, vector<vector<Vector3d>>& points, vector<vector<double>>& yaws);
  void updateFrontierCostMatrix();
  void getFullCostMatrix(const Vector3d& cur_pos, const Vector3d& cur_vel, const Vector3d cur_yaw,
      Eigen::MatrixXd& mat);
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities,
      const vector<double> yaws, Eigen::MatrixXd& mat);
  void getSwarmCostMatrix(const vector<Vector3d>& positions, const vector<Vector3d>& velocities,
      const vector<double>& yaws, const vector<int>& ftr_ids,
      const vector<Eigen::Vector3d>& grid_pos, Eigen::MatrixXd& mat);

  void getPathForTour(const Vector3d& pos, const vector<int>& frontier_ids, vector<Vector3d>& path);

  void setNextFrontier(const int& id);
  bool isFrontierCovered();
  void wrapYaw(double& yaw);
  int computeGainOfView(const Eigen::Vector3d& pos, const double& yaw);
  int deleteFrontiers(const vector<uint16_t>& ids);
  int addFrontiers(const vector<pair<Eigen::Vector3d, double>>& views);

  void binaryClassify();

  shared_ptr<PerceptionUtils> percep_utils_;

private:
  void splitLargeFrontiers(list<Frontier>& frontiers);
  bool splitHorizontally(const Frontier& frontier, list<Frontier>& splits);
  void mergeFrontiers(Frontier& ftr1, const Frontier& ftr2);
  bool isFrontierChanged(const Frontier& ft);
  bool haveOverlap(
      const Vector3d& min1, const Vector3d& max1, const Vector3d& min2, const Vector3d& max2);
  bool haveAnyOverlap(const Vector3d& min1, const Vector3d& max1, const vector<Vector3d>& mins,
      const vector<Vector3d>& maxs);
  void computeFrontierInfo(Frontier& frontier);
  void downsample(
      const vector<pair<LABEL, Eigen::Vector3d>>& cluster_in, vector<Vector3d>& cluster_out);
  void sampleViewpoints(Frontier& frontier);

  int countVisibleCells(const Vector3d& pos, const double& yaw, const vector<Vector3d>& cluster);
  bool isNearUnknown(const Vector3d& pos);
  vector<Eigen::Vector3i> sixNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> tenNeighbors(const Eigen::Vector3i& voxel);
  vector<Eigen::Vector3i> allNeighbors(const Eigen::Vector3i& voxel);
  bool isNeighborUnknown(const Eigen::Vector3i& voxel);
  void expandFrontier(const Eigen::Vector3i& first /* , const int& depth, const int& parent_id */);

  // Wrapper of sdf map
  int toadr(const Eigen::Vector3i& idx);
  bool knownfree(const Eigen::Vector3i& idx);
  bool inmap(const Eigen::Vector3i& idx);

  // Deprecated
  Eigen::Vector3i searchClearVoxel(const Eigen::Vector3i& pt);
  bool isInBoxes(const vector<pair<Vector3d, Vector3d>>& boxes, const Eigen::Vector3i& idx);
  bool canBeMerged(const Frontier& ftr1, const Frontier& ftr2);
  void findViewpoints(const Vector3d& sample, const Vector3d& ftr_avg, vector<Viewpoint>& vps);

  // Data
  vector<char> frontier_flag_;
  list<Frontier> frontiers_, dormant_frontiers_, tmp_frontiers_, infront_frontiers_;
  vector<int> removed_ids_;
  list<Frontier>::iterator first_new_ftr_;
  Frontier next_frontier_;

  // Params
  int cluster_min_;
  double cluster_size_xy_, cluster_size_z_;
  double candidate_rmax_, candidate_rmin_, candidate_dphi_, min_candidate_dist_,
      min_candidate_clearance_;
  int down_sample_;
  double min_view_finish_fraction_, resolution_;
  int min_visib_num_, candidate_rnum_;

  // Utils
  shared_ptr<EDTEnvironment> edt_env_;
  unique_ptr<RayCaster> raycaster_;
};

}  // namespace fast_planner
#endif
