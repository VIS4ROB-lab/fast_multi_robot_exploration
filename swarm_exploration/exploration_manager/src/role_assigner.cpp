#include <exploration_manager/role_assigner.h>

namespace fast_planner {

RoleAssigner::RoleAssigner(const ros::NodeHandle& nh) {
  params_.reset(new RoleParams);
  nh.param("role_assigner/region_size", params_->region_size, 8.0);
  nh.param("role_assigner/cluster_xy_size", params_->cluster_xy_size, 3.0);
  nh.param("role_assigner/min_num_neighbours", params_->min_num_neighbours, 3);
  nh.param("role_assigner/min_num_islands", params_->min_num_islands, 4);
  nh.param("role_assigner/min_num_trails", params_->min_num_trails, 1);

  // For ablation study
  nh.param("role_assigner/fixed", fixed_, false);

  int fix_role_int;
  nh.param("role_assigner/fix_role", fix_role_int, int(ROLE::UNKNOWN));
  fix_role_ = ROLE(fix_role_int);

  assert(fix_role_ == ROLE::GARBAGE_COLLECTOR || fix_role_ == ROLE::EXPLORER);
}

RoleAssigner::~RoleAssigner() {
}

ROLE RoleAssigner::assignRoleCloseByFrontiers(
    const Eigen::Vector3d& position, const list<Frontier>& frontiers) const {

  // Ablation study
  if (fixed_) {
    return fix_role_;
  }

  //
  size_t num_trails = 0, num_ftrs = 0;
  for (const auto& ftr : frontiers) {
    // Ignore unlabelled frontiers
    if (ftr.label_ == LABEL::UNLABELED) {
      continue;
    }

    // Update counters
    if ((position - ftr.average_).head(2).norm() <= params_->region_size) {
      if (ftr.label_ == LABEL::FRONTIER) {
        ++num_ftrs;
      } else if (ftr.label_ == LABEL::TRAIL) {
        ++num_trails;
      }
    }
  }

  // In case we do not have any frontier/trail around, speed up the drone
  // by going into GARBAGE_COLLECTOR mode
  if (num_trails == 0 && num_ftrs == 0) {
    return ROLE::GARBAGE_COLLECTOR;
  } else {
    return num_trails >= params_->min_num_trails ? ROLE::GARBAGE_COLLECTOR : ROLE::EXPLORER;
  }
}

ROLE RoleAssigner::assignRole(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity,
    const list<Frontier>& frontiers) const {
  // Ablation study
  if (fixed_) {
    return fix_role_;
  }

  //
  // Find close-by frontiers
  list<Frontier> close_by_frontiers;
  for (const auto& f : frontiers) {
    if ((f.average_ - position).norm() <= params_->region_size) {
      close_by_frontiers.push_back(f);
    }
  }

  // Check: do we have close-by frontier islands?
  auto closeClusters = [&](const list<Frontier>::iterator& ftr1,
                           const list<Frontier>::iterator& ftr2) {
    // Check if it can merge two frontier without exceeding size limit
    for (const auto& c1 : ftr1->cells_) {
      for (const auto& c2 : ftr2->cells_) {
        if ((c1.second - c2.second).head(2).norm() < params_->cluster_xy_size / 2.0) {
          return true;
        }
      }
    }
    return false;
  };

  size_t islands_count = 0;
  for (auto f1_iter = close_by_frontiers.begin(); f1_iter != close_by_frontiers.end(); ++f1_iter) {
    size_t merged_ftr = 0;
    for (auto f2_iter = f1_iter; f2_iter != close_by_frontiers.end(); ++f2_iter) {
      if (closeClusters(f1_iter, f2_iter)) ++merged_ftr;
    }
    if (merged_ftr >= params_->min_num_neighbours) ++islands_count;
  }

  return islands_count >= params_->min_num_islands ? ROLE::GARBAGE_COLLECTOR : ROLE::EXPLORER;
}

ROLE RoleAssigner::assignRole(const Eigen::Vector3d& position, int ego_id,
    const std::vector<DroneState>& swarm_states, const list<Frontier>& frontiers) {
  // Ablation study
  if (fixed_) {
    return fix_role_;
  }

  //
  for (int i = 0; i < int(swarm_states.size()); ++i) {
    if (ego_id == i) continue;
    if (swarm_states[i].role_ == ROLE::GARBAGE_COLLECTOR &&
        (position - swarm_states[i].pos_).norm() <= params_->region_size) {
      return ROLE::EXPLORER;
    }
  }

  return assignRoleCloseByFrontiers(position, frontiers);
}

}  // namespace fast_planner
