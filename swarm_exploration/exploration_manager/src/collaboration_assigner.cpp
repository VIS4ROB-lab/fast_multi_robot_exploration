#include <exploration_manager/collaboration_assigner.h>

#include <nlopt.hpp>

namespace fast_planner {

CollaborationAssigner::CollaborationAssigner(const ros::NodeHandle& nh) {
  params_.reset(new CollaborationParams);
  nh.param("collab_assigner/dist_range", params_->dist_range, 15.0);
  nh.param("collab_assigner/dist_collision", params_->dist_collision, 5.0);
  nh.param("collab_assigner/w_range", params_->w_range, 0.6);
  nh.param("collab_assigner/w_collision", params_->w_collision, 0.3);
  nh.param("collab_assigner/active", active_, true);

  d_star_ =
      (params_->w_collision * params_->dist_collision + params_->w_range * params_->dist_range) /
      (params_->w_collision + params_->w_range);
}

CollaborationAssigner::~CollaborationAssigner() {
}

bool CollaborationAssigner::optimizePositions(const Eigen::Vector3d& x1, const Eigen::Vector3d& x2,
    Eigen::Vector3d& x1_opt, Eigen::Vector3d& x2_opt) {

  // For ablation study
  if (!active_) {
    x1_opt = x1;
    x2_opt = x2;
    return true;
  }

  // Initialize optimized variables to initial values
  x1_opt = x1;
  x2_opt = x2;

  // State to optimize
  std::vector<double> x = { x1.x(), x1.y(), x2.x(), x2.y() };

  // Optimizer
  nlopt::opt opt(nlopt::LD_MMA, 4);
  opt.set_min_objective(CollaborationAssigner::distanceCost, &d_star_);

  const double tol = 1e-5;
  opt.set_xtol_rel(tol);

  const double max_time = 1.;  // [s]
  opt.set_maxtime(max_time);

  try {
    std::vector<double> grad(4, 0.0);
    double initial_cost = this->distanceCost(x, grad, &d_star_);

    double final_cost;
    nlopt::result result = opt.optimize(x, final_cost);
    std::cout << "Optimization Result: " << result << std::endl;
    std::cout << "Cost went from " << initial_cost << " to " << final_cost << std::endl;
  } catch (std::exception& e) {
    std::cout << e.what() << std::endl;
    return false;
  }

  // Optimized variables
  x1_opt[0] = x[0];
  x1_opt[1] = x[1];

  x2_opt[0] = x[2];
  x2_opt[1] = x[3];

  std::cout << "Agent 1 from " << x1.transpose() << " to " << x1_opt.transpose() << std::endl;
  std::cout << "Agent 2 from " << x2.transpose() << " to " << x2_opt.transpose() << std::endl;

  return true;
}

double CollaborationAssigner::distanceCost(
    const std::vector<double>& x, std::vector<double>& grad, void* func_data) {

  // Pre-allocation
  const double d_star = *reinterpret_cast<double*>(func_data);
  const double d_star2 = d_star * d_star;
  const double d_star4 = d_star2 * d_star2;

  double distance2 = (x[0] - x[2]) * (x[0] - x[2]) + (x[1] - x[3]) * (x[1] - x[3]);
  const double factor = 1. / std::sqrt(distance2) - d_star * d_star2 / (distance2 * distance2);

  // Gradient
  if (grad.empty()) grad.resize(x.size(), 0.0);
  grad[0] = (x[0] - x[2]) * d_star * factor;
  grad[1] = (x[1] - x[3]) * d_star * factor;
  grad[2] = -grad[0];
  grad[3] = -grad[1];

  // Cost
  return d_star * std::sqrt(distance2) + 0.5 * d_star4 / distance2 - 3. / 2. * d_star2;
}

}  // end namespace fast_planner