#include <gtest/gtest.h>

#include <exploration_manager/collaboration_assigner.h>
#include <nlopt.hpp>

using namespace fast_planner;

TEST(CollaborationCostTest, TestJacobian) {
  // Set up the problem
  Eigen::Vector3d x1(10., 4., 0.);
  Eigen::Vector3d x2(3., 8., 0.);
  double d_star = 5.;

  // Get the analytical jacobian
  std::vector<double> x = { x1[0], x1[1], x2[0], x2[1] };
  std::vector<double> analytical_jacobian(4, 0.0);
  double cost_init = CollaborationAssigner::distanceCost(x, analytical_jacobian, &d_star);

  // Get the numerical jacobian
  double delta = 0.0001;
  std::vector<double> numerical_jacobian(4, 0.0);

  for (size_t i = 0; i < 4; ++i) {
    std::vector<double> x_delta = x;
    x_delta[i] += delta;

    std::vector<double> tmp_jacobian(4, 0.0);
    double cost_delta = CollaborationAssigner::distanceCost(x_delta, tmp_jacobian, &d_star);
    numerical_jacobian[i] = (cost_delta - cost_init) / delta;
  }

  // Print Jacobians
  // for (size_t i = 0; i < numerical_jacobian.size(); ++i) {
  //     std::cout << i << ": " << numerical_jacobian[i] << " vs " << analytical_jacobian[i] <<
  //     std::endl;
  // }

  // Check if the jacobians are similar
  double tolerance = 1e-3;
  for (size_t i = 0; i < numerical_jacobian.size(); ++i) {
    ASSERT_NEAR(numerical_jacobian[i], analytical_jacobian[i], tolerance);
  }
}

TEST(CollaborationCostTest, TestOptimization) {
  // Auxiliary
  srand(time(NULL));
  auto generateRandomNumber = [](double fMin, double fMax) {
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
  };

  // State to optimize
  std::vector<double> x(4);
  for (size_t i = 0; i < x.size(); ++i) {
    x[i] = generateRandomNumber(0., 20.);
  }

  // Parameters
  double d_star = generateRandomNumber(0., 10.);

  // Optimizer
  nlopt::opt opt(nlopt::LD_MMA, 4);
  opt.set_min_objective(CollaborationAssigner::distanceCost, &d_star);
  opt.set_xtol_rel(1e-5);

  // Optimization
  double final_cost;
  nlopt::result result = opt.optimize(x, final_cost);
  ASSERT_GE(result, 0);

  // Print optimized state
  // std::cout << "Opt x1: [" << x[0] << "," << x[1] << "]" << std::endl;
  // std::cout << "Opt x2: [" << x[2] << "," << x[3] << "]" << std::endl;

  // Distance should be equal to d_star
  double distance = sqrt(pow(x[0] - x[2], 2.) + pow(x[1] - x[3], 2.));
  ASSERT_NEAR(distance, d_star, 1e-3);
  // std::cout << "Distance: " << distance << std::endl;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}