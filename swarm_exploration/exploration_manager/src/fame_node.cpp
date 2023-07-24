#include <ros/ros.h>
#include <exploration_manager/fame_exploration_fsm.h>

#include <plan_manage/backward.hpp>
namespace backward {
backward::SignalHandling sh;
}

using namespace fast_planner;

int main(int argc, char** argv) {
  ros::init(argc, argv, "fame_node");
  ros::NodeHandle nh("~");

  FameExplorationFSM expl_fame;
  expl_fame.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}
