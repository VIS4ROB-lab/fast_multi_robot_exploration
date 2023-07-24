#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "trigger_node");

  ros::NodeHandle nh("~");
  ros::Publisher pub =
      nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 100);

  double waiting_time = nh.param("waiting_time", 15.0);
  ros::Time start_time = ros::Time::now();
  ros::Rate rate(100);
  while (ros::ok() && (ros::Time::now() - start_time).toSec() < waiting_time) {
    ros::spinOnce();
    rate.sleep();
  }

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "world";
  pose.pose.orientation.w = 1.0;

  ROS_INFO("Triggering!");

  ros::Rate rate_pub(5);
  for (int i = 0; i < 5; ++i) {
    pub.publish(pose);
    ros::spinOnce();
    rate_pub.sleep();
  }

  return 0;
}
