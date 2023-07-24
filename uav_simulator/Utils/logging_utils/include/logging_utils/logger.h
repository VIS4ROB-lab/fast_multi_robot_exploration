#ifndef _LOGGER_H_
#define _LOGGER_H_

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>

#include <Eigen/Dense>

namespace logging {

struct LoggingParams {
  double logging_rate;
  double odometry_rate;
  double max_allowed_time;
  double max_heartbit_time;

  std::string log_folder;
};

class Logger {
  public:
  Logger(const ros::NodeHandle& nh);
  ~Logger();

  void run();

  private:
  void odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                        size_t drone_id);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void stopCallback(const std_msgs::Int32ConstPtr& msg, size_t drone_id);
  void heartbitCallback(const std_msgs::EmptyConstPtr& msg, size_t drone_id);

  protected:
  ros::NodeHandle nh_;
  std::vector<ros::Subscriber> odom_subs_, stop_subs_, heartbit_subs_;
  ros::Subscriber trigger_sub_;

  LoggingParams params_;

  bool start_logging_;
  std::vector<bool> stop_logging_;

  ros::Time start_time_;
  std::vector<ros::Time> stop_times_, last_hearbit_times_;
  std::vector<int> stop_codes_;

  // Logging of data
  std::vector<std::pair<size_t, double>> velocities_;
  std::vector<double> distances_;
  std::vector<Eigen::Vector3d> prev_pos_;

  // Files
  std::vector<std::string> odom_files_;
};

}  // end namespace logging

#endif
