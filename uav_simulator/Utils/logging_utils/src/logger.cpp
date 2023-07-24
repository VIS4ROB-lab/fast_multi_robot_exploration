#include <logging_utils/logger.h>

#include <boost/filesystem.hpp>

namespace logging {

Logger::Logger(const ros::NodeHandle& nh) : nh_(nh), start_logging_(false) {
  // Parameters
  int drone_num;
  nh_.param("drone_num", drone_num, -1);
  assert(drone_num >= 1);
  ROS_INFO("[Logger] Initializing with %d drones", drone_num);

  std::string odom_topic;
  nh_.param("odom_topic", odom_topic, std::string(""));

  // Logging Parameters
  nh_.param("odometry_rate", params_.odometry_rate, 10.);
  nh_.param("logging_rate", params_.logging_rate, 200.);
  nh_.param("max_allowed_time", params_.max_allowed_time,
            2700.);  // Default: 45 min
  nh_.param("max_heartbit_time", params_.max_heartbit_time,
            60.);  // Default: 1 min

  // Info
  ROS_INFO("Max time for the experiment: %1.1f s", params_.max_allowed_time);
  ROS_INFO("Max time for heartbit: %1.1f s", params_.max_heartbit_time);

  // Output paths
  nh.param("log_folder", params_.log_folder, std::string(""));
  assert(!params_.log_folder.empty());

  using namespace boost::filesystem;
  if (exists(params_.log_folder)) {
    ROS_WARN(
        "[Logger] Output folder already exists. Adding to existing folder");
  } else {
    if (!exists(params_.log_folder))
      create_directories(params_.log_folder);
  }

  ROS_INFO("[Logger] Output folder: %s", params_.log_folder.c_str());
  std::string output_path_odometry = params_.log_folder + "/odom_";

  // Subscribers
  trigger_sub_ = nh_.subscribe<geometry_msgs::PoseStamped>(
      "/move_base_simple/goal", 1, &Logger::triggerCallback, this);

  stop_logging_.resize(drone_num, false);
  stop_times_.resize(drone_num, ros::Time::now());
  last_hearbit_times_.resize(drone_num, ros::Time::now());
  stop_codes_.resize(drone_num, 0);
  velocities_.resize(drone_num, std::make_pair<size_t, double>(0, 0.0));
  distances_.resize(drone_num, 0.0);
  prev_pos_.resize(drone_num, Eigen::Vector3d::Zero());
  for (size_t i = 1; i <= size_t(drone_num); ++i) {
    // ROS
    odom_subs_.push_back(nh_.subscribe<nav_msgs::Odometry>(
        "/" + odom_topic + "_" + std::to_string(i), 1,
        boost::bind(&Logger::odometryCallback, this, _1, i)));

    stop_subs_.push_back(nh_.subscribe<std_msgs::Int32>(
        "/stop_" + std::to_string(i), 100,
        boost::bind(&Logger::stopCallback, this, _1, i), ros::VoidConstPtr(),
        ros::TransportHints().tcpNoDelay()));

    heartbit_subs_.push_back(nh_.subscribe<std_msgs::Empty>(
        "/heartbit_" + std::to_string(i), 100,
        boost::bind(&Logger::heartbitCallback, this, _1, i),
        ros::VoidConstPtr(), ros::TransportHints().tcpNoDelay()));

    // Logging
    odom_files_.push_back(output_path_odometry + std::to_string(i) + ".csv");
  }
}

Logger::~Logger() {}

void Logger::run() {
  ros::Rate rate(params_.logging_rate);

  bool all_stopped = false;
  while (ros::ok() && !all_stopped) {
    ros::spinOnce();
    rate.sleep();

    all_stopped = std::all_of(stop_logging_.begin(), stop_logging_.end(),
                              [](bool v) { return v; });

    // Check if we have reached the maximum allowed time - if so, stop the timer
    if (start_logging_ &&
        (ros::Time::now() - start_time_).toSec() >= params_.max_allowed_time) {
      // Print info
      ROS_WARN("Maximum time reached!");

      // Set a fake stop time to all agents that have not yet finished
      for (size_t i = 0; i < stop_logging_.size(); ++i) {
        if (!stop_logging_[i]) {
          stop_times_[i] = ros::Time::now();
          stop_codes_[i] = -2;
        }
      }
      // Exit the loop
      break;
    }

    // Check if any of the planners crashed
    if (start_logging_) {
      bool planner_crashed = std::any_of(
          last_hearbit_times_.begin(), last_hearbit_times_.end(),
          [&](const ros::Time& t) {
            return (ros::Time::now() - t).toSec() > params_.max_heartbit_time;
          });

      // Set a fake stop time to all agents that have not yet finished
      if (planner_crashed) {
        ROS_ERROR("A planner crashed!");
        for (size_t i = 0; i < stop_logging_.size(); ++i) {
          if (!stop_logging_[i]) {
            stop_times_[i] = ros::Time::now();
            stop_codes_[i] = -3;
          }
        }
        // Exit the loop
        break;
      }
    }

    // Check if any planner is in collision
    if (start_logging_) {
      bool drone_crashed =
          std::any_of(stop_codes_.begin(), stop_codes_.end(),
                      [](const int code) { return code == -1; });

      // Set a fake stop time to all agents that have not yet finished
      if (drone_crashed) {
        ROS_ERROR("A drone crashed against an obstacle!");
        for (size_t i = 0; i < stop_logging_.size(); ++i) {
          if (!stop_logging_[i]) {
            stop_times_[i] = ros::Time::now();
            stop_codes_[i] = -1;
          }
        }
        // Exit the loop
        break;
      }
    }
  }

  ROS_WARN("[Logger] Shutting down");

  // Generate final timings
  std::fstream file;
  file.open(params_.log_folder + "/summary_times.csv", std::ios::app);
  file << std::setprecision(5)
       << "Agent,Exit Code,Time[s],Velocity[m/s],Distance[m]\n";
  for (size_t i = 0; i < stop_times_.size(); ++i) {
    double elapsed_time = (stop_times_[i] - start_time_).toSec();
    double avg_velocity = velocities_[i].second / double(velocities_[i].first);
    file << i << ", " << stop_codes_[i] << ", " << elapsed_time << ", "
         << avg_velocity << ", " << distances_[i] << "\n";
    ROS_INFO("[Logger] Drone %ld took %1.3f s with avg velocity of %1.2f m/s",
             i, elapsed_time, avg_velocity);
  }
  file.close();
}

void Logger::odometryCallback(const nav_msgs::OdometryConstPtr& odom_msg,
                              size_t drone_id) {
  if (!start_logging_) {
    prev_pos_[drone_id - 1] = Eigen::Vector3d(odom_msg->pose.pose.position.x,
                                              odom_msg->pose.pose.position.y,
                                              odom_msg->pose.pose.position.z);
    return;
  } else {
    ROS_INFO_ONCE("[Logger] Starting logging of odometry");
  }

  // If we have stopped, just return
  if (stop_logging_[drone_id - 1]) {
    return;
  }

  // Throttling
  static ros::Time time = ros::Time::now();
  if ((ros::Time::now() - time).toSec() < 1. / params_.odometry_rate) {
    return;
  }

  // Store velocity info
  velocities_[drone_id - 1].first += 1;
  velocities_[drone_id - 1].second += std::sqrt(
      odom_msg->twist.twist.linear.x * odom_msg->twist.twist.linear.x +
      odom_msg->twist.twist.linear.y * odom_msg->twist.twist.linear.y +
      odom_msg->twist.twist.linear.z * odom_msg->twist.twist.linear.z);

  // Store distance
  Eigen::Vector3d curr_pos(odom_msg->pose.pose.position.x,
                           odom_msg->pose.pose.position.y,
                           odom_msg->pose.pose.position.z);
  distances_[drone_id - 1] += (curr_pos - prev_pos_[drone_id - 1]).norm();
  prev_pos_[drone_id - 1] = curr_pos;

  // Store to file
  time = ros::Time::now();

  std::fstream file;
  file.open(odom_files_[drone_id - 1], std::ios::app);
  file << std::setprecision(20) << odom_msg->header.stamp.toSec() << ","
       << odom_msg->pose.pose.position.x << ","
       << odom_msg->pose.pose.position.y << ","
       << odom_msg->pose.pose.position.z << ","
       << odom_msg->pose.pose.orientation.w << ","
       << odom_msg->pose.pose.orientation.x << ","
       << odom_msg->pose.pose.orientation.y << ","
       << odom_msg->pose.pose.orientation.z << ","
       << odom_msg->twist.twist.linear.x << ","
       << odom_msg->twist.twist.linear.y << ","
       << odom_msg->twist.twist.linear.z << ","
       << odom_msg->twist.twist.angular.x << ","
       << odom_msg->twist.twist.angular.y << ","
       << odom_msg->twist.twist.angular.z << "\n";
  file.close();
}

void Logger::triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ROS_INFO("[Logger] Start logging");

  start_logging_ = true;
  start_time_ = ros::Time::now();
}

void Logger::stopCallback(const std_msgs::Int32ConstPtr& msg, size_t drone_id) {
  // If we have already stopped, then do nothing
  if (stop_logging_[drone_id - 1])
    return;

  // Collect stop time
  ROS_INFO("[Logger] Stopping logging for drone %ld", drone_id);

  // RACER is using 1-based index
  stop_logging_[drone_id - 1] = true;
  stop_times_[drone_id - 1] = ros::Time::now();
  stop_codes_[drone_id - 1] = msg->data;
}

void Logger::heartbitCallback(const std_msgs::EmptyConstPtr& msg,
                              size_t drone_id) {
  last_hearbit_times_[drone_id - 1] = ros::Time::now();
}

}  // namespace logging