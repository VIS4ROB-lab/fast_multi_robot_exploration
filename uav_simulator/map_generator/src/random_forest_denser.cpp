#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <pcl/search/kdtree.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <Eigen/Eigen>
#include <iostream>
#include <random>

using namespace std;

// pcl::search::KdTree<pcl::PointXYZ> kdtreeLocalMap;
pcl::KdTreeFLANN<pcl::PointXYZ> kdtreeLocalMap;
vector<int> pointIdxRadiusSearch;
vector<float> pointRadiusSquaredDistance;

random_device rd;
default_random_engine eng;
uniform_real_distribution<double> rand_x;
uniform_real_distribution<double> rand_y;
uniform_real_distribution<double> rand_w;
uniform_real_distribution<double> rand_h;
uniform_real_distribution<double> rand_local;

ros::Publisher _local_map_pub;
ros::Publisher _all_map_pub;
ros::Publisher click_map_pub_;
ros::Subscriber _odom_sub;

vector<double> _state;

int _obs_num, _occl_num;
double _x_size, _y_size, _z_size;
double min_x_, max_x_, min_y_, max_y_, _w_l, _w_h, _h_l, _h_h;
double local_sampling_radius_;
double _z_limit, _sensing_range, _resolution, _sense_rate, _init_x, _init_y;

bool _map_ok = false;
bool _has_odom = false;

sensor_msgs::PointCloud2 globalMap_pcd;
pcl::PointCloud<pcl::PointXYZ> cloudMap;

sensor_msgs::PointCloud2 localMap_pcd;
pcl::PointCloud<pcl::PointXYZ> clicked_cloud_;

void RandomMapGenerate() {
  pcl::PointXYZ pt_random;

  rand_x = uniform_real_distribution<double>(min_x_, max_x_);
  rand_y = uniform_real_distribution<double>(min_y_, max_y_);
  rand_w = uniform_real_distribution<double>(_w_l, _w_h);
  rand_h = uniform_real_distribution<double>(_h_l, _h_h);
  rand_local = uniform_real_distribution<double>(-local_sampling_radius_,
                                                 local_sampling_radius_);

  // generate polar obs
  std::vector<pair<double, double>> trees;
  for (int i = 0; i < _obs_num; i++) {
    double x, y, w, h;
    x = rand_x(eng);
    y = rand_y(eng);
    w = rand_w(eng);

    if (sqrt(pow(x - _init_x, 2) + pow(y - _init_y, 2)) < 4.0) {
      i--;
      continue;
    }

    x = floor(x / _resolution) * _resolution + _resolution / 2.0;
    y = floor(y / _resolution) * _resolution + _resolution / 2.0;
    trees.push_back({x, y});

    int widNum = ceil(w / _resolution);

    // Sample in the area around (x,y)

    for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
      for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
        h = rand_h(eng);
        int heiNum = ceil(h / _resolution);
        for (int t = -5; t < heiNum; t++) {
          pt_random.x = x + r * _resolution + 1e-2;
          pt_random.y = y + s * _resolution + 1e-2;
          pt_random.z = (t + 0.5) * _resolution + 1e-2;
          cloudMap.points.push_back(pt_random);
        }
      }
  }

  // Add occlusions
  for (const auto& tree : trees) {
    for (int i = 0; i < _occl_num; i++) {
      double x_local, y_local;
      x_local = rand_local(eng);
      y_local = rand_local(eng);

      double x = tree.first + x_local;
      double y = tree.second + y_local;

      double w = rand_w(eng);
      int widNum = ceil(w / _resolution);

      for (int r = -widNum / 2.0; r < widNum / 2.0; r++)
        for (int s = -widNum / 2.0; s < widNum / 2.0; s++) {
          double h = rand_h(eng);
          int heiNum = ceil(h / _resolution);
          for (int t = -5; t < heiNum; t++) {
            pt_random.x = x + r * _resolution + 1e-2;
            pt_random.y = y + s * _resolution + 1e-2;
            pt_random.z = (t + 0.5) * _resolution + 1e-2;
            cloudMap.points.push_back(pt_random);
          }
        }
    }
  }

  // add ground
  for (double x = min_x_; x < max_x_; x += _resolution) {
    for (double y = min_y_; y < max_y_; y += _resolution) {
      pt_random.x = x;
      pt_random.y = y;
      pt_random.z = -0.1;
      cloudMap.push_back(pt_random);
    }
  }

  cloudMap.width = cloudMap.points.size();
  cloudMap.height = 1;
  cloudMap.is_dense = true;

  ROS_WARN("Finished generate random map ");

  kdtreeLocalMap.setInputCloud(cloudMap.makeShared());

  _map_ok = true;
}

void rcvOdometryCallbck(const nav_msgs::Odometry odom) {
  if (odom.child_frame_id == "X" || odom.child_frame_id == "O")
    return;
  _has_odom = true;

  _state = {odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z,
            odom.twist.twist.linear.x,
            odom.twist.twist.linear.y,
            odom.twist.twist.linear.z,
            0.0,
            0.0,
            0.0};
}

int i = 0;
void pubSensedPoints() {
  // if (i < 10) {
  pcl::toROSMsg(cloudMap, globalMap_pcd);
  globalMap_pcd.header.frame_id = "world";
  _all_map_pub.publish(globalMap_pcd);
  // }

  return;

  /* ---------- only publish points around current position ---------- */
  if (!_map_ok || !_has_odom)
    return;

  pcl::PointCloud<pcl::PointXYZ> localMap;

  pcl::PointXYZ searchPoint(_state[0], _state[1], _state[2]);
  pointIdxRadiusSearch.clear();
  pointRadiusSquaredDistance.clear();

  pcl::PointXYZ pt;

  if (isnan(searchPoint.x) || isnan(searchPoint.y) || isnan(searchPoint.z))
    return;

  if (kdtreeLocalMap.radiusSearch(searchPoint, _sensing_range,
                                  pointIdxRadiusSearch,
                                  pointRadiusSquaredDistance) > 0) {
    for (size_t i = 0; i < pointIdxRadiusSearch.size(); ++i) {
      pt = cloudMap.points[pointIdxRadiusSearch[i]];
      localMap.points.push_back(pt);
    }
  } else {
    ROS_ERROR("[Map server] No obstacles .");
    return;
  }

  localMap.width = localMap.points.size();
  localMap.height = 1;
  localMap.is_dense = true;

  pcl::toROSMsg(localMap, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  _local_map_pub.publish(localMap_pcd);
}

void clickCallback(const geometry_msgs::PoseStamped& msg) {
  double x = msg.pose.position.x;
  double y = msg.pose.position.y;
  // double w = rand_w(eng);
  double w = 0.6;
  double h;
  pcl::PointXYZ pt_random;
  std::cout << "click: " << x << ", " << y << std::endl;

  x = floor(x / 0.5) * 0.5 + 0.5 / 2.0;
  y = floor(y / 0.5) * 0.5 + 0.5 / 2.0;

  double res2 = _resolution * 0.5;
  int wid_num2 = ceil(w / res2);

  for (int r = -wid_num2 / 2.0; r < wid_num2 / 2.0; r++)
    for (int s = -wid_num2 / 2.0; s < wid_num2 / 2.0; s++) {
      h = rand_h(eng);
      int heiNum = ceil(2.5 / res2);
      int hei_num2 = ceil(1 / res2);
      for (int t = -hei_num2; t < heiNum; t++) {
        pt_random.x = x + (r + 0.5) * res2 + 1e-2;
        pt_random.y = y + (s + 0.5) * res2 + 1e-2;
        pt_random.z = (t + 0.5) * res2 + 1e-2;
        clicked_cloud_.points.push_back(pt_random);
        cloudMap.points.push_back(pt_random);
      }
    }
  clicked_cloud_.width = clicked_cloud_.points.size();
  clicked_cloud_.height = 1;
  clicked_cloud_.is_dense = true;

  pcl::toROSMsg(clicked_cloud_, localMap_pcd);
  localMap_pcd.header.frame_id = "world";
  click_map_pub_.publish(localMap_pcd);

  cloudMap.width = cloudMap.points.size();

  return;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "random_map_sensing");
  ros::NodeHandle n("~");

  _local_map_pub =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/local_cloud", 1);
  _all_map_pub =
      n.advertise<sensor_msgs::PointCloud2>("/map_generator/global_cloud", 1);

  _odom_sub = n.subscribe("odometry", 50, rcvOdometryCallbck);

  click_map_pub_ =
      n.advertise<sensor_msgs::PointCloud2>("/pcl_render_node/local_map", 1);
  ros::Subscriber click_sub =
      n.subscribe("/move_base_simple/goal", 10, clickCallback);

  n.param("init_state_x", _init_x, 0.0);
  n.param("init_state_y", _init_y, 0.0);

  n.param("map/x_size", _x_size, 50.0);
  n.param("map/y_size", _y_size, 50.0);
  n.param("map/z_size", _z_size, 5.0);
  n.param("map/obs_num", _obs_num, 30);
  n.param("map/occl_num", _occl_num, 5);
  n.param("map/resolution", _resolution, 0.1);
  n.param("map/local_sampling_radius", local_sampling_radius_, 2.0);

  n.param("ObstacleShape/lower_rad", _w_l, 0.3);
  n.param("ObstacleShape/upper_rad", _w_h, 0.8);
  n.param("ObstacleShape/lower_hei", _h_l, 3.0);
  n.param("ObstacleShape/upper_hei", _h_h, 7.0);

  n.param("sensing/radius", _sensing_range, 10.0);
  n.param("sensing/rate", _sense_rate, 10.0);

  min_x_ = -_x_size / 2.0;
  max_x_ = +_x_size / 2.0;

  min_y_ = -_y_size / 2.0;
  max_y_ = +_y_size / 2.0;

  _obs_num = min(_obs_num, (int)_x_size * 10);
  _z_limit = _z_size;

  ros::Duration(0.5).sleep();

  // init random device
  int seed;
  n.param("ObstacleShape/seed", seed, -1);
  if (seed < 0) {
    seed = rd() % INT32_MAX;
  }
  std::cout << "map seed: " << seed << std::endl;

  eng = default_random_engine(seed);

  RandomMapGenerate();

  ros::Rate loop_rate(_sense_rate);
  int loop_num = 0;

  while (ros::ok()) {
    pubSensedPoints();
    ros::spinOnce();
    loop_rate.sleep();
    ROS_WARN_THROTTLE(2, "seed: %d", seed);
  }
}
