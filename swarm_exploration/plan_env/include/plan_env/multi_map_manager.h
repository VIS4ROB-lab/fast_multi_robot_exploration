#ifndef _MULTI_MAP_MANAGER_H
#define _MULTI_MAP_MANAGER_H

#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <plan_env_msgs/ChunkStamps.h>
#include <plan_env_msgs/ChunkData.h>

#include <memory>
#include <random>
#include <vector>
#include <unordered_map>
#include <boost/filesystem.hpp>

using std::shared_ptr;
using std::unordered_map;
using std::vector;

namespace fast_planner {
class SDFMap;
class MapROS;

// A map chunk, the elementary exchange unit between robots
struct MapChunk {
  // double stamp_;
  uint32_t idx_;  // Start from 1
  vector<uint32_t> voxel_adrs_;
  vector<uint8_t> voxel_occ_;

  bool need_query_;
  bool empty_;
};

struct ChunksData {
  vector<MapChunk> chunks_;
  // uint32_t latest_idx_;
  vector<int> idx_list_;
  // double latest_stamp_;
};

struct ChunksBox {
  Eigen::Vector3d min_;
  Eigen::Vector3d max_;
  bool valid_;
};

// Utility for logging explored volume over time
class MapLogger {
public:
  MapLogger(const std::string& log_folder, int drone_id) {
    // Create log folder if necessary
    using namespace boost::filesystem;
    if (!exists(log_folder)) {
      create_directories(log_folder);
    }

    log_file_path_ = log_folder + "/explored_volume_" + std::to_string(drone_id) + ".csv";
  }

  ~MapLogger() {
  }

  void log(double timestamp, double explored_volume) {
    std::fstream log_file;
    log_file.open(log_file_path_, std::ios::app);
    log_file << std::setprecision(10) << timestamp << "," << explored_volume << "\n";
    log_file.close();
  }

protected:
  std::string log_file_path_;

public:
  typedef shared_ptr<MapLogger> Ptr;
};

class MultiMapManager {
public:
  MultiMapManager();
  ~MultiMapManager();
  void setMap(SDFMap* map);
  void init();

  void updateMapChunk(const vector<uint32_t>& adrs);
  void getChunkBoxes(
      vector<Eigen::Vector3d>& mins, vector<Eigen::Vector3d>& maxs, bool reset = true);

private:
  void sendChunks(const int& chunk_drone_id, const int& to_drone_id, const vector<int>& idx_list);
  void getOccOfChunk(const vector<uint32_t>& adrs, vector<uint8_t>& occs);
  void insertChunkToMap(const MapChunk& chunk, const int& chunk_drone_id);
  void adrToIndex(const uint32_t& adr, Eigen::Vector3i& idx);

  void stampTimerCallback(const ros::TimerEvent& e);
  void chunkTimerCallback(const ros::TimerEvent& e);
  void loggingTimerCallback(const ros::TimerEvent& e);
  void stampMsgCallback(const plan_env_msgs::ChunkStampsConstPtr& msg);
  void chunkCallback(const plan_env_msgs::ChunkDataConstPtr& msg);

  // Operations on the chunk idx list
  void findMissedChunkIds(
      const vector<int>& self_idx_list, const vector<int>& other_idx_list, vector<int>& miss_ids);
  bool findIntersect(
      const int& min1, const int& max1, const int& min2, const int max2, int& minr, int& maxr);
  void mergeChunkIds(const vector<int>& list1, const vector<int>& list2, vector<int>& output);

  // data----------------

  int drone_id_, map_num_;
  int vis_drone_id_;  // ONLY use for ground node!
  int chunk_size_;

  SDFMap* map_;
  ros::NodeHandle node_;
  ros::Publisher stamp_pub_, chunk_pub_, marker_pub_;
  ros::Subscriber stamp_sub_, chunk_sub_;
  ros::Timer stamp_timer_, chunk_timer_, logging_timer_;

  vector<ChunksData> multi_map_chunks_;                    // Main map data
  vector<uint32_t> adr_buffer_;                            // Buffer for chunks of this map
  vector<vector<plan_env_msgs::ChunkData>> chunk_buffer_;  // Buffer for chunks of external map
  vector<unordered_map<int, char>> buffer_map_;  // Hash map to avoid repeated insertion of chunk
                                                 // msg
  vector<double> last_chunk_stamp_time_;

  // Bounding box of map chunks of swarm
  vector<ChunksBox> chunk_boxes_;

  vector<Eigen::Vector3i> tmp_ids_;
  Eigen::Vector3d drone_pos_;

  // Parameters
  double communication_range_;

  // Logging
  MapLogger::Ptr logger_;

  friend SDFMap;
  friend MapROS;
};
}  // namespace fast_planner

#endif