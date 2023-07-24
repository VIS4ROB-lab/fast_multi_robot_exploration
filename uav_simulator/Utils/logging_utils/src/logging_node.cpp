#include <logging_utils/logger.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "logging_node");
  ros::NodeHandle n("~");

  logging::Logger logger(n);
  logger.run();
}