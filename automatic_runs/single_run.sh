#!bash/usr/bin/env

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Input arguments
if [ "$#" -ne 5 ]; then
  echo -e "${RED}Wrong number of input arguments${NC}"
  return
fi

drone_num=${1}
planner_type=${2}
log_folder=${3}
model=${4}
vulkan_renderer=${5}

if [ "${planner_type}" != "racer" ]  && [ "${planner_type}" != "fame" ]; then
  echo -e "${RED}Unknown planner type: ${planner_type}${NC}"
  return
fi

# Parameters
visualize=true
communication_range=1000000.0  # Basically infinite communication

if [ ${vulkan_renderer} == true ]; then
  waiting_time=60.0
else
  waiting_time=15.0
fi

# Roscore
roscore &
sleep 5s

# Visualization
if [ ${visualize} == true ]; then
  roslaunch exploration_manager rviz.launch &
  sleep 2s
fi

# Planner
cmd_line_args="drone_num:=${drone_num} log_folder:=${log_folder} vulkan_renderer:=${vulkan_renderer} model:=${model} communication_range:=${communication_range}"

if [ ${planner_type} == "explorer" ]; then
  cmd_line_args="${cmd_line_args} role_assigner_fixed:=true planner_type:=fame"
elif [ ${planner_type} == "no_coll" ]; then
  cmd_line_args="${cmd_line_args} active_collaboration:=false planner_type:=fame"
else
  cmd_line_args="${cmd_line_args} planner_type:=${planner_type}"
fi

roslaunch exploration_manager swarm_exploration.launch ${cmd_line_args} &
sleep 2s

# Automatic trigger
roslaunch logging_utils trigger_node.launch waiting_time:=${waiting_time} &
sleep 1s

# Logging
roslaunch logging_utils logging_node.launch drone_num:=${drone_num} log_folder:=${log_folder} model:=${model}
sleep 1s

# Kill all
kill $(jobs -p)
rosnode kill -a && killall rosmaster

# Backup
cp ../swarm_exploration/exploration_manager/launch/single_drone_planner_${planner_type}.xml ${log_folder}

