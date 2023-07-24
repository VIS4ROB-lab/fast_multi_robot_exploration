#!bash/usr/bin/env

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Input arguments
if [ "$#" -ne 1 ]; then
  echo -e "${RED}Wrong number of input arguments${NC}"
  return
fi

num_runs=${1}

# Parameters
num_planners=2
output_folder_date=$(date +'%F_%H:%M:%S')

# Clean up
rosclean purge -y

# Iterate over models and number of agents
for (( model_id=1; model_id<=4; model_id++ ))
do

  if [ ${model_id} == 1 ]; then
    model="forest_50x50_01_200"
  elif [ ${model_id} == 2 ]; then
    model="forest_50x50_01_300"
  elif [ ${model_id} == 3 ]; then
    model="forest_4_densities_50_40_20_10"
  elif [ ${model_id} == 4 ]; then
    model="forest_50x50_100_denser_3"
  else
    continue
  fi

  for (( drone_num=1; drone_num<=10; drone_num++ ))
  do
    if [ ${drone_num} == 5 ]; then
      continue
    elif [ ${drone_num} == 7 ]; then
      continue
    elif [ ${drone_num} == 9 ]; then
      continue
    fi

    # Logging folder
    log_folder_top="${HOME}/exp_logs/${output_folder_date}/${model}/${drone_num}_drones"

    for (( i=1; i<=${num_planners}; i++ ))
    do  
      # Planner Type
      if [ ${i} == 1 ]; then
        planner_type="racer"
      elif [ ${i} == 2 ]; then
        planner_type="fame"
      else
        continue
      fi

      # Individual runs
      for (( j=1; j<=${num_runs}; j++ ))
      do
        # Clean up
        rosclean purge -y

        # Logging folder
        log_folder_run="${log_folder_top}/${planner_type}/run_${j}"

        # Run Experiment
        . single_run.sh ${drone_num} ${planner_type} ${log_folder_run} ${model} false
        sleep 5s
        
      done

    done

    # Plotting
    python3 ../swarm_exploration/exploration_manager/scripts/plot_series.py -f ${log_folder_top} --num-agents ${drone_num} --num-runs ${num_runs}

  done

done
