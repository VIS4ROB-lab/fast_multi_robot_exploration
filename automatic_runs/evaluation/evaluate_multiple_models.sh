#!bash/usr/bin/env

# User input
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Input arguments
if [ "$#" -ne 2 ]; then
  echo -e "${RED}Wrong number of input arguments${NC}"
  return
fi

folder=${1}
num_runs=${2}
max_n_drones=10

for dir in ${folder}/*/     # list directories in the form "/tmp/dirname/"
do
    # Experiment folder
    dir=${dir%*/}      # remove the trailing "/"
    exp_folder="${folder}${dir##*/}"

    # Summary files
    for (( n_drones=1; n_drones<=${max_n_drones}; n_drones++ ))
    do

        eval_folder="${exp_folder}/${n_drones}_drones"

        echo "Number of drones: ${n_drones}"
        python3 ../../swarm_exploration/exploration_manager/scripts/plot_series.py -f ${eval_folder} --num-agents ${n_drones} --num-runs ${num_runs}

    done

done
