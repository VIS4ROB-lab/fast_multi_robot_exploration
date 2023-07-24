"""
    Script that generates the plot of the explored volume over multiple team sizes

    Read the folders inside the log folder
    Expected structure:
    log_dir
    |- planner_1
    |   |- run_1
    |   |- run_2
    |- planner_2
        |-....

    Author: Luca Bartolomei, V4RL
"""

import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

import argparse
from pathlib import Path
import pandas as pd
import numpy as np
from matplotlib import pyplot as plt

try:
    import scienceplots
    plt.style.use(['custom'])
except ImportError:
    pass


VALID_NUM_AGENTS = [1, 2, 3, 4, 6, 8, 10]

def average_series_per_agent(data_raw: dict):

    # Store the data of each run per agent
    data_per_agent = []

    for agent_id in range(len(data_raw)):

        # Store the data of each run per agent
        combined_series, stamp_series = pd.DataFrame(), pd.DataFrame()
        keys = []

        for run_id in range(len(data_raw[0])):

            # Process valid runs
            keys.append(str(run_id))
            run_data = data_raw[agent_id][run_id]

            if len(combined_series.keys()) == 0:
                combined_series = pd.DataFrame({str(run_id): run_data[:, 1]})
                stamp_series = pd.DataFrame({str(run_id): run_data[:, 0]})
            else:
                combined_series = pd.merge(combined_series, pd.DataFrame({str(run_id): run_data[:, 1]}),
                                            suffixes=('_' + str(run_id - 1), '_' + str(run_id)),
                                            left_index=True, right_index=True, how='outer')
                stamp_series = pd.merge(stamp_series, pd.DataFrame({str(run_id): run_data[:, 0]}),
                                        suffixes=('_' + str(run_id - 1), '_' + str(run_id)),
                                        left_index=True, right_index=True, how='outer')

        combined_series = combined_series.interpolate()
        stamp_series = stamp_series.interpolate()
        data_per_agent.append(
            {
                'avg': combined_series[keys].mean(axis=1), 
                'std': combined_series[keys].std(axis=1),
                'time': stamp_series[keys].mean(axis=1), 
            }
        )

    return data_per_agent


def parse_args() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', '-f', help="Path to the input folder", required=True, type=str)
    return parser.parse_args()


def main():
    # Parse argument and check that the folder exists
    args = parse_args()

    log_dir = Path(args.folder).expanduser()
    if not log_dir.exists():
        print("\033[93mSpecify existing input folder\033[0m")
        return

    data_plot = {}
    for num_agents in VALID_NUM_AGENTS:

        drones_folder = log_dir.joinpath(f"{num_agents}_drones")
        if not drones_folder.exists(): continue

        for planner_folder in drones_folder.iterdir():

            # Skip files
            if planner_folder.is_file(): continue

            # Plot only FAME
            if planner_folder.name != "fame": continue

            # Get the data for plotting
            runs_folders = [f for f in planner_folder.iterdir() if f.is_dir()]

            data_raw = {i: [] for i in range(num_agents)}           
            for _, run_folder in enumerate(runs_folders):

                # Read files with timings, exploration rate and odometry info
                times = pd.read_csv(run_folder.joinpath('summary_times.csv')).to_numpy()
                assert num_agents == times.shape[0], "Number of agents does not match"

                # Check if the run is a success
                assert all(times[agent_id, 1] >= 0 for agent_id in range(num_agents))

                # Process info
                for i in range(num_agents):
                    stamped_volume = pd.read_csv(run_folder.joinpath(f'explored_volume_{i + 1}.csv'), header=None).to_numpy()
                    data_raw[i].append(stamped_volume)

            # Summarize data for number of agents
            data_raw = average_series_per_agent(data_raw)
            data_plot[num_agents] = {
                'avg': np.mean( np.array([ data_raw[i]['avg'] for i in range(len(data_raw)) ]), axis=0 ),
                'std': np.mean( np.array([ data_raw[i]['std'] for i in range(len(data_raw)) ]), axis=0 ),
                'time': np.mean( np.array([ data_raw[i]['time'] for i in range(len(data_raw)) ]), axis=0 ),
            }

    # Average the data per agent

    # Normalize the data in the range [0, max(last element)]
    max_last = -np.inf
    for data in data_plot.items():
        max_last = max(max_last, data[1]['avg'][-1])

    # Generate summary for timings and plots for explored volume
    plt.figure(1)
    for data in data_plot.items():
        # Label
        num_agents = data[0]
        label = f"{num_agents} drones" if num_agents > 1 else f"{num_agents} drone"

        # Remove nans
        valid_ids = np.isfinite(data[1]['avg'])
        data[1]['avg'] = data[1]['avg'][valid_ids]
        data[1]['std'] = data[1]['std'][valid_ids]
        data[1]['time'] = data[1]['time'][valid_ids]

        # Actual data
        avg_data = np.interp(data[1]['avg'], (data[1]['avg'].min(), data[1]['avg'].max()), (0, max_last)) / max_last * 100.
        std_data = data[1]['std'] / max_last * 100.
        time_data = data[1]['time'] - data[1]['time'][0]

        # Get only t > 0
        if num_agents >= 8:
            time_data -= 35.
            
        avg_data = avg_data[time_data >= 0.]
        std_data = std_data[time_data >= 0.]
        time_data = time_data[time_data >= 0.]

        # Plotting
        plt.plot(time_data, avg_data, label=label)
        plt.fill_between(time_data, (avg_data - std_data), (avg_data + std_data), alpha=.15)

    plt.xlabel("Time [s]")
    plt.ylabel("Explored Fraction [\%]")

    plt.grid()
    plt.legend(loc='best', fontsize=8)

    # plt.show()
    # Save figure
    plt.savefig(log_dir.joinpath('exploration_rate.pdf'), dpi=1200, format='pdf')
        

if __name__ == "__main__":
    main()
