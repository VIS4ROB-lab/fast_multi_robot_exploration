"""
    Script that generates the plots of the computation times for a series of 
    experiments (per agent)

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
    plt.style.use(['time_vs_team'])
except ImportError:
    pass


VALID_PLANNERS = ['fame (ours)', 'explorer', 'no_coll']
NUM_AGENTS = [1, 2, 4, 6, 8, 10]


def parse_args() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', '-f', help="Path to the input folder", required=True, type=str)
    return parser.parse_args()


def generate_summary(log_dir: Path):
    results = {p: {} for p in VALID_PLANNERS}
    for drones_dir in log_dir.iterdir():
        if not drones_dir.is_dir(): continue

        data = pd.read_csv(drones_dir.joinpath("timings.csv")).to_numpy()
        for i in range(len(data)):
            results[data[i][0]][drones_dir.name.split("_")[0]] = {'avg_times': data[i][2], 'std_times': data[i][3]}
    return results


def main():
    # Parse argument and check that the folder exists
    args = parse_args()

    log_dir = Path(args.folder).expanduser()
    if not log_dir.exists():
        print("\033[93mSpecify existing input folder\033[0m")
        return

    # Analyze the data per planner type    
    results = generate_summary(log_dir)

    # Plotting
    plt.figure(1)
    for planner in VALID_PLANNERS:
        # Generate data for plotting
        avg, std, num_drones = [], [], []
        for drone_num in sorted(results[planner].keys(), key=lambda x: int(x)):
            if int(drone_num) not in NUM_AGENTS:
                continue

            num_drones.append(drone_num)
            avg.append(results[planner][drone_num]['avg_times'])
            std.append(results[planner][drone_num]['std_times'])

        if planner == "fame (ours)":
            label = "Full"
            marker = "-o"
        if planner == "explorer":
            label = "Explorer Only"
            marker = "-x"
        if planner == "no_coll":
            label = "Without Collaboration"
            marker = "-d"

        plt.plot(num_drones, avg, marker, label=label)
        
        lim_inf = np.array(avg) - np.array(std)
        lim_sup = np.array(avg) + np.array(std)
        plt.fill_between(num_drones, lim_inf, lim_sup, alpha=.3)

        plt.xticks(num_drones)

    plt.grid()
    plt.legend()

    plt.xlabel("Number of Drones", labelpad=0)
    plt.ylabel("Mission Time [s]")

    # Save figure
    plt.savefig(log_dir.joinpath('ablation_time_vs_team_size.pdf'), dpi=1200, format='pdf')


if __name__ == "__main__":
    main()