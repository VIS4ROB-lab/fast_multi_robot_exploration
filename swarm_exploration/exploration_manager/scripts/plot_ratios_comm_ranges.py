"""
    Script that generates the plot of the ratios over varying number of UAVs 
    and communication ranges

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


VALID_TEAMS = [2, 4, 6, 8, 10]


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

    # Analyze the data per planner type
    ratios = {n: {} for n in VALID_TEAMS}
    data = pd.read_csv(log_dir.joinpath("ratios.csv")).to_numpy()
    for entry in data:
        cr_label = f"CR{entry[0]:1.1f}m"
        for i in range(len(VALID_TEAMS)):
            ratios[VALID_TEAMS[i]][cr_label] = entry[i + 1]

    # Plotting
    plt.figure(1)

    # X: CR, Y: Ratio
    # for drone_num in VALID_TEAMS:
    #     # Generate data for plotting
    #     x, y = [], []
    #     for entry in ratios[drone_num].items():
    #         x.append(entry[0][2:-1])
    #         y.append(entry[1])

    #     plt.plot(x, y, "-o", label=drone_num)
    # 
    # plt.xlabel("Communication Range [m]", labelpad=0)

    # X: Drone num, Y: Ratio
    x = VALID_TEAMS
    comm_ranges = sorted(ratios[VALID_TEAMS[0]].keys(), key=lambda x: int(x[2:4]))

    for cr in comm_ranges:
        y = []
        for drone_num in VALID_TEAMS:
            # Generate data for plotting
            y.append(ratios[drone_num][cr])

        cr_label = f"Comm. Range {cr[2:4]} m"
        plt.plot(x, y, "-o", label=cr_label)
    
    plt.xlabel("Number of Drones", labelpad=0)
    
    plt.xticks(x)
    plt.grid()
    plt.legend(loc='best')

    plt.ylabel("Ratio $\\frac{t_{Ours}}{t_{RACER}}$ [-]")
    # plt.ylabel("Ratio [-]")

    # Save figure
    plt.savefig(log_dir.joinpath('communication_range_ratio_plot.pdf'), dpi=1200, format='pdf')


if __name__ == "__main__":
    main()