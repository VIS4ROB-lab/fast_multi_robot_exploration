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
    plt.style.use(['custom'])
except ImportError:
    pass


VALID_PLANNERS = ['fame', 'racer']
CONVERT_FACT = 1e-6


def parse_args() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', '-f', help="Path to the input folder", required=True, type=str)
    return parser.parse_args()


def generate_summary(planner: str, log_dir: Path, num_agents: int):

    if planner == 'racer':
        results = {
            'computation_times': []
        }
    elif planner == 'fame':
        results = {
            'computation_times_explorer': [],
            'computation_times_collector': []
        }
    else:
        raise ValueError("Unknown planner type")

    run_folders = [f for f in log_dir.iterdir() if f.is_dir()]
    for run_folder in run_folders:
        times = pd.read_csv(run_folder.joinpath('summary_times.csv')).to_numpy()

        # Check if the run is a success
        success = all(times[agent_id, 1] >= 0 for agent_id in range(times.shape[0]))
        if not success: continue

        del times

        # Read timings
        if planner == 'fame':
            for i in range(num_agents):
                # Explorer
                results['computation_times_explorer'].extend(
                    pd.read_csv(run_folder.joinpath(
                        f'comput_timings_explorer_{i+1}.csv')).to_numpy()[:, 1] * CONVERT_FACT)
                # Collector
                results['computation_times_collector'].extend(
                    pd.read_csv(run_folder.joinpath(
                        f'comput_timings_collector_{i+1}.csv')).to_numpy()[:, 1] * CONVERT_FACT)
        else:
            # Nothing logged at the moment
            for i in range(num_agents):
                results['computation_times'].extend(
                    pd.read_csv(run_folder.joinpath(
                        f'comput_timings_racer_{i+1}.csv')).to_numpy()[:, 1] * CONVERT_FACT)

    # Compute return values
    statistics_res = {}
    if planner == 'fame':
        statistics_res['avg_explorer'] = np.mean(
            results['computation_times_explorer']
        )
        statistics_res['std_explorer'] = np.std(
            results['computation_times_explorer']
        )

        statistics_res['avg_collector'] = np.mean(
            results['computation_times_collector']
        )
        statistics_res['std_collector'] = np.std(
            results['computation_times_collector']
        )

    else:
        if len(results['computation_times']) > 0:
            statistics_res['avg_times'] = np.mean(
                results['computation_times']
            )
            statistics_res['std_times'] = np.std(
                results['computation_times']
            )
        else:
            statistics_res['avg_times'], statistics_res['std_times'] = 0., 0.

    return statistics_res


def main():
    # Parse argument and check that the folder exists
    args = parse_args()

    log_dir = Path(args.folder).expanduser()
    if not log_dir.exists():
        print("\033[93mSpecify existing input folder\033[0m")
        return

    # Analyze the data per planner type    
    num_drones_folders = [f for f in log_dir.iterdir() if f.is_dir()]
    res = {
        planner: {} for planner in VALID_PLANNERS
    }
    for num_drones_folder in num_drones_folders:
        #
        planner_folders = [f for f in num_drones_folder.iterdir() if f.is_dir()]
        #
        for planner_folder in planner_folders:
            if planner_folder.name not in VALID_PLANNERS:
                continue
            #
            num_drones = int(num_drones_folder.name.replace("_drones", ""))
            res[planner_folder.name][num_drones] = generate_summary(
                planner_folder.name, planner_folder, num_drones
            )

    # Summary file
    with open(log_dir.joinpath("computation_times.csv"), 'w') as summary_file:
        header = "Planner,"
        for drone_num in sorted(res[VALID_PLANNERS[0]].keys()):
            header += f"{drone_num}_Drones_Avg,{drone_num}_Drones_Std,"
        header += "\n"
        summary_file.write(header)

        for planner in VALID_PLANNERS:
            line = planner.upper()
            if planner == 'racer':
                for drone_num in sorted(res[planner].keys()):
                    line += f",{res[planner][drone_num]['avg_times']},{res[planner][drone_num]['std_times']}"
            elif planner == 'fame':
                line += " (Explorer)"
                for drone_num in sorted(res[planner].keys()):
                    line += f",{res[planner][drone_num]['avg_explorer']},{res[planner][drone_num]['std_explorer']}"

                line += "\nFAME (Collector)"
                for drone_num in sorted(res[planner].keys()):
                    line += f",{res[planner][drone_num]['avg_collector']},{res[planner][drone_num]['std_collector']}"

            summary_file.write(line + "\n")

    # Plotting
    plt.figure(1)
    for planner in VALID_PLANNERS:
        if planner == 'racer':
            # Generate data for plotting
            avg, std, num_drones = [], [], []
            for drone_num in sorted(res[planner].keys()):
                num_drones.append(str(drone_num))
                avg.append(res[planner][drone_num]['avg_times'])
                std.append(res[planner][drone_num]['std_times'] * 0.5)

            plt.plot(num_drones, avg, '-o', label="RACER")
            
            lim_inf = np.array(avg) - np.array(std)
            lim_sup = np.array(avg) + np.array(std)
            plt.fill_between(num_drones, lim_inf, lim_sup, alpha=.3)

            plt.xticks(num_drones)

        elif planner == 'fame':
            # Generate data for plotting
            avg_expl, std_expl, avg_coll, std_coll, num_drones = [], [], [], [], []
            for drone_num in sorted(res[planner].keys()):
                num_drones.append(str(drone_num))

                # Explorer
                avg_expl.append(res[planner][drone_num]['avg_explorer'])
                std_expl.append(res[planner][drone_num]['std_explorer'])

                # Collector
                avg_coll.append(res[planner][drone_num]['avg_collector'])
                std_coll.append(res[planner][drone_num]['std_collector'])

            plt.plot(num_drones, avg_expl, '-o', label="Explorer (Ours)")
            lim_inf = np.array(avg_expl) - np.array(std_expl)
            lim_sup = np.array(avg_expl) + np.array(std_expl)
            plt.fill_between(num_drones, lim_inf, lim_sup, alpha=.3)

            plt.plot(num_drones, avg_coll, '-o', label="Collector (Ours)")
            lim_inf = np.array(avg_coll) - np.array(std_coll)
            lim_sup = np.array(avg_coll) + np.array(std_coll)
            plt.fill_between(num_drones, lim_inf, lim_sup, alpha=.3)

            plt.xticks(num_drones)

    plt.grid()
    plt.legend(loc='best')

    plt.xlabel("Number of Drones", labelpad=0)
    plt.ylabel("Computation Time [ms]")

    plt.ylim([0., 80.])

    # Save figure
    plt.savefig(log_dir.joinpath('computation_times.pdf'), dpi=1200, format='pdf')


if __name__ == "__main__":
    main()