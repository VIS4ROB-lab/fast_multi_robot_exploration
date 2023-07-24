"""
    Script that generates the plots for a series of experiments

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
import statistics
import numpy as np
from matplotlib import pyplot as plt


TIME_THRESHOLD_VALID = 10.0


def parse_args() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--folder', '-f', help="Path to the input folder", required=True, type=str)
    parser.add_argument('--num-agents', help="Number of agents used in the experiments", required=True, type=int)
    parser.add_argument('--num-runs', help="Number of run experiments", required=True, type=int)
    return parser.parse_args()


def average_series_per_agent(series: dict):

    num_runs = len(series)
    num_agents = len(series[0])

    # Store the data of each run per agent
    data_per_agent = []

    for agent_id in range(num_agents):
        combined_series, stamp_series = pd.DataFrame(), pd.DataFrame()
        keys = []
        for run_id in range(num_runs):
            # Skip failures
            if len(series[run_id][agent_id]) == 0: continue

            # Process valid runs
            keys.append(str(run_id))
            run_data = series[run_id][agent_id][0]
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


def running_statistics(x, idx, window_size=20):
    min_idx = int(max(idx - window_size / 2, 0))
    max_idx = int(min(idx + window_size / 2, x.shape[0]))
    return np.mean(x[min_idx:max_idx]), np.std(x[min_idx:max_idx])


def main():
    # Parse argument and check that the folder exists
    args = parse_args()

    num_agents = args.num_agents
    num_runs = args.num_runs
    log_dir = Path(args.folder).expanduser()
    if not log_dir.exists():
        print("\033[93mSpecify existing input folder\033[0m")
        return

    planners_folders = [f for f in log_dir.iterdir() if f.is_dir()]
    results_database = {
        planner_folder.name: {
            'timings': [],
            'velocities': [],
            'travelled_distance': [],
            'volume_per_run': {
                run_id: { agent_id: [] for agent_id in range(num_agents) } for run_id in range(num_runs)
            },
            'velocity_norm_per_run': {
                run_id: { agent_id: [] for agent_id in range(num_agents) } for run_id in range(num_runs)
            },
        } for planner_folder in planners_folders
    }

    for planner_folder in planners_folders:
        # print(f"-Planner: {planner_folder.name.upper()}")
        runs_folders = [f for f in planner_folder.iterdir() if f.is_dir()]
        assert num_runs == len(runs_folders), f"Number of runs does not match {num_runs} / {len(runs_folders)}"
        for run_id, run_folder in enumerate(runs_folders):
            # print(f"\t{run_folder.name}")

            # Read files with timings, exploration rate and odometry info
            times = pd.read_csv(run_folder.joinpath('summary_times.csv')).to_numpy()
            assert num_agents == times.shape[0], "Number of agents does not match"

            # Check if the run is a success
            success = all(times[agent_id, 1] >= 0 for agent_id in range(num_agents))
            if not success: continue

            # Sanity check
            assert all(times[agent_id, 2] >= TIME_THRESHOLD_VALID for agent_id in range(num_agents)), \
                "Timings are too low -- sanity check not passed"

            # Check if all data are correct
            if np.isinf(times.astype(np.float64)).any(): continue
            if np.isnan(times.astype(np.float64)).any(): continue

            # Process info
            results_database[planner_folder.name]['timings'].append(np.mean(times[:, 2]))
            results_database[planner_folder.name]['velocities'].append(np.mean(times[:, 3]))
            results_database[planner_folder.name]['travelled_distance'].append(np.mean(times[:, 4]))
            for agent_id in range(num_agents):
                # results_database[planner_folder.name]['timings'].append(times[agent_id, 2])
                # results_database[planner_folder.name]['velocities'].append(times[agent_id, 3])
                # results_database[planner_folder.name]['travelled_distance'].append(times[agent_id, 4])
                # print(f"\t\t- Agent {agent_id}: {times[i, 1]:1.2f} s")

                stamped_volume = pd.read_csv(run_folder.joinpath(f'explored_volume_{agent_id+1}.csv'), header=None).to_numpy()
                results_database[planner_folder.name]['volume_per_run'][run_id][agent_id].append(stamped_volume)

                odometry = pd.read_csv(run_folder.joinpath(f'odom_{agent_id+1}.csv'), header=None).to_numpy()
                velocity_norm = [[entry[0], np.linalg.norm(entry[8:11])] for entry in odometry]
                results_database[planner_folder.name]['velocity_norm_per_run'][run_id][agent_id].append(np.array(velocity_norm))

    # Generate summary for timings and plots for explored volume
    fig, axs = plt.subplots(2)
    success_rates,final_timings, final_velocities, final_trav_dist = {}, {}, {}, {}
    for entry in results_database.items():
        # If we have no success, then return nothing
        if len(entry[1]['timings']) == 0:
            print("\033[93mNo success cases!\033[0m")
            # Set everything to zero
            success_rates[entry[0]] = 0.
            final_timings[entry[0]] = [0., 0.]
            final_velocities[entry[0]] = [0., 0.]
            final_trav_dist[entry[0]] = [0., 0.]
            continue

        # Timings
        avg_timings = statistics.mean(entry[1]['timings'])
        try:
            std_timings = statistics.stdev(entry[1]['timings'])
        except statistics.StatisticsError:
            std_timings = 0.0

        final_timings[entry[0]] = [avg_timings, std_timings]
        print(f"{entry[0]}: {avg_timings:1.2f} +/- {std_timings:1.2f} s")

        # Velocities
        avg_velocities = statistics.mean(entry[1]['velocities'])
        try:
            std_velocities = statistics.stdev(entry[1]['velocities'])
        except statistics.StatisticsError:
            std_velocities = 0.0

        final_velocities[entry[0]] = [avg_velocities, std_velocities]

        # Travelled Distances
        avg_trav_distances = statistics.mean(entry[1]['travelled_distance'])
        try:
            std_trav_distances = statistics.stdev(entry[1]['travelled_distance'])
        except statistics.StatisticsError:
            std_trav_distances = 0.0

        final_trav_dist[entry[0]] = [avg_trav_distances, std_trav_distances]

        # Generate series
        volume_per_agent = average_series_per_agent(entry[1]['volume_per_run'])
        velocity_per_agent = average_series_per_agent(entry[1]['velocity_norm_per_run'])

        # Align the series (volume is always logged, while velocity only at the beginning of the experiment)
        t0_vel = velocity_per_agent[0]['time'][0]
        idx0_vol, t0_vol = min(enumerate(volume_per_agent[0]['time']), key=lambda x: abs(x[1]-t0_vel))

        # Success rate
        success_rates[entry[0]] = len(entry[1]['timings']) / num_runs
        
        # Plotting (FIXME: This data is per agent!)
        planner_name = entry[0].upper()
        if planner_name == "FAME": planner_name += " (Ours)"

        # Explored volume
        volume_times = volume_per_agent[0]['time'][idx0_vol:] - t0_vol
        volume_avg = volume_per_agent[0]['avg'][idx0_vol:]
        volume_std = volume_per_agent[0]['std'][idx0_vol:]

        axs[0].plot(volume_times, volume_avg, label=planner_name, linewidth=0.7)
        axs[0].fill_between(volume_times, (volume_avg - volume_std), (volume_avg + volume_std), alpha=.3)
        axs[0].grid(b=True, which='major')
        axs[0].set_ylabel('Explored Volume [m$^3$]')
        axs[0].legend(loc='lower right')
        axs[0].autoscale(tight=True)

        # Velocity (filtered)
        velocity_times = velocity_per_agent[0]['time'] - t0_vel
        num_velocity_entries = velocity_per_agent[0]['avg'].shape[0]
        velocity_avg, velocity_std = np.zeros((num_velocity_entries)), np.zeros((num_velocity_entries))
        for k in range(num_velocity_entries):
            velocity_avg[k], velocity_std[k] = running_statistics(velocity_per_agent[0]['avg'], k)
        
        axs[1].plot(velocity_times, velocity_avg, label=planner_name, linewidth=0.7)
        axs[1].fill_between(velocity_times, (velocity_avg - velocity_std), (velocity_avg + velocity_std), alpha=.3)
        axs[1].grid(b=True, which='major')
        axs[1].set_ylabel('Velocity Norm [m/s]')
        axs[1].set_xlabel('Time [s]')
        axs[1].legend(loc='lower right')
        axs[1].autoscale(tight=True)

    # Store timings to file
    with open(log_dir.joinpath('timings.csv'), 'w') as output_file:
        output_file.write("Planner,Success Rate [-],Time Avg [s],Time Std [s],Velocity Avg [m/s],Velocity Std [m/s],Trav Dist Avg [m],Trav Dist Std [m]\n")
        for (items_time, items_vel, items_dist) in zip(final_timings.items(), final_velocities.items(), final_trav_dist.items()):
            planner_name = items_time[0]
            if planner_name == "fame": planner_name += " (ours)"
            output_file.write(f"{planner_name},{success_rates[items_time[0]]},{items_time[1][0]},{items_time[1][1]}," \
                f"{items_vel[1][0]},{items_vel[1][1]},{items_dist[1][0]},{items_dist[1][1]}\n")

    # Storing plot
    fig.suptitle("Summarized Results")
    plt.savefig(log_dir.joinpath('performance.pdf'), dpi=1200, format='pdf')

if __name__ == "__main__":
    main()
