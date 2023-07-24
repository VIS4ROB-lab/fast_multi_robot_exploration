# Fast Multi-Robot Decentralized Exploration of Forests

If you use this code in your academic work, please cite ([PDF](https://www.research-collection.ethz.ch/handle/20.500.11850/620637)):

    @inproceedings{bartolomei2023multi,
      title={Fast Multi-UAV Decentralized Exploration of Forests},
      author={Bartolomei, Luca and Teixeira, Lucas and Chli, Margarita},
      booktitle={IEEE Robotics and Automation Letters},
      year={2023}
    }

This project is released under a GPLv3 license.

## Video
<a href="https://youtu.be/F3aKqi5Q2LE" target="_blank"><img src="http://img.youtube.com/vi/F3aKqi5Q2LE/0.jpg" alt="Mesh" width="480" height="360" border="0" /></a>

# Installation

The pipeline has been built and tested in **Ubuntu 20.04 LTS** and **ROS Noetic**.

* Install dependencies: `sudo apt install libarmadillo-dev libelf-dev libdw-dev python3-wstool python3-catkin-tools python3-pip`
* Install python dependencies: `pip install pandas`
* Install NLopt (check [here](https://github.com/SYSU-STAR/RACER#unexpected-crash)):
    ```
    $ git clone  https://github.com/stevengj/nlopt.git --branch v2.7.1
    $ cd nlopt && mkdir build && cd build
    $ cmake ..
    $ make -j8
    $ sudo make install
    ```
* Install dependencies as described [here](https://github.com/SYSU-STAR/RACER#quick-start):
    ```
    $ wget http://akira.ruc.dk/~keld/research/LKH-3/LKH-3.0.6.tgz
    $ tar xvfz LKH-3.0.6.tgz
    $ cd LKH-3.0.6
    $ make
    $ sudo cp LKH /usr/local/bin
    $ cd ..
    $ rm LKH-3.0.6.tgz
    ```
* Create a catkin workspace:
	```
	$ mkdir -p catkin_ws/src
	$ cd catkin_ws
	$ catkin init
	```

Once the dependencies are installed, the code can be compiled as follows:
```
$ cd catkin_ws
$ source /opt/ros/noetic.bash
$ catkin_make
```

# Valid models
The list of valid models can be found (as series of `yaml` files) in [`exploration_manager/config/maps`](swarm_exploration/exploration_manager/config/maps). The names of the models still need to be changed to match the ones in the paper. For the moment, this list can be used:

* `forest_50x50_01_200.pcd` corresponds to `Sparse Forest (0.1 trees / m^2)`
* `forest_50x50_01_300.pcd` corresponds to `Mid-density Forest (0.15 trees / m^2)`
* `forest_50x50_100_denser_3.pcd` corresponds to `Dense Forest (0.2 trees / m^2)`
* `forest_4_densities_50_40_20_10.pcd` corresponds to `Multi-density Forest`

# Run Unit Tests
To run Unit Tests, run the following command:
```
$ catkin_make run_tests_exploration_manager
```

# Running commands
To run an experiment with a planner, run the following commands in separate terminals (possible `planner_type`: `fame` and `racer`):
```
$ roscore
$ roslaunch exploration_manager rviz.launch
$ roslaunch exploration_manager swarm_exploration.launch model:=${model_name} drone_num:=${num_drones} planner_type:=fame
```
To start the exploration, use the `2D Nav Goal` tool in `Rviz`.

## Series of experiments
A sequence of experiments in all the environments can be launch by running the scripts in [`automatic_runs`](./automatic_runs/). In this case, the experiments will be trigger automatically and a logging node will dump all the statistics.

**Note**: The experiments with 8 and 10 agents were run on EC2 instance on the cloud. The type of the instance was `m4.4xlarge` (16-core CPU and 64GB of RAM)

# Evaluation of series of experiments
To evaluate the results of an experiment, run the scripts [here](./swarm_exploration/exploration_manager/scripts/):
* Evaluate a series of runs: `$ python3 plot_series.py` (use flag `-h` to check how to pass arguments)

To evaluate a folder with runs in multiple models, use the script in [`automatic_runs/evaluation`](./automatic_runs/evaluation):
```
$ cd automatic_runs/evaluation
$ . evaluate_multiple_models.sh ${path_to_results_folder} ${num_runs_per_exp}
```

# Generate maps
Run the following launch file to create a random map with 4 areas with different densities:
```
$ roslaunch exploration_manager multi_density.launch
```

Run the following launch file to create a random map:
```
$ roslaunch exploration_manager gen_map_denser.launch
```

# Acknowledgments
Our software is based the open-source project [RACER](https://github.com/SYSU-STAR/RACER).

# Contributing
Contributions that help to improve the code are welcome. In case you want to contribute, please adapt to the [Google C++ coding style](https://google.github.io/styleguide/cppguide.html) and run `bash clang-format-all .` on your code before any commit.
