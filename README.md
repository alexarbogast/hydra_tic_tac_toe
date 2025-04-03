# hydra_tic_tac_toe
A multi-robot task-allocation and scheduling demonstration using the game of
tic-tac-toe.

## Installation
The tic-tac-toe demo has the following dependencies:
- [pyrobopath](https://pyrobopath.readthedocs.io/en/latest/)
- [pyrobopath_ros](https://github.com/alexarbogast/pyrobopath_ros)
- [cartesian_planning](https://github.com/alexarbogast/cartesian_planning)
- [za_ros](https://github.com/alexarbogast/za_ros)
- [hydra_ros](https://github.com/alexarbogast/hydra_ros) (branch: ampf)

These can be installed as follows.
```bash
pip install pyrobopath
```
Create a catkin workspace and build the ROS package dependencies.
```sh
mkdir -p pyrobopath_ws/src && cd pyrobopath_ws/src

git clone git@github.com:alexarbogast/cartesian_planning.git
git clone git@github.com:alexarbogast/pyrobopath_ros.git
git clone --recurse-submodules git@github.com:alexarbogast/za_ros.git
git clone --recurse-submodules -b ampf git@github.com:alexarbogast/hydra_ros.git
cd ../
catkin build
```

## Playing the game
The [minimax](https://en.wikipedia.org/wiki/Minimax) algorithm is used to
simulate a tic-tac-toe game between two agents. Each robot draws the game board
and takes turns drawing moves to play the game of tic-tac-toe.

```bash
roslaunch hydra_tic_tac_toe cartesian_trajectory_execution.launch
roslaunch hydra_tic_tac_toe tic_tac_toe.launch
```

To visualize the schedule before executing the motion on the robots, uncomment
the following code blocks from [tic_tac_toe_exec.py](./scripts/tic_tac_toe_exec.py)
```python
animate_multi_agent_toolpath_full(
    toolpath,
    self.sched_exec._schedule,
    self.sched_exec.agent_models,
    limits=LIMITS,
)
```
