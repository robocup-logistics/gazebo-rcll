[![robocup-logistics](https://circleci.com/gh/robocup-logistics/gazebo-rcll.svg?style=shield)](https://app.circleci.com/pipelines/github/robocup-logistics/gazebo-rcll)

# RoboCup-Logistics Gazebo Simulation

This repository includes the models, worlds and plugins to simulate the RoboCup Logistics League with Gazebo.

## Setup Instructions

1. [Install Gazebo](http://gazebosim.org/tutorials?cat=install)
2. Build the plugins with `cmake`:
   ```
   $ cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo
   $ cmake --build build
   ```
3. Set up the environment. Open `~/.bashrc` with your favorite text editor and add the following lines:
   ```bash
   source /usr/share/gazebo/setup.sh
   export GAZEBO_RCLL=~/gazebo-rcll
   export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:$GAZEBO_RCLL/build/plugins
   export GAZEBO_MODEL_PATH=$GAZEBO_RCLL/models
   export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$GAZEBO_RCLL/models/carologistics
   export GAZEBO_WORLD_PATH=$GAZEBO_RCLL/worlds/carologistics/llsf.world
   ```
   You may have to change the path depending on where you cloned the repository and which world you want to run.
4. Restart your terminal to make sure the environment variables are set correctly.

Then you can start gazebo from the terminal.
