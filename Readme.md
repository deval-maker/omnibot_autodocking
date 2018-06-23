# Omni-directional Bot Autodocking Simulation

## Prerequisites
 
* A linux machine 
* ROS
* Gazebo 


## How to run the code ?

- Clone this repository/workspace and go to the downloaded directory.

```sh
$ git clone https://github.com/deval-maker/omnibot_autodocking.git
$ cd omnibot_autodocking/
```

- Build and run the launch file.

```sh
$ catkin_make
$ source devel/setup.sh  # sh/bash/zsh, depending upon the shell env.
$ roslaunch omnibot_description omnibot.launch
```
- Unpause the simulation from gazebo. (Press Space key)

## Code Tested and working with 

* 16.04.1-Ubuntu Gnome
* ROS Kinetic
* Gazebo 7.13.1 
 
