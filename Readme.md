# Omnidirectional Bot Autodocking Simulation

## Prerequisites
 
* A linux machine 
* [Robot Operating System](http://www.ros.org)
* [Gazebo](http://gazebosim.org)


## How to run the code ?

- Clone this repository/workspace and go to the downloaded directory.

```sh
$ git clone https://github.com/deval-maker/omnibot_autodocking.git
$ cd omnibot_autodocking/
```

- Build and Configure.

```sh
$ catkin_make
$ source devel/setup.sh  # sh/bash/zsh, depending upon the shell env.
```

- (A) To run Gazebo and Rviz.

```sh
$ roslaunch omnibot_description omnibot.launch
```

- (B) To run full Nav stack and Gazebo.

```sh
$ roslaunch omnibot_nav omnibot_nav.launch
```
- Unpause the simulation from gazebo. (Press Space key)


## Code Tested and working with 

* 16.04.1-Ubuntu Gnome
* ROS Kinetic
* Gazebo 7.13.1 
 
