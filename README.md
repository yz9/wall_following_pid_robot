# Wall Following PID Robot

> Assignment designed to write a feedback controller that controls the yaw of a ground robot via PID control based on  proportional, integral, and derivative terms (denoted P, I, and D respectively), mainly relying on gazebo for simulated scenarios.

Demo:

![pid_controller](https://user-images.githubusercontent.com/22216684/42646067-c4dc9b3a-85cd-11e8-8e57-9f2323aa3fcc.gif)

*   **PID Controller**     [Explanation](https://www.techbriefs.com/component/content/article/tb/features/articles/20013)

## Prerequisites

* Ubuntu 16.04
* ROS kinetic: Installation instruction can be found [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).
* Required ROS packages:
    ```
    $ sudo apt-get install ros-kinetic-control-toolbox ros-kinetic-joystick-drivers ros-kineticrealtime-tools ros-kinetic-ros-control ros-kinetic-gazebo-ros-control ros-kinetic-roscontrollers
    ```

NOTE: By default the HOME environment variable is not set. Set this up doing the following:
```
export ROS_HOME=~/.ros
source /opt/ros/kinetic/setup.bash
```

## Build

compile the code
```
$ catkin_make
```

## How to Use

Start gazebo simulator
Bring up a world with walls in the gazebo simulator
```
$ roslaunch wall_following_assignment gazebo_world.launch world_name:=walls_one_sided
```
Start husky follower robot
Bring up the robot in that world
```
$ roslaunch wall_following_assignment husky_follower.launch
```
The only error that should appear after these two commands is that no joystick can be found.

Start PID controller
Then launch the PID controller
```
$ roslaunch wall_following_assignment wall_follower_python.launch
```

### Parameter Tuning
You can tweak the PID parameters using the dynamic reconfigure server.
```
$ rosrun rqt_reconfgure rqt_reconfgure
```

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details
