# Run time monitor on robots

## Introduction
A runtime verification demostration on Jackal robot.

## Installation Guild
### Software Environment Requirement
- Ubuntu 20.04
- SPOT 2.11.6
- ROS Noetic
- Python 3

### Install SPOT
- ref[https://spot.lre.epita.fr/install.html]

### Install ROS
- ref[http://wiki.ros.org/noetic/Installation]

### Install the code
```
# create a workspace
mkdir rv_ws
cd rv_ws
mkdir src
# pull this repo
cd src
git pull https://github.com/TieneSabor/monitor
cd ..
# compile codes
catkin_make
```

## Run Example
### Lidar Instrument
This program collects data from lidar (topic name: "\scan", topic type: "sensor_msgs/LaserScan") and convert it to the following atomic proposition
- e is true if and only if the nearest obstacle to the lidar is greater than 0.75 meters away.

### Monitor 
This program receives events from Lidar Instrument and check the trace with following formula
- G e (e should always be true)

### Controller
This program, as a demostration, drive the robot forward in 0.25 m/s.

### Safety Switch
This program receives signal from Monitor and stop the robot (isolate the robot from singals from Controller) when any of the specification is violated.

### Run the code
```
# first terminal
cd /path/to/ros/workspace/rv_ws
python3 src/monitor/scripts/fake_controller.py
# second terminal
cd /path/to/ros/workspace/rv_ws
python3 src/monitor/scripts/lidar_instrument.py
# third terminal
cd /path/to/ros/workspace/rv_ws
python3 src/monitor/scripts/safety_switch_by_monitor.py
# forth terminal
cd /path/to/ros/workspace/rv_ws
rosrun monitor monitorNode
```

