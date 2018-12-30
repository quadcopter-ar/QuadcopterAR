# Quadcopter Controls Software Requirements and Installation Instructions

## Requirements



WARNING 1: Do NOT use QGC indoor (please use mavproxy)

WARNING 2: Make sure having GPS fix before takeoff

### Ubuntu

- We use Ubuntu 16.04 LTS (long term support)
- Dual boot or dedicated machine is preferable to virtual machine, VM is untested with the below

### ROS

- ROS is a software framework that provides packages in C++ and Python for modular robotics software development
- We use ROS Kinetic
- Install using the instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)

### Arducopter SITL (Software in the Loop)

- SITL is a simulation of the quadcopter firmware. More info [here](http://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)

- Ardupilot contains SITL: when you download Ardupilot, along with the firmware there are utilities for running the firmware in simulation as opposed to on a real drone

- We are currently using [Randy Mackay’s fork of Ardupilot,](https://github.com/rmackay9/rmackay9-ardupilot) since this is where the external localization functionality was implemented in summer 2018. 

- - Commit no: 02b9b582604db96b1ec968c9312310c374f96eae

Follow the steps for cloning and installing the repo  [here](http://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html)



