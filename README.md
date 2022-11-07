# Introduction
This repo provides a demo of creating a ROS control drone in VREP(now the CoppeliaSim), equipped with various sensors.

# VREP Associated Child Scripts
The key of running and controlling a robot in VREP is to use associated child script to create publishers and subscribers communicating with external ROS package controller. Scripts are at /associated_scripts.

There are five kinds of sensors in drone_pro2.0.ttm model, including imu, proximity sensors, quadricopter, spherical vision sensor, and ultrasonic sensor. The associated scripts are written by lua.

The quadricopter configuration is like this:
![avatar](/UAV%20config.png)


# Usage
The lua script can publish drone's states and subscribe remote control command from external via ROS msg. In detail, flight autopilot subscribes info from gps and imu, and publish motor speed command as outputs. Thus, user should write an external drone controller outside the VREP.
