# Introduction
This repo provides a demo of creating a ROS control drone in VREP(now the CoppeliaSim), equipped with various sensors.

# VREP Associated Child Scripts
The key of running and controlling a robot in VREP is to use associated child script to create publishers and subscribers communicating with external ROS package controller. Scripts are in /associated_scripts.

There are five kinds of sensors in drone_pro2.0.ttm model, including imu, proximity sensors, quadricopter, spherical vision sensor, and ultrasonic sensor. The associated scripts are written by lua.

The quadricopter configuration is like this:
![avatar](/UAV%20config.png)


# ROS Control
/drone_controller is a ROS package. It provides a classical quadcopter cascade PID controller, with position, velocity, attitude, and even accelaration control loop if you like.

This package provides an external controller like a remote control. We can publish position cmd and velocity cmd in ROS, and the VREP scripts subscribe cmd and execute.

***Note: The package relays on glog_catkin, mav_comm, mav_msgs, mavros, planning_msgs.***

# Usage
This controller provides two modes: position control, and velocity control. 

Flight autopilot subscribes info from gps and imu, and publish motor speed command as outputs.
