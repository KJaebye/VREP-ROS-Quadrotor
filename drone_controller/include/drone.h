/*
 * Copyright 2020 Kangyao Huang, ACSE, The University of Sheffield, UK
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// some neccesary msg types
#include <sensor_msgs/Range.h>
#include <std_msgs/Bool.h>

// drone controller loops
#include "attitude_control_loop.h"
#include "position_control_loop.h"
#include "velocity_control_loop.h"
#include "acceleration_control_loop.h"



namespace drone_control{

class Drone
{
    public:
    Drone();
    ~Drone();

    //initialize
    void InitializeParams();

    //publish functions
    void Publish();
    void PublishForTest();

    //specialize each drone
    void SpecializeParams(int id_number);
    void SpecializeFirstDroneParams();

    private:

    //controller loops
    AttitudeController attitude_controller;
    PositionController position_controller;
    VelocityController velocity_controller;
    AccelerationController acceleration_controller;

    // names of the drone and topics
    std::string drone_id;
    std::string gps_topic_id;
    std::string imu_topic_id;

    std::string waypoint_topic_id;
    std::string motor_speed_topic_id;

    // sensing data
    nav_msgs::Odometry current_gps;
    sensor_msgs::Imu current_imu;

    // commands
    mav_msgs::CommandTrajectory command_wp_msg; // external command

    mav_msgs::CommandVelocityTrajectory command_velocity_msg;
    mav_msgs::CommandVelocityTrajectory command_acc_msg;
    mav_msgs::CommandRollPitchYawrateThrust command_attitude_msg;
    
    Eigen::VectorXd command_angular_rate_msg;
    Eigen::VectorXd command_control_output;
    Eigen::VectorXd command_motor_speed;

    // subscribers and publishers for drone
    ros::NodeHandle nh;
    ros::Subscriber gps_sub;
    ros::Subscriber imu_sub;

    ros::Subscriber command_trajectory_sub;
    ros::Publisher command_motor_speed_pub;
    
    //sensor msgs processing
    void OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg);
    void IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg);

    //command waypoint processing
    void CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_msg);

    // initial position
    mav_msgs::CommandTrajectory initial_position;

/////////////////// test publishers //////////////////////////////////////////////////////////
    ros::Publisher vel_pub;
    ros::Publisher roll_pitch_yaw_thrust_pub;
    ros::Publisher acc_pub;
};


}