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

#include "drone.h"
namespace drone_control{

Drone::Drone()
{
    InitializeParams();
    //publishers for test
    vel_pub = nh.advertise<mav_msgs::CommandVelocityTrajectory>("/command/linear_velocity", 10);
    roll_pitch_yaw_thrust_pub = nh.advertise<mav_msgs::CommandRollPitchYawrateThrust>("/command/attitude", 10);
    acc_pub = nh.advertise<mav_msgs::CommandVelocityTrajectory>("/command/linear_accelerater", 10);
}

Drone::~Drone(){}

void Drone::InitializeParams()
{

    position_controller.InitializeParameters(nh);
    velocity_controller.InitializeParameters(nh);
    attitude_controller.InitializeParameters(nh);
    acceleration_controller.InitializeParameters(nh);
    
    command_wp_msg.position.x=0;
    command_wp_msg.position.y=0;
    command_wp_msg.position.z=1.5;
    command_wp_msg.yaw=0;
}


//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Specialization Processing ////////////////////////////////////////////////

void Drone::SpecializeFirstDroneParams()
{
    //specialize the name of this object
    drone_id = "Quadricopter";
    // drone controller topic id
    waypoint_topic_id = drone_id + "/command/way_point";
    motor_speed_topic_id = drone_id + "/command/motor_speed";
  
    gps_topic_id = drone_id + "/sensor/gps";
    imu_topic_id = drone_id + "/sensor/imu";
 
    //getting sensors msg
    gps_sub = nh.subscribe(gps_topic_id, 1000, &Drone::OdometryCallback, this);
    imu_sub = nh.subscribe(imu_topic_id, 1000, &Drone::IMUCallback, this);

    // drone control commands in
    command_trajectory_sub = nh.subscribe(waypoint_topic_id, 10, &Drone::CommandTrajectoryCallback, this);
    // drone control commands out
    command_motor_speed_pub = nh.advertise<mav_msgs::CommandMotorSpeed>(motor_speed_topic_id, 10);

}

void Drone::SpecializeParams(int id_number)
{
    //specialize the name of this object
    drone_id = "Quadricopter_" + std::to_string(id_number);

    waypoint_topic_id = drone_id + "/command/way_point";
    motor_speed_topic_id = drone_id + "/command/motor_speed";
  
    gps_topic_id = drone_id + "/sensor/gps";
    imu_topic_id = drone_id + "/sensor/imu";

    // getting sensors msg
    gps_sub = nh.subscribe(gps_topic_id, 1000, &Drone::OdometryCallback, this);
    imu_sub = nh.subscribe(imu_topic_id, 1000, &Drone::IMUCallback, this);
  
    // commands in
    command_trajectory_sub = nh.subscribe(waypoint_topic_id, 10, &Drone::CommandTrajectoryCallback, this);
    // commands out
    command_motor_speed_pub = nh.advertise<mav_msgs::CommandMotorSpeed>(motor_speed_topic_id, 10);
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Publishing ///////////////////////////////////////////////////////////////

// drone controller publish
void Drone::Publish()
{
    mav_msgs::CommandMotorSpeedPtr cmd_motors_msg(new mav_msgs::CommandMotorSpeed);
    cmd_motors_msg->motor_speed.clear();

    for (int i = 0; i < command_motor_speed.size(); i++)
    {
        cmd_motors_msg->motor_speed.push_back(command_motor_speed[i]);
    }

    ros::Time update_time = ros::Time::now();
    cmd_motors_msg->header.stamp = update_time;
    cmd_motors_msg->header.frame_id = "drone_";
    command_motor_speed_pub.publish(cmd_motors_msg);
}


void Drone::PublishForTest()
{
    vel_pub.publish(command_velocity_msg);
    roll_pitch_yaw_thrust_pub.publish(command_attitude_msg);
    acc_pub.publish(command_acc_msg);
}


////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Command Processing ///////////////////////////////////////////////////////

void Drone::CommandTrajectoryCallback(const mav_msgs::CommandTrajectoryConstPtr& trajectory_msg)
{
    //ROS_INFO_ONCE("Drone got the first trajectory command.");
    command_wp_msg = *trajectory_msg;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// Sensor MSG Processing /////////////////////////////////////////////////////

void Drone::OdometryCallback(const nav_msgs::OdometryConstPtr& odometry_msg)
{
    //ROS_INFO_ONCE("Drone got the first GPS message.");
    current_gps = *odometry_msg;

    position_controller.CalculatePositionControl(command_wp_msg, current_gps, &command_velocity_msg);
    velocity_controller.CalculateVelocityControl(command_velocity_msg, current_gps, &command_acc_msg);
    acceleration_controller.CalculateAccelerationControl(command_acc_msg, current_gps, current_imu, &command_attitude_msg);
}

void Drone::IMUCallback(const sensor_msgs::ImuConstPtr& imu_msg)
{
    //ROS_INFO_ONCE("Drone got the first IMU message.");
    current_imu = *imu_msg;
    attitude_controller.CalculateAttitudeControl(command_attitude_msg, current_imu, &command_angular_rate_msg);
    attitude_controller.CalculateRateControl(command_angular_rate_msg, current_imu, &command_control_output);
    attitude_controller.CalculateMotorCommands(command_control_output, &command_motor_speed);
    Publish();
}

}




int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_controller_node");
    int client_number = 1;
    drone_control::Drone drones[client_number];
    if(client_number==1)
    {
        drones[0].SpecializeFirstDroneParams();
    }else
    {
        // for creating a swarm of quadrotors
        drones[0].SpecializeFirstDroneParams();
        for(int i = 1; i < client_number; i++)
        {
            drones[i].SpecializeParams(i-1);
        }
    }

    ros::spin();
    return 0;
}