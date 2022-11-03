#ifndef CONTROLLER_UTILITY_H
#define CONTROLLER_UTILITY_H

#include <Eigen/Eigen>

#include <mav_msgs/conversions.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <mav_msgs/CommandMotorSpeed.h>
#include <mav_msgs/CommandRollPitchYawrateThrust.h>
#include <mav_msgs/CommandVelocityTrajectory.h>
#include <mav_msgs/MotorSpeed.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <planning_msgs/WayPoint.h>
#include <planning_msgs/eigen_planning_msgs.h>
#include <planning_msgs/conversions.h>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

namespace drone_control{


class ControllerUtility
{
    public:
    ControllerUtility();
    ~ControllerUtility();

    //Utility functions 
    double map(double x, double in_min, double in_max, double out_min, double out_max);
    double limit( double in, double min, double max);

    bool GetSwitchValue(void);
    bool UpdateSwitchValue(bool currInput);

    Eigen::Vector3d rotateGFtoBF(double GF_x, double GF_y, double GF_z, double GF_roll, double GF_pitch, double GF_yaw);

    private:
    bool switchValue;
    bool prevInput;
};
}

#endif