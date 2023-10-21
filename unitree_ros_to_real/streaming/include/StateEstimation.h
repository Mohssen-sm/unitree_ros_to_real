#ifndef SE_H
#define SE_H

#include <ros/ros.h>
#include <chrono>
#include <pthread.h>
#include <math.h>
#include <stdint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include "Utilities/orientation_tools.h"
#include "Utilities/cppTypes.h"

struct StreamedData{
    double time;

    Vec4<double> quaternion;
    RotMat<double> rotmat;
    Vec3<double> rpy;
    Vec3<double> position;
    Vec3<double> v_world;
    Vec3<double> v_body;
    Vec3<double> omega_world;
    Vec3<double> omega_body;
};

class StateEstimation
{
public:
    StateEstimation(std::string object_name);
    StreamedData _data;
    StreamedData _dataPrev;
    geometry_msgs::TwistStamped twist_msg;

private:
    ros::NodeHandle n;
    ros::Subscriber sub;
    ros::Publisher pub;
    void ObjectCallback(const geometry_msgs::PoseStamped& msg);
    void LinearVelocityCalc();
    void AngularVelocityCalc();
};

#endif