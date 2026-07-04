
#include "VehicleInterface.h"
#include "ptera_msgs/msg/vehicle_waypoint.hpp"
#include "RosTopicManager.hpp"
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <plog/Log.h>

VehicleInterface::VehicleInterface()
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::VehicleWaypoint>("robot/vehicle/waypoint"); 

    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::ControllerStatus>("robot/vehicle/controller_status",
        std::bind(&VehicleInterface::statusCallback, this, std::placeholders::_1));

    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::RobotState>("robot/state", 
        std::bind(&VehicleInterface::poseCallback, this, std::placeholders::_1)); 
}

VehicleInterface::~VehicleInterface()
{

}

void VehicleInterface::send(const Waypoint& aGoalPose)
{
    ptera_msgs::msg::VehicleWaypoint wp; 
    wp.position.set__x(aGoalPose.goal.x); 
    wp.position.set__y(aGoalPose.goal.y); 
    wp.position.set__z(aGoalPose.goal.z); 

    LOGD << "Goal position " << wp.position.x << " " << wp.position.y << " " << wp.position.z;

    wp.quat.set__x(aGoalPose.goal.qx);
    wp.quat.set__y(aGoalPose.goal.qy);
    wp.quat.set__z(aGoalPose.goal.qz);
    wp.quat.set__w(aGoalPose.goal.qw);

    // compute euler from quat (atan2-based roll/pitch/yaw, not Eigen::eulerAngles():
    // eulerAngles() forces its first angle into [0, pi], which for a near-planar
    // rotation (roll/pitch ~ 0) can flip to the equivalent (pi, pi, yaw - pi)
    // solution and send yaw off by 180 degrees)
    double qw = aGoalPose.goal.qw, qx = aGoalPose.goal.qx, qy = aGoalPose.goal.qy, qz = aGoalPose.goal.qz;
    double roll  = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
    double pitch = std::asin(std::clamp(2.0 * (qw * qy - qz * qx), -1.0, 1.0));
    double yaw   = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));

    LOGD << "Goal orientation " << roll << " " << pitch << " " << yaw;

    wp.euler.set__yaw(yaw);
    wp.euler.set__pitch(pitch);
    wp.euler.set__roll(roll);

    ptera_msgs::msg::Vec3 pos_tol; 
    pos_tol.set__x(aGoalPose.tolerance[0]);
    pos_tol.set__x(aGoalPose.tolerance[1]);
    pos_tol.set__x(aGoalPose.tolerance[2]);

    ptera_msgs::msg::Euler euler_tol; 
    euler_tol.set__yaw(aGoalPose.tolerance[5]); 
    euler_tol.set__pitch(aGoalPose.tolerance[4]); 
    euler_tol.set__roll(aGoalPose.tolerance[3]); 

    wp.set__position_tolerance(pos_tol); 
    wp.set__euler_tolerance(euler_tol); 
    wp.set__execution_duration(600); // TODO: make settable per waypoint

    RosTopicManager::getInstance()->publishMessage("robot/vehicle/waypoint", wp); 
}

void VehicleInterface::statusCallback(ptera_msgs::msg::ControllerStatus::SharedPtr aStatus)
{
    std::lock_guard<std::mutex> lock(mStatusMutex); 
    mLatestStatus = *aStatus;
    
    if(!mStatusRecvd) 
        mStatusRecvd = true; 
}

void VehicleInterface::poseCallback(ptera_msgs::msg::RobotState::SharedPtr aState)
{
    Pose6D pose; 
    pose.x = aState->position.x; 
    pose.y = aState->position.y; 
    pose.z = aState->position.z; 

    pose.qx = aState->quat.x;
    pose.qy = aState->quat.y; 
    pose.qz = aState->quat.z; 
    pose.qw = aState->quat.w; 

    std::lock_guard<std::mutex> lock(mPoseMutex); 
    mLatestPose = pose; 

    if(!mPoseRecvd) 
        mPoseRecvd = true; 
}

Pose6D VehicleInterface::currentPose()
{
    std::lock_guard<std::mutex> lock(mPoseMutex); 
    return mLatestPose; 
}

bool VehicleInterface::isArrived()
{
    std::lock_guard<std::mutex> lock(mStatusMutex); 
    if(ptera_msgs::msg::ControllerStatus::ARRIVED == mLatestStatus.arrival)
    {
        return true; 
    }

    return false; 
}

bool VehicleInterface::isConnected()
{
    if(mPoseRecvd && mStatusRecvd)
    {
        return true; 
    }

    return false; 
}