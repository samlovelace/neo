
#include "VehicleInterface.h"
#include "robot_idl/msg/vehicle_waypoint.hpp"
#include "RosTopicManager.hpp"
#include <eigen3/Eigen/Dense>

VehicleInterface::VehicleInterface()
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::VehicleWaypoint>("robot/vehicle/waypoint"); 

    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ControllerStatus>("robot/vehicle/controller_status",
        std::bind(&VehicleInterface::statusCallback, this, std::placeholders::_1));

    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::RobotState>("robot/state", 
        std::bind(&VehicleInterface::poseCallback, this, std::placeholders::_1)); 
}

VehicleInterface::~VehicleInterface()
{

}

void VehicleInterface::send(const Pose6D& aGoalPose)
{
    robot_idl::msg::VehicleWaypoint wp; 
    wp.position.set__x(aGoalPose.x); 
    wp.position.set__y(aGoalPose.y); 
    wp.position.set__z(aGoalPose.z); 

    wp.quat.set__x(aGoalPose.qx);
    wp.quat.set__y(aGoalPose.qy);
    wp.quat.set__z(aGoalPose.qz);
    wp.quat.set__w(aGoalPose.qw);

    // compute euler from quat 
    Eigen::Quaterniond q(aGoalPose.qw, aGoalPose.qx, aGoalPose.qy, aGoalPose.qz); 
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); 

    std::cout << "Goal orientation " << euler.transpose() << std::endl; 
    
    wp.euler.set__yaw(euler(2)); 
    wp.euler.set__pitch(euler(1)); 
    wp.euler.set__roll(euler(0)); 

    RosTopicManager::getInstance()->publishMessage("robot/vehicle/waypoint", wp); 
}

void VehicleInterface::statusCallback(robot_idl::msg::ControllerStatus::SharedPtr aStatus)
{
    std::lock_guard<std::mutex> lock(mStatusMutex); 
    mLatestStatus = *aStatus;
    
    if(!mStatusRecvd) 
        mStatusRecvd = true; 
}

void VehicleInterface::poseCallback(robot_idl::msg::RobotState::SharedPtr aState)
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
    if(robot_idl::msg::ControllerStatus::ARRIVED == mLatestStatus.arrival)
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