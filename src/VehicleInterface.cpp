
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

void VehicleInterface::send(const Waypoint& aGoalPose)
{
    robot_idl::msg::VehicleWaypoint wp; 
    wp.position.set__x(aGoalPose.goal.x); 
    wp.position.set__y(aGoalPose.goal.y); 
    wp.position.set__z(aGoalPose.goal.z); 

    std::cout << "Goal position " << wp.position.x << " " << wp.position.y << " " << wp.position.z << std::endl; 

    wp.quat.set__x(aGoalPose.goal.qx);
    wp.quat.set__y(aGoalPose.goal.qy);
    wp.quat.set__z(aGoalPose.goal.qz);
    wp.quat.set__w(aGoalPose.goal.qw);

    // compute euler from quat 
    Eigen::Quaterniond q(aGoalPose.goal.qw, aGoalPose.goal.qx, aGoalPose.goal.qy, aGoalPose.goal.qz); 
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(0, 1, 2); 

    std::cout << "Goal orientation " << euler.transpose() << std::endl; 
    
    wp.euler.set__yaw(euler(2)); 
    wp.euler.set__pitch(euler(1)); 
    wp.euler.set__roll(euler(0)); 

    robot_idl::msg::Vec3 pos_tol; 
    pos_tol.set__x(aGoalPose.tolerance[0]);
    pos_tol.set__x(aGoalPose.tolerance[1]);
    pos_tol.set__x(aGoalPose.tolerance[2]);

    robot_idl::msg::Euler euler_tol; 
    euler_tol.set__yaw(aGoalPose.tolerance[5]); 
    euler_tol.set__pitch(aGoalPose.tolerance[4]); 
    euler_tol.set__roll(aGoalPose.tolerance[3]); 

    wp.set__position_tolerance(pos_tol); 
    wp.set__euler_tolerance(euler_tol); 
    wp.set__execution_duration(600); // TODO: make settable per waypoint

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