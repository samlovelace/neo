
#include "VehicleInterface.h"
#include "robot_idl/msg/vehicle_waypoint.hpp"
#include "RosTopicManager.hpp"

VehicleInterface::VehicleInterface()
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::VehicleWaypoint>("vehicle/waypoint"); 

    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::ControllerStatus>("vehicle/status",
        std::bind(&VehicleInterface::statusCallback, this, std::placeholders::_1)); 
}

VehicleInterface::~VehicleInterface()
{

}

void VehicleInterface::send(const Pose6D& aGoalPose)
{
    robot_idl::msg::VehicleWaypoint wp; 
    wp.position.set__x(aGoalPose.x); 
    wp.position.set__y(aGoalPose.x); 
    wp.position.set__z(aGoalPose.x); 

    // TODO: compute euler from quat 

    wp.quat.set__x(aGoalPose.qx);
    wp.quat.set__y(aGoalPose.qy);
    wp.quat.set__z(aGoalPose.qz);
    wp.quat.set__w(aGoalPose.qw);
    
    RosTopicManager::getInstance()->publishMessage("vehicle/waypoint", wp); 
}

void VehicleInterface::statusCallback(robot_idl::msg::ControllerStatus::SharedPtr aStatus)
{
    std::lock_guard<std::mutex> lock(mStatusMutex); 
    mLatestStatus = *aStatus; 
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