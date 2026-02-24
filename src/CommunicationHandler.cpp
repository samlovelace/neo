
#include "CommunicationHandler.h"
#include "RosTopicManager.hpp"
#include "robot_idl/msg/vec3.hpp"
#include "robot_idl/msg/quaternion.hpp"
#include "robot_idl/msg/vehicle_waypoint.hpp"

CommunicationHandler::CommunicationHandler()
{

}

CommunicationHandler::~CommunicationHandler()
{

}

bool CommunicationHandler::init()
{
    auto topicManager = RosTopicManager::getInstance();
    topicManager->createPublisher<robot_idl::msg::VehicleWaypoint>("/vehicle/waypoint"); 
    
    topicManager->spinNode(); 
    
    while (!topicManager->isROSInitialized())
    {
        // do nothing
    }

    //LOGD << "ROS Comms Initialized";
    return true; 
}

bool CommunicationHandler::commandVehicle(const std::vector<double>& aVehiclePos_gl)
{
    if(3 != aVehiclePos_gl.size())
    {
        std::cout << "Incorrect waypoint size" << std::endl; 
        return false; 
    }

    robot_idl::msg::Vec3 pos; 
    pos.set__x(aVehiclePos_gl[0]); 
    pos.set__y(aVehiclePos_gl[1]); 
    pos.set__z(0.0); 

    robot_idl::msg::Quaternion quat; 
    quat.set__w(0.7071); 
    quat.set__x(0.0); 
    quat.set__y(0.0); 
    quat.set__z(0.7071); 

    robot_idl::msg::VehicleWaypoint wp; 
    wp.set__position(pos); 
    wp.set__orientation(quat); 

    RosTopicManager::getInstance()->publishMessage<robot_idl::msg::VehicleWaypoint>("/vehicle/waypoint", wp); 
    return true; 
}