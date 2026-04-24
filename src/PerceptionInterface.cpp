
#include "PerceptionInterface.h"
#include "RosTopicManager.hpp"

PerceptionInterface::PerceptionInterface()
{
    RosTopicManager::getInstance()->createPublisher<robot_idl::msg::VisionCommand>("vision/command");
    
    RosTopicManager::getInstance()->createSubscriber<robot_idl::msg::FoundObjectResponse>("vision/response", 
        std::bind(&PerceptionInterface::responseCallback, this, std::placeholders::_1)); 
    
}

PerceptionInterface::~PerceptionInterface()
{
    
}

void PerceptionInterface::findObject(const std::string& anObjectId)
{
    std_msgs::msg::String cmdStr; 
    cmdStr.set__data("find_object"); 

    std_msgs::msg::String idStr; 
    idStr.set__data(anObjectId); 

    robot_idl::msg::VisionCommand cmd; 
    cmd.set__command(cmdStr); 
    cmd.set__object_type(idStr); 

    RosTopicManager::getInstance()->publishMessage("vision/command", cmd); 
}

void PerceptionInterface::responseCallback(robot_idl::msg::FoundObjectResponse::SharedPtr aResponse)
{
    // TODO: populate rest of fields 

    ObjectData objData; 
    objData.mId = aResponse->object_type.data;
    
    Pose6D pose; 
    pose.x = aResponse->obj_centroid_g.point.x;
    pose.y = aResponse->obj_centroid_g.point.y;
    pose.z = aResponse->obj_centroid_g.point.z; 
    
    pose.qw = 1; 
    pose.qx = 0; 
    pose.qy = 0; 
    pose.qz = 0; 

    std::lock_guard<std::mutex> lock(mFoundObjectsMutex); 
    mFoundObjects.push_back(objData); 
}

bool PerceptionInterface::popFoundObjects(std::vector<ObjectData>& aFoundObjects)
{
    std::lock_guard<std::mutex> lock(mFoundObjectsMutex);
    if (mFoundObjects.empty())
        return false;

    aFoundObjects = std::move(mFoundObjects);  // steals the buffer, no copy
    mFoundObjects.clear();                      // clear is a no-op after move, but explicit is fine
    return true;
}