
#include "PerceptionInterface.h"
#include "RosTopicManager.hpp"

PerceptionInterface::PerceptionInterface()
{
    RosTopicManager::getInstance()->createPublisher<ptera_msgs::msg::VisionCommand>("vision/command");
    
    RosTopicManager::getInstance()->createSubscriber<ptera_msgs::msg::FoundObjectResponse>("vision/response", 
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

    ptera_msgs::msg::VisionCommand cmd; 
    cmd.set__command(cmdStr); 
    cmd.set__object_type(idStr); 

    RosTopicManager::getInstance()->publishMessage("vision/command", cmd); 
}

void PerceptionInterface::responseCallback(ptera_msgs::msg::FoundObjectResponse::SharedPtr aResponse)
{
    // TODO: populate rest of fields 

    ObjectData objData; 
    objData.mId = aResponse->object_type.data;
    
    Pose6D pose; 
    pose.x = aResponse->obj_pose_g.pose.position.x;
    pose.y = aResponse->obj_pose_g.pose.position.y;
    pose.z = aResponse->obj_pose_g.pose.position.z; 
    
    pose.qw = aResponse->obj_pose_g.pose.orientation.w; 
    pose.qx = aResponse->obj_pose_g.pose.orientation.x; 
    pose.qy = aResponse->obj_pose_g.pose.orientation.y; 
    pose.qz = aResponse->obj_pose_g.pose.orientation.z; 

    objData.mPose = pose; 

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