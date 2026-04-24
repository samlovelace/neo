#ifndef PERCEPTIONINTERFACE_H
#define PERCEPTIONINTERFACE_H

#include <string> 
#include <vector> 
#include <mutex> 

#include "robot_idl/msg/vision_command.hpp"
#include "robot_idl/msg/found_object_response.hpp"

#include "ObjectData.hpp"

class PerceptionInterface
{
public:
    PerceptionInterface();
    ~PerceptionInterface();

    void findObject(const std::string& anObjectId); 

    bool popFoundObjects(std::vector<ObjectData>& aFoundObjects); 

private:
    void responseCallback(robot_idl::msg::FoundObjectResponse::SharedPtr aResponse); 
    std::vector<ObjectData> mFoundObjects; 
    std::mutex mFoundObjectsMutex; 

};
#endif 

