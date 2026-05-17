#ifndef PERCEPTIONINTERFACE_H
#define PERCEPTIONINTERFACE_H

#include <string> 
#include <vector> 
#include <mutex> 

#include "ptera_msgs/msg/vision_command.hpp"
#include "ptera_msgs/msg/found_object_response.hpp"

#include "ObjectData.hpp"

class PerceptionInterface
{
public:
    PerceptionInterface();
    ~PerceptionInterface();

    void findObject(const std::string& anObjectId); 

    bool popFoundObjects(std::vector<ObjectData>& aFoundObjects); 

private:
    void responseCallback(ptera_msgs::msg::FoundObjectResponse::SharedPtr aResponse); 
    std::vector<ObjectData> mFoundObjects; 
    std::mutex mFoundObjectsMutex; 

};
#endif 

