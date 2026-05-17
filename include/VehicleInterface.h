#ifndef VEHICLECOMMS_H
#define VEHICLECOMMS_H

#include <mutex> 

#include "CommonTypes.hpp"
#include "ptera_msgs/msg/controller_status.hpp"
#include "ptera_msgs/msg/robot_state.hpp"

class VehicleInterface 
{ 
public:
    VehicleInterface();
    ~VehicleInterface();

    void send(const Waypoint& aGoalPose);
    bool isArrived(); 
    Pose6D currentPose(); 
    bool isConnected(); 

private:
    void statusCallback(ptera_msgs::msg::ControllerStatus::SharedPtr aStatus); 
    void poseCallback(ptera_msgs::msg::RobotState::SharedPtr aState);

private: 
    std::mutex mStatusMutex; 
    ptera_msgs::msg::ControllerStatus mLatestStatus;
    bool mStatusRecvd;  

    std::mutex mPoseMutex; 
    Pose6D mLatestPose; 
    bool mPoseRecvd; 

};
#endif //VEHICLECOMMANDER_H