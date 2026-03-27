#ifndef VEHICLECOMMS_H
#define VEHICLECOMMS_H

#include <mutex> 

#include "CommonTypes.hpp"
#include "robot_idl/msg/controller_status.hpp"
#include "robot_idl/msg/robot_state.hpp"

class VehicleInterface 
{ 
public:
    VehicleInterface();
    ~VehicleInterface();

    void send(const Pose6D& aGoalPose);
    bool isArrived(); 
    Pose6D currentPose(); 
    bool isConnected(); 

private:
    void statusCallback(robot_idl::msg::ControllerStatus::SharedPtr aStatus); 
    void poseCallback(robot_idl::msg::RobotState::SharedPtr aState);

private: 
    std::mutex mStatusMutex; 
    robot_idl::msg::ControllerStatus mLatestStatus;
    bool mStatusRecvd;  

    std::mutex mPoseMutex; 
    Pose6D mLatestPose; 
    bool mPoseRecvd; 

};
#endif //VEHICLECOMMANDER_H