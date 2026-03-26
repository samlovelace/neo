#ifndef VEHICLECOMMS_H
#define VEHICLECOMMS_H

#include "CommonTypes.hpp"
#include "robot_idl/msg/controller_status.hpp"

class VehicleInterface 
{ 
public:
    VehicleInterface();
    ~VehicleInterface();

    void send(const Pose6D& aGoalPose);
    bool isArrived(); 

private:
    void statusCallback(robot_idl::msg::ControllerStatus::SharedPtr aStatus); 

private: 
    std::mutex mStatusMutex; 
    robot_idl::msg::ControllerStatus mLatestStatus; 

};
#endif //VEHICLECOMMANDER_H