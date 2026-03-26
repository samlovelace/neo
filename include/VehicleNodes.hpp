#ifndef VEHICLENODES_HPP
#define VEHICLENODES_HPP

#include <memory> 

#include "behaviortree_cpp/action_node.h"

#include "VehicleInterface.h"
#include "CommonTypes.hpp"

class SendVehicleWaypointNode : public BT::SyncActionNode
{
public:
    SendVehicleWaypointNode(const std::string&      name,
                            const BT::NodeConfig&   config,
                            std::shared_ptr<VehicleInterface> aVehInterface)
        : SyncActionNode(name, config)
        , mInterface(aVehInterface) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Pose6D>>("goal_pose")
        };
    }

    BT::NodeStatus tick() override
    {
        auto goalPose = getInput<std::shared_ptr<Pose6D>>("goal_pose");

        if(!goalPose)
        {
            return BT::NodeStatus::FAILURE; 
        }

        mInterface->send(*goalPose.value());
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<VehicleInterface> mInterface;
};

/**
 * @brief PollVehicleArrivalNode
 */
class PollVehicleArrivalNode : BT::StatefulActionNode
{
public:
    PollVehicleArrivalNode(const std::string&      name,
                           const BT::NodeConfig&   config,
                           std::shared_ptr<VehicleInterface> aVehInterface)
        : StatefulActionNode(name, config)
        , mInterface(aVehInterface) {} 

    ~PollVehicleArrivalNode();

    BT::NodeStatus onStart() override
    {
        // Init timeout counter 
    }

    BT::NodeStatus onRunning() override
    {
        // TODO: add timeout and return failure if timeout reached 

        if(mInterface->isArrived())
        {
            return BT::NodeStatus::SUCCESS; 
        }

        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override
    {
        // something 
    }

private:
    std::shared_ptr<VehicleInterface> mInterface; 

};

#endif