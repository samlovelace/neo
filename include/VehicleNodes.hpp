#ifndef VEHICLENODES_HPP
#define VEHICLENODES_HPP

#include <memory> 

#include "behaviortree_cpp/action_node.h"

#include "CommonTypes.hpp"
#include "VehicleInterface.h"
#include "ScanPatternGenerator.h"

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

class GetNextScanWaypointNode : public BT::SyncActionNode
{
public:
    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("pattern"),
            BT::OutputPort<std::shared_ptr<Pose6D>>("goal_pose")
        };
    }

    BT::NodeStatus tick() override
    {
        // Generate pattern on first call
        if (!mScanPattern)
        {
            auto pattern      = getInput<std::string>("pattern").value();
            auto current_pose = mInterface->currentPose();
            mScanPattern      = std::make_unique<ScanPatternGenerator>(pattern, current_pose);
        }

        if (!mScanPattern->hasMoreWaypoints())
            return BT::NodeStatus::FAILURE;

        auto pose = std::make_shared<Pose6D>(mScanPattern->nextWaypoint());
        setOutput("goal_pose", pose);
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<VehicleInterface>       mInterface;
    std::unique_ptr<ScanPatternGenerator>   mScanPattern;
};

#endif