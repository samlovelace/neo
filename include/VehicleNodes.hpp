#ifndef VEHICLENODES_HPP
#define VEHICLENODES_HPP

#include <memory>
#include <plog/Log.h>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"

#include "CommonTypes.hpp"
#include "VehicleInterface.h"
#include "ScanPatternGenerator.h"

class SendVehicleWaypointNode : public BT::StatefulActionNode
{
public:
    SendVehicleWaypointNode(const std::string&      name,
                            const BT::NodeConfig&   config,
                            std::shared_ptr<VehicleInterface> aVehInterface)
        : StatefulActionNode(name, config)
        , mInterface(aVehInterface) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Waypoint>>("goal_pose")
        };
    }

    BT::NodeStatus onStart() override 
    {
        auto goalPose = getInput<std::shared_ptr<Waypoint>>("goal_pose");

        if(!goalPose)
        {
            return BT::NodeStatus::FAILURE; 
        }

        LOGD << "Sending waypoint to mobile base...";
        mInterface->send(*goalPose.value());
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        if(mInterface->isArrived())
        {
            LOGV << "Waiting for vehicle ack...";
            return BT::NodeStatus::RUNNING; 
        }

        return BT::NodeStatus::SUCCESS;
    }

    void onHalted() override
    {

    }

private:
    std::shared_ptr<VehicleInterface> mInterface;
};

/**
 * @brief PollVehicleArrivalNode
 */
class PollVehicleArrivalNode : public BT::StatefulActionNode
{
public:
    PollVehicleArrivalNode(const std::string&      name,
                           const BT::NodeConfig&   config,
                           std::shared_ptr<VehicleInterface> aVehInterface)
        : StatefulActionNode(name, config)
        , mInterface(aVehInterface) {} 

    ~PollVehicleArrivalNode() = default; 

    BT::NodeStatus onStart() override
    {
        // Init timeout counter
        LOGD << "starting vehicle arrival check...";
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        // TODO: add timeout and return failure if timeout reached
        //LOGV << "polling vehicle arrival state...";

        if(mInterface->isArrived())
        {
            LOGI << "vehicle is arrived...";
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
    GetNextScanWaypointNode(const std::string&        name,
                            const BT::NodeConfig&     config,
                            std::shared_ptr<VehicleInterface> vehicle_interface)
        : SyncActionNode(name, config)
        , mInterface(vehicle_interface) {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("pattern"),
            BT::OutputPort<std::shared_ptr<Waypoint>>("goal_pose")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!mScanPattern)
        {
            auto pattern      = getInput<std::string>("pattern").value();
            LOGD << "Generating " << pattern << " scan pattern";

            auto current_pose = mInterface->currentPose();
            mScanPattern      = ScanPatternGenerator::generate(pattern, current_pose);
        }

        if (!mScanPattern->hasMoreWaypoints())
            return BT::NodeStatus::FAILURE;

        auto pose = std::make_shared<Waypoint>(mScanPattern->nextWaypoint());
        setOutput("goal_pose", pose);
        LOGV << "sending next waypoint...";
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<VehicleInterface> mInterface;
    std::unique_ptr<ScanPattern>      mScanPattern;
};

#endif