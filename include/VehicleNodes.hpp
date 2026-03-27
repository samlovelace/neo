#ifndef VEHICLENODES_HPP
#define VEHICLENODES_HPP

#include <memory> 
#include <iostream> 

#include "behaviortree_cpp/bt_factory.h"
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

        std::cout << "Sending waypoint to mobile base..." << std::endl; 
        mInterface->send(*goalPose.value());
        return BT::NodeStatus::SUCCESS;
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
        std::cout << "starting vehicle arrival check...";  
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        // TODO: add timeout and return failure if timeout reached 
        std::cout << "polling vehicle arrival state..." << std::endl; 

        if(mInterface->isArrived())
        {
            std::cout << "vehicle is arrived..." << std::endl; 
            return BT::NodeStatus::SUCCESS; 
        }

        std::cout << "vehicle is still moving..." << std::endl; 
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
            BT::OutputPort<std::shared_ptr<Pose6D>>("goal_pose")
        };
    }

    BT::NodeStatus tick() override
    {
        if (!mScanPattern)
        {
            auto pattern      = getInput<std::string>("pattern").value();
            std::cout << "Generating " << pattern << " scan pattern" << std::endl; 

            auto current_pose = mInterface->currentPose();
            mScanPattern      = ScanPatternGenerator::generate(pattern, current_pose);
        }

        if (!mScanPattern->hasMoreWaypoints())
            return BT::NodeStatus::FAILURE;

        auto pose = std::make_shared<Pose6D>(mScanPattern->nextWaypoint());
        setOutput("goal_pose", pose);
        std::cout << "sending next waypoint..." << std::endl; 
        return BT::NodeStatus::SUCCESS;
    }

private:
    std::shared_ptr<VehicleInterface> mInterface;
    std::unique_ptr<ScanPattern>      mScanPattern;
};

#endif