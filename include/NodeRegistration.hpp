#ifndef NODEREGISTRATION_HPP
#define NODEREGISTRATION_HPP

#include "behaviortree_cpp/bt_factory.h"

#include "VehicleInterface.h"
#include "VehicleNodes.hpp"

#include "PerceptionInterface.h"
#include "ObjectNodes.hpp"

inline void registerVehicleNodes(BT::BehaviorTreeFactory& factory,
                                 std::shared_ptr<VehicleInterface> vehicle_interface)
{
    factory.registerBuilder<SendVehicleWaypointNode>(
        "SendVehicleWaypoint",
        [vehicle_interface](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<SendVehicleWaypointNode>(name, config, vehicle_interface);
        });

    factory.registerBuilder<PollVehicleArrivalNode>(
        "PollVehicleArrival",
        [vehicle_interface](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<PollVehicleArrivalNode>(name, config, vehicle_interface);
        });

    factory.registerBuilder<GetNextScanWaypointNode>(
        "GetNextScanWaypoint",
        [vehicle_interface](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<GetNextScanWaypointNode>(name, config, vehicle_interface);
        });
}

inline void registerPerceptionNodes(BT::BehaviorTreeFactory& factory, 
                                    std::shared_ptr<PerceptionInterface> perception_interface)
{
    factory.registerBuilder<SendFindObjectNode>(
        "SendFindObject",
        [perception_interface](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<SendFindObjectNode>(name, config, perception_interface);
        });

    factory.registerBuilder<PollFoundObjectNode>(
        "PollFoundObject",
        [perception_interface](const std::string& name, const BT::NodeConfig& config)
        {
            return std::make_unique<PollFoundObjectNode>(name, config, perception_interface);
        });
}

#endif