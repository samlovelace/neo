
#include <csignal>
#include <atomic>
#include <plog/Log.h>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "RosTopicManager.hpp"
#include "ObjectData.hpp"
#include "VehicleInterface.h"
#include "DataLogger.h"

#include "NodeRegistration.hpp"
#include "BehaviorTreeLoader.hpp"

#include "ObjectNodes.hpp"

namespace
{
    std::atomic<bool> gShutdownRequested{false};

    void signalHandler(int)
    {
        gShutdownRequested = true;
        rclcpp::shutdown();  // installing our own handler replaces rclcpp's; unblock spin() ourselves
    }
}

int main()
{
    DataLogger::get().createMainLog("neo");

    rclcpp::init(0, nullptr);
    std::signal(SIGINT, signalHandler);
    RosTopicManager::getInstance()->spinNode();

    auto veh = std::make_shared<VehicleInterface>();
    while(!veh->isConnected())
    {
        if(gShutdownRequested || !rclcpp::ok())
        {
            LOGI << "shutdown requested before vehicle connected";
            rclcpp::shutdown();
            return 0;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    BT::BehaviorTreeFactory factory; 
    
    //register nodes before reading xml tree 
    registerVehicleNodes(factory, veh);
    
    auto percep = std::make_shared<PerceptionInterface>(); 

    // register object/perception nodes
    factory.registerNodeType<CheckObjectKnownNode>("CheckObjectKnown");
    factory.registerNodeType<PlanObjectApproachNode>("PlanObjectApproach"); 
    registerPerceptionNodes(factory, percep); 

    std::string treesDir = ament_index_cpp::get_package_share_directory("neo") + "/trees";
    BehaviorTreeLoader::registerTreesFromDirectory(factory, treesDir);
    auto tree = factory.createTree("FindObject");

    // initialize the object registry on the blackboard
    auto registry = std::make_shared<ObjectRegistry>(); 
    tree.rootBlackboard()->set("object_registry", registry); 
    tree.rootBlackboard()->set<std::string>("object_id", "test");  // TODO: this should come from somewhere else, maybe parsed from a natural language cmd

    LOGI << "--- starting tree ---";
    while(!gShutdownRequested && rclcpp::ok())
    {
        auto status = tree.tickOnce();

        if(status != BT::NodeStatus::RUNNING)
        {
            LOGI << "--- tree finished ---";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if(gShutdownRequested)
    {
        LOGI << "shutdown requested, exiting.";
    }

    rclcpp::shutdown();
    return 0;
}