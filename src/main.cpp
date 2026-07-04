
#include <csignal>
#include <atomic>
#include <plog/Log.h>

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "RosTopicManager.hpp"
#include "ObjectData.hpp"
#include "VehicleInterface.h"
#include "DataLogger.h"

#include "NodeRegistration.hpp"

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
            LOGI << "--- shutdown requested before vehicle connected ---";
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

    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4" main_tree_to_execute="FindObject">
            <BehaviorTree ID="FindObject">
                <Fallback name="find_object_or_scan">
                    <CheckObjectKnown object_id="{object_id}"/>
                    <Sequence>
                        <SendFindObject object_id="{object_id}"/>
                        <Parallel success_count="1" failure_count="2">
                            <PollFoundObject object_id="{object_id}"/>
                            <SubTree ID="VehicleScan"/>
                        </Parallel>
                        <SubTree ID="ApproachObject" object_id="{object_id}" _autoremap="true"/>
                    </Sequence>
                </Fallback>
            </BehaviorTree>

            <BehaviorTree ID="VehicleScan">
                <KeepRunningUntilFailure>
                    <Sequence>
                        <GetNextScanWaypoint pattern="pirouette" goal_pose="{goal_pose}"/>
                        <SendVehicleWaypoint goal_pose="{goal_pose}"/>
                        <PollVehicleArrival/>
                    </Sequence>
                </KeepRunningUntilFailure>
            </BehaviorTree>

            <BehaviorTree ID="ApproachObject">
                <Sequence>
                    <PlanObjectApproach object_id="{object_id}" goal_pose="{goal_pose}"/>
                    <SendVehicleWaypoint goal_pose="{goal_pose}"/>
                    <PollVehicleArrival/>
                </Sequence>
            </BehaviorTree> 

        </root>
            )"); 

    // auto tree = factory.createTreeFromText(R"(
    //     <root BTCPP_format="4">
    //         <BehaviorTree ID="VehicleScan">
    //             <KeepRunningUntilFailure>
    //                 <Sequence>
    //                     <GetNextScanWaypoint pattern="pirouette" goal_pose="{goal_pose}"/>
    //                     <SendVehicleWaypoint goal_pose="{goal_pose}"/>
    //                     <PollVehicleArrival/>
    //                 </Sequence> 
    //             </KeepRunningUntilFailure>
    //         </BehaviorTree>
    //     </root>
    // )"); 

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