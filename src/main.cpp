
#include <iostream> 

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "RosTopicManager.hpp"
#include "ObjectData.hpp"
#include "VehicleInterface.h"

#include "NodeRegistration.hpp"

#include "ObjectNodes.hpp"

int main()
{   
    rclcpp::init(0, nullptr); 
    RosTopicManager::getInstance()->spinNode(); 

    // auto veh = std::make_shared<VehicleInterface>();
    // while(!veh->isConnected())
    // {
    //     std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
    // }

    // register nodes before reading xml tree 
    //registerVehicleNodes(factory, veh);

    BT::BehaviorTreeFactory factory; 
     
    auto percep = std::make_shared<PerceptionInterface>(); 

    // register object/perception nodes
    factory.registerNodeType<CheckObjectKnownNode>("CheckObjectKnown");
    registerPerceptionNodes(factory, percep); 

    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="FindObject">
                <Fallback name="find_object_or_scan">
                    <CheckObjectKnown object_id="box"/>

                    <Sequence>
                        <SendFindObject object_id="{object_id}"/>
                        <KeepRunningUntilFailure>
                            <PollFoundObject object_id="{object_id}"/>
                        </KeepRunningUntilFailure>
                    </Sequence>
                     
                </Fallback>
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
    tree.rootBlackboard()->set<std::string>("object_id", "box");  // TODO: this should come from somewhere else, maybe parsed from a natural language cmd

    std::cout << "--- starting tree ---" << std::endl;
    while(true)
    {
        auto status = tree.tickOnce(); 
    
        if(status != BT::NodeStatus::RUNNING)
        {
            std::cout << "--- tree finished ---" << std::endl; 
            break; 
        }
    
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }


    rclcpp::shutdown(); 
    return 0; 
}