
#include <iostream> 

#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "RosTopicManager.hpp"
#include "ObjectData.hpp"
#include "VehicleInterface.h"

#include "NodeRegistration.hpp"

int main()
{   
    rclcpp::init(0, nullptr); 
    RosTopicManager::getInstance()->spinNode(); 

    auto veh = std::make_shared<VehicleInterface>();
    while(!veh->isConnected())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(250)); 
    }

    BT::BehaviorTreeFactory factory; 

    // register nodes before reading xml tree 
    registerVehicleNodes(factory, veh); 

    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="VehicleScan">
                <Sequence>
                    <GetNextScanWaypoint pattern="pirouette" goal_pose="{goal_pose}"/>
                    <SendVehicleWaypoint goal_pose="{goal_pose}"/>
                    <PollVehicleArrival/>
                </Sequence> 
            </BehaviorTree>
        </root>
    )"); 

    // initialize the object registry on the blackboard 
    auto registry = std::make_shared<ObjectRegistry>(); 
    tree.rootBlackboard()->set("object_registry", registry); 

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