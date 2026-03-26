
#include <iostream> 

#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"

#include "ObjectData.hpp"

int main()
{   
    BT::BehaviorTreeFactory factory; 

    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="Main">
                <Fallback>
                    <CheckObjectKnown object_type="bottle"/>
                <Sequence>
                    <SendFindObject object_type={object_type}/>
                    <Fallback>
                        <PollObjectFound object_type={object_type} timeout_s="60"/>
                        <Parallel>
                            <PollObjectFound object_type={object_type} timeout_s="30000"/>    
                            <Sequence> 
                                <ComputeNextWaypoint>
                                <SendVehicleWaypoint>
                                <WaitForVehicleArrival>
                            </Sequence> 
                        </Parallel>
                    </Fallback>
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
    
        std::this_thread::sleep_for(std::chrono::milliseconds(500)); 
    }

    return 0; 
}