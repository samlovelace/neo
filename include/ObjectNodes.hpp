#ifndef OBJECTNODES_HPP
#define OBJECTNODES_HPP

#include "behaviortree_cpp/condition_node.h"
#include "ObjectData.hpp"

class CheckObjectKnownNode : public BT::ConditionNode
{
public:

    CheckObjectKnownNode() = default; 
    ~CheckObjectKnownNode() = default; 

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id")
        }; 
    }

    BT::NodeStatus tick() override
    {
        std::string objectId = getInput<std::string>("object_id").value(); 
        auto registry = config().blackboard->get<std::shared_ptr<ObjectRegistry>>("object_registry"); 

        for(const auto& obj : *registry)
        {
            if(objectId == obj.mId)
            {
                return BT::NodeStatus::SUCCESS; 
            }
        }

        return BT::NodeStatus::FAILURE; 
    }

};



#endif