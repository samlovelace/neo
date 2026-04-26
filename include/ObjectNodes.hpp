#ifndef OBJECTNODES_HPP
#define OBJECTNODES_HPP

#include <behaviortree_cpp/condition_node.h>
#include <behaviortree_cpp/action_node.h>

#include "ObjectData.hpp"
#include "PerceptionInterface.h"

class CheckObjectKnownNode : public BT::ConditionNode
{
public:

    CheckObjectKnownNode(const std::string& name, const BT::NodeConfig& config) : 
        BT::ConditionNode(name, config) 
    {}

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
                std::cout << "Object " << objectId << " is already known\n"; 
                return BT::NodeStatus::SUCCESS; 
            }
        }

        std::cout << "Could not find " << objectId << " in registry, starting scan...\n"; 
        return BT::NodeStatus::FAILURE; 
    }

};

class SendFindObjectNode : public BT::SyncActionNode
{
public: 
    SendFindObjectNode(const std::string&    name, 
                       const BT::NodeConfig& config, 
                       std::shared_ptr<PerceptionInterface> aPerceptionInterface)
        : BT::SyncActionNode(name, config), 
          mPerception(aPerceptionInterface)
    {}

    ~SendFindObjectNode() = default; 

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id")
        }; 
    }

    BT::NodeStatus tick() override 
    {
        std::string objectId = getInput<std::string>("object_id").value(); 
        mPerception->findObject(objectId);
        std::cout << "Commanded perception to find object '" << objectId << "'\n"; 
        return BT::NodeStatus::SUCCESS; 
    }


private: 
    std::shared_ptr<PerceptionInterface> mPerception; 

};

class PollFoundObjectNode : public BT::StatefulActionNode 
{
public: 
    PollFoundObjectNode(const std::string& name, 
                        const BT::NodeConfig& config, 
                        std::shared_ptr<PerceptionInterface> aPerceptionInterface) 
        : BT::StatefulActionNode(name, config), 
          mPerception(aPerceptionInterface)
    {}

    ~PollFoundObjectNode() = default; 

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::string>("object_id")
        }; 
    }

    BT::NodeStatus onStart() override
    {
        // TODO: anything else? 
        mObjectIdToFind = getInput<std::string>("object_id").value();
        mTargetFound = false;  
        return BT::NodeStatus::RUNNING; 
    }

    // TODO: add timeout and return failure on timeout 
    BT::NodeStatus onRunning() override 
    {
        std::vector<ObjectData> objects; 
        if(mPerception->popFoundObjects(objects))
        {
            std::cout << "Perception found " << objects.size() << " new objects!\n";
            for(const auto& obj : objects)
            {
                // TODO: be more selective about pushing data to the registry, check for duplicates etc. 
                auto registry = config().blackboard->get<std::shared_ptr<ObjectRegistry>>("object_registry"); 
                registry->push_back(obj); 
                std::cout << "Added object of type '" << obj.mId << "' to the registry!\n"; 

                if(mObjectIdToFind == obj.mId)
                {
                    std::cout << "Target object '" << obj.mId << "' has been found!\n";       
                    mTargetFound = true; 
                }
            }
        }

        return mTargetFound ? BT::NodeStatus::SUCCESS : BT::NodeStatus::RUNNING; 
    }

    void onHalted() override 
    {

    }

private: 
    std::string mObjectIdToFind; 
    std::shared_ptr<PerceptionInterface> mPerception; 
    bool mTargetFound; 
};



#endif