
#include <iostream> 
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
class SayHelloNode : public BT::SyncActionNode
{
public: 
    SayHelloNode(const std::string& aName, const BT::NodeConfig& aConfig) : SyncActionNode(aName, aConfig) 
    {

    }

    static BT::PortsList providedPorts() {return {}; }

    BT::NodeStatus tick() override
    {
        std::cout << "Hello behavior tree" << std::endl; 
        return BT::NodeStatus::SUCCESS; 
    }
};

class SlowActionNode : public BT::StatefulActionNode
{
public:
    SlowActionNode(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config)
    {

    }

    ~SlowActionNode() = default; 

    static BT::PortsList providedPorts() {
       return {
            BT::InputPort<std::string>("label"), 
            BT::InputPort<int>("ticks_to_complete")
       }; 
    }

    BT::NodeStatus onStart() override
    {
        ticks_remaining_ = getInput<int>("ticks_to_complete").value(); 
        label_           = getInput<std::string>("label").value(); 

        std::cout << "[" << label_ << "] starting, will take " 
                  << ticks_remaining_ << " ticks" << std::endl; 
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        ticks_remaining_--; 
        std::cout << "[" << label_ << "] tick, " 
                  << ticks_remaining_ << " remaining" << std::endl;
                
        if (ticks_remaining_ <= 0)
        {
            std::cout << "[" << label_ << "] complete" << std::endl;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        std::cout << "[" << label_ << "] was halted" << std::endl; 
    }

private:
    int ticks_remaining_ = 0; 
    std::string label_; 

};

struct Pose3D
{
    double x, y, z;
    
    Pose3D(double _x, double _y, double _z) : x(_x), y(_y), z(_z) {}
}; 

class SimulatePerceptionNode : public BT::StatefulActionNode
{
public: 
    SimulatePerceptionNode(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config)
    {

    }

    ~SimulatePerceptionNode() = default; 

    static BT::PortsList providedPorts() {
        return {
            BT::InputPort<int>("ticks_to_find"), 
            BT::OutputPort<std::shared_ptr<Pose3D>>("target_pose")
        };
    }

    BT::NodeStatus onStart() override
    {
        ticks_remaining_ = getInput<int>("ticks_to_find").value(); 
        std::cout << "[perception] searching..." << std::endl;
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        ticks_remaining_--; 
        if(ticks_remaining_ <= 0)
        {
            auto found = std::make_shared<Pose3D>(1.5, 2.0, 3.0);
            setOutput("target_pose", found); 
            std::cout << "[perception] object found! writing pose to blackboard" 
                      << std::endl;
            return BT::NodeStatus::SUCCESS;  
        }
        std::cout << "[perception] still searching..." << std::endl;
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override {}

private: 
    int ticks_remaining_ = 0; 

};

class MonitorPoseNode : public BT::StatefulActionNode
{
public: 
    MonitorPoseNode(const std::string& name, const BT::NodeConfig& config) : StatefulActionNode(name, config)
    {

    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::shared_ptr<Pose3D>>("target_pose")
        }; 
    }

    BT::NodeStatus onStart() override {
        return BT::NodeStatus::RUNNING; 
    }

    BT::NodeStatus onRunning() override
    {
        auto pose = getInput<std::shared_ptr<Pose3D>>("target_pose"); 
        
        if(pose && pose.has_value())
        {
            std::cout << "[monitor] pose: x=" << pose.value()->x 
                      << " y=" << pose.value()->y << " z=" << pose.value()->z << std::endl; 
        }
        else
        {
            std::cout << "[monitor] no pose yet..." << std::endl; 
        }

        return BT::NodeStatus::RUNNING; 
    }

    void onHalted() override {}
};

int main()
{   
    BT::BehaviorTreeFactory factory; 
    factory.registerNodeType<SayHelloNode>("SayHello"); 
    factory.registerNodeType<SlowActionNode>("SlowAction"); 
    factory.registerNodeType<SimulatePerceptionNode>("SimPerception"); 
    factory.registerNodeType<MonitorPoseNode>("MonitorPose"); 

    auto tree = factory.createTreeFromText(R"(
        <root BTCPP_format="4">
            <BehaviorTree ID="Main">
                <Parallel success_count="2" failure_count="1">
                    <SimPerception ticks_to_find="10" target_pose="{target_pose}"/>
                    <MonitorPose target_pose="{target_pose}"/>
                </Parallel>
            </BehaviorTree> 
        </root>
    )");

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