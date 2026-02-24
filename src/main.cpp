
#include "ExecutionEngine.h"
#include "CommunicationHandler.h"
#include <memory>
#include <rclcpp/rclcpp.hpp> 

int main()
{   
    // rclcpp::init(0, nullptr); 

    // auto comms = std::make_shared<CommunicationHandler>(); 
    // comms->init(); 
    
    ExecutionEngine engine; 
    engine.run(); 

    //rclcpp::shutdown(); 
}