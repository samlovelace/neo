
#include "ExecutionEngine.h"
#include "CommunicationHandler.h"
#include <memory>

int main()
{   
    auto comms = std::make_shared<CommunicationHandler>(); 
    comms->init(); 
    
    ExecutionEngine engine(comms); 
    engine.run(); 
}