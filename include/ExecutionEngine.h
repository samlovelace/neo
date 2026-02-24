#ifndef EXECUTIONENGINE_H
#define EXECUTIONENGINE_H
 
#include <queue> 
#include <string> 
#include <vector>  
#include <memory>

#include "ActionRegistry.h"
#include "IActionExecutor.hpp"
#include "CommunicationHandler.h"

class ExecutionEngine 
{ 
public:
    ExecutionEngine(std::shared_ptr<CommunicationHandler> aCommsHandler);
    ExecutionEngine(); 
    ~ExecutionEngine();

    void run(); 
    void execute(); 

    bool isRunning() {return mRunning; }

private:
    bool mRunning; 
    std::deque<std::string> mActionQueue; 
    std::unique_ptr<IActionExecutor> mActiveAction; 

    std::shared_ptr<CommunicationHandler> mComms; 
    ActionRegistry mActionRegistry; 
   
};
#endif //EXECUTIONENGINE_H