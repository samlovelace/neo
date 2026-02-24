
#include "ActionRegistry.h"
#include "FindObjectExecutor.h"

ActionRegistry::ActionRegistry()
{

}

ActionRegistry::~ActionRegistry()
{

}

std::unique_ptr<IActionExecutor> ActionRegistry::create(const std::string& anActionType)
{
    std::unique_ptr<IActionExecutor> exec; 

    if("find_object" == anActionType)
    {
        exec = std::make_unique<FindObjectExecutor>(); 
    }

    return std::move(exec); 
}