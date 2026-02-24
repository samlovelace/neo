#ifndef ACTIONREGISTRY_H
#define ACTIONREGISTRY_H
 
#include <memory> 
#include <string> 
#include "IActionExecutor.hpp"
 
class ActionRegistry 
{ 
public:
    ActionRegistry();
    ~ActionRegistry();

    std::unique_ptr<IActionExecutor> create(const std::string& anActionType); 

private:
   
};
#endif //ACTIONREGISTRY_H   