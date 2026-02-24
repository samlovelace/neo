#ifndef FINDOBJECTEXECUTOR_H
#define FINDOBJECTEXECUTOR_H
 
#include "IActionExecutor.hpp"
 
class FindObjectExecutor : public IActionExecutor
{ 
public:
    FindObjectExecutor();
    ~FindObjectExecutor() override; 

    void reset() override; 
    ActionStatus tick() override; 

private:
   
};
#endif //FINDOBJECTEXECUTOR_H