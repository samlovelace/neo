
#include "FindObjectExecutor.h"
#include <iostream>

FindObjectExecutor::FindObjectExecutor()
{

}

FindObjectExecutor::~FindObjectExecutor()
{

}

void FindObjectExecutor::reset()
{

}

ActionStatus FindObjectExecutor::tick()
{
    std::cout << "Find object..." << std::endl; 
    return ActionStatus::RUNNING; 
}