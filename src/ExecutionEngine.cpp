
#include "ExecutionEngine.h"
#include "RateController.hpp"
#include <iostream> 

ExecutionEngine::ExecutionEngine(std::shared_ptr<CommunicationHandler> aCommsHandler) : mComms(aCommsHandler), mRunning(true)
{
    mActions.push_back("find_object"); 
    mActions.push_back("move_to"); 
    mActions.push_back("pick_object"); 

    mCurrentAction = "none"; 
}

ExecutionEngine::~ExecutionEngine()
{

}

void ExecutionEngine::run()
{
    RateController rate(25); 

    while(isRunning())
    {
        rate.start(); 
        execute();  
        rate.block(); 
    }

}

void ExecutionEngine::execute()
{
    if("none" == mCurrentAction)
    {   
        if(!mActions.empty())
        {
            mCurrentAction = mActions.front(); 
            mActions.pop_front();  
        }
    }
    else if("find_object" == mCurrentAction)
    {
        // command perception to find object and wait for response(s) 

        // TODO: hardcoded for now
        std::cout << "Found Object!" << std::endl; 
        mObjectCentroid_Gl = {1, 1, 0.5}; 
        mCurrentAction = "none"; 
    }
    else if ("move_to" == mCurrentAction)
    {
        // command vehicle to move to certain location 
        std::cout << "Moving to object..." << std::endl; 

        mComms->commandVehicle(); 
    }
    else if ("pick_object" == mCurrentAction)
    {

    }
    else 
    {
        // default behavior is to do nothing 
    }

}