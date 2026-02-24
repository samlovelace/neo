
#include "ExecutionEngine.h"
#include "RateController.hpp"
#include <iostream> 

ExecutionEngine::ExecutionEngine() : ExecutionEngine(nullptr)
{

}

ExecutionEngine::ExecutionEngine(std::shared_ptr<CommunicationHandler> aCommsHandler) : mComms(aCommsHandler), mRunning(true), mActionRegistry()
{
    mActionQueue.push_back("find_object"); 
    mActionQueue.push_back("move_to"); 
    mActionQueue.push_back("pick_object"); 
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

        if (!mActiveAction)
        {
            if (mActionQueue.empty())
            {
                // TODO: something here 
                return;
            }

            auto action = mActionQueue.front();
            mActionQueue.pop_front();

            mActiveAction = mActionRegistry.create(action);
            mActiveAction->reset();
        }

        // TODO: pass context to tick 
        auto status = mActiveAction->tick();

        switch (status)
        {
            case ActionStatus::RUNNING:
                break;

            case ActionStatus::SUCCESS:
                mActiveAction.reset();
                break;

            case ActionStatus::FAILURE:
                //handleFailure();
                mActiveAction.reset();
                break;

            default:
                break;
        } 

        rate.block(); 
    }

}