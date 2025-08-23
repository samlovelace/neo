#ifndef EXECUTIONENGINE_H
#define EXECUTIONENGINE_H
 
#include <queue> 
#include <string> 
#include <vector>  
#include <memory>

#include "CommunicationHandler.h"

class ExecutionEngine 
{ 
public:
    ExecutionEngine(std::shared_ptr<CommunicationHandler> aCommsHandler);
    ~ExecutionEngine();

    void run(); 
    void execute(); 

    bool isRunning() {return mRunning; }

private:
    bool mRunning; 
    std::deque<std::string> mActions;
    std::string mCurrentAction;  

    std::vector<double> mObjectCentroid_Gl; 

    std::shared_ptr<CommunicationHandler> mComms; 
   
};
#endif //EXECUTIONENGINE_H