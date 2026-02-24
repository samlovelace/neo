#ifndef IACTIONEXECUTOR_HPP
#define IACTIONEXECUTOR_HPP
 

enum class ActionStatus
{
    RUNNING, 
    SUCCESS, 
    FAILURE
}; 

class IActionExecutor 
{ 
public:
    virtual ~IActionExecutor() = default; 

    virtual void reset() = 0; 
    virtual ActionStatus tick() = 0;   

private:
   
};
#endif //IACTIONEXECUTOR_HPP    