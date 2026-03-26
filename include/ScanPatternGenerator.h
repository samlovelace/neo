#ifndef SCANPATTERNGENERATOR_H
#define SCANPATTERNGENERATOR_H
 
#include <string> 
#include "CommonTypes.hpp"
 
class ScanPatternGenerator 
{ 
public:
    ScanPatternGenerator();
    ~ScanPatternGenerator();

    bool generate(const std::string& aType, const Pose6D& aCurrentPose); 

    bool hasMoreWaypoints(); 
    Pose6D nextWaypoint(); 

private:


   
};
#endif //SCANPATTERNGENERATOR_H