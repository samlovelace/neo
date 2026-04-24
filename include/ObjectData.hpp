#ifndef OBJECTDATA_HPP
#define OBJECTDATA_HPP

#include <string> 
#include <vector>

#include "CommonTypes.hpp"

struct ObjectData
{
    std::string mId; // e.g cat001, soda_can001
    Pose6D mPose;
};

using ObjectRegistry = std::vector<ObjectData>; 

#endif
