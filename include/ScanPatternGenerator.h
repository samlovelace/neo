#ifndef SCANPATTERNGENERATOR_H
#define SCANPATTERNGENERATOR_H
 
#include <string> 
#include <functional>
#include <memory> 
#include <iostream>
#include <queue> 
#include <cmath>

#include <eigen3/Eigen/Dense>

#include "CommonTypes.hpp"

class ScanPattern 
{ 
public:
    ScanPattern(const std::queue<Pose6D>& aScanPattern) 
        : mWaypoints(aScanPattern) {}
    ~ScanPattern() = default; 

    bool hasMoreWaypoints()
    {
        return !mWaypoints.empty(); 
    }

    Pose6D nextWaypoint()
    {
        if(!hasMoreWaypoints())
        {
            return Pose6D{}; 
        }

        Pose6D wp = mWaypoints.front(); 
        mWaypoints.pop(); 
        return wp; 
    }

private: 
    std::queue<Pose6D> mWaypoints; 
};

class ScanPatternGenerator
{
public: 

    static std::unique_ptr<ScanPattern> generate(const std::string& aType, const Pose6D& aCurrentPose)
    {
        std::queue<Pose6D> waypoints;
        waypoints.push(aCurrentPose); // first waypoint is always the current pose

        // based on the type, instantiate ScanPattern with certain function binding
        if("pirouette" == aType)
        {
            // compute the pirouette scan pattern and give vector of waypoints to ScanPattern
            // Probably want to make this configurable at some point but thats a future somebody problem
            // From current pose, compute 10 waypoints to rotate 360deg at current position 

            int numWaypoints = 10; 
            double angleDelta = (2 * M_PI) / numWaypoints; 

            // scan pattern position stays constant 
            Pose6D wp; 
            wp.x = aCurrentPose.x; 
            wp.y = aCurrentPose.y; 
            wp.z = aCurrentPose.z; 

            Eigen::Quaterniond start(aCurrentPose.qw, aCurrentPose.qx, aCurrentPose.qy, aCurrentPose.qz);
            Eigen::Quaterniond next; 

            for(int i = 0; i < numWaypoints; i++)
            {
                // apply rotation of some amount to starting quaternion 
                Eigen::AngleAxisd increment(angleDelta * i, Eigen::Vector3d::UnitZ()); 
                next = start * increment; 

                wp.qx = next.x(); 
                wp.qy = next.y(); 
                wp.qz = next.z(); 
                wp.qw = next.w(); 

                waypoints.push(wp); 
            }
        } 
        else
        {
            std::cout << "Unsupported scan pattern type!!!!! " << std::endl; 
            return nullptr; 
        }

        auto pattern = std::make_unique<ScanPattern>(waypoints); 
        return pattern; 
    }

private:
    ScanPatternGenerator() = default; 
    ~ScanPatternGenerator() = default; 
};

#endif //SCANPATTERN_H