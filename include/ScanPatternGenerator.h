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
            double angleDelta = (2 * M_PI) / (double)numWaypoints; 

            // scan pattern position stays constant 
            Pose6D wp; 
            wp.x = aCurrentPose.x; 
            wp.y = aCurrentPose.y; 
            wp.z = aCurrentPose.z; 

            std::cout << "Starting scan pattern from " << wp.x << "," << wp.y << "," << wp.z << std::endl; 
        
            // Build starting transform
            Eigen::Affine3d T_start = Eigen::Affine3d::Identity();
            T_start.translation() << aCurrentPose.x, aCurrentPose.y, aCurrentPose.z;
            T_start.linear() = Eigen::Quaterniond(
                aCurrentPose.qw, aCurrentPose.qx, aCurrentPose.qy, aCurrentPose.qz
            ).toRotationMatrix();

            std::cout << "Starting orientation (rpy): " << T_start.linear().eulerAngles(0, 1, 2).transpose() << std::endl; 

            for (int i = 0; i < numWaypoints; i++)
            {
                // Build world-frame Z rotation
                Eigen::Affine3d T_rot = Eigen::Affine3d::Identity();
                T_rot.linear() = Eigen::AngleAxisd(angleDelta * i, Eigen::Vector3d::UnitZ())
                                    .toRotationMatrix();

                // Left-multiply = world frame rotation
                Eigen::Affine3d T_wp = T_rot * T_start;

                // Extract back to Pose6D
                Eigen::Quaterniond q(T_wp.linear());

                wp.qx = q.x();
                wp.qy = q.y();
                wp.qz = q.z();
                wp.qw = q.w();
                waypoints.push(wp);
            }
        } 
        else
        {
            std::cout << "Unsupported scan pattern type!!!!! " << std::endl; 
            return nullptr; 
        }

        std::cout << "Generated scan pattern with " << waypoints.size() << " waypoints" << std::endl; 
        auto pattern = std::make_unique<ScanPattern>(waypoints); 
        return pattern; 
    }

private:
    ScanPatternGenerator() = default; 
    ~ScanPatternGenerator() = default; 
};

#endif //SCANPATTERN_H