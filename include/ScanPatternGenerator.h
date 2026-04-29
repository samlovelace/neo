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
    ScanPattern(const std::queue<Waypoint>& aScanPattern) 
        : mWaypoints(aScanPattern) {}
    ~ScanPattern() = default; 

    bool hasMoreWaypoints()
    {
        return !mWaypoints.empty(); 
    }

    Waypoint nextWaypoint()
    {
        if(!hasMoreWaypoints())
        {
            return Waypoint{}; 
        }

        Waypoint wp = mWaypoints.front(); 
        mWaypoints.pop(); 
        return wp; 
    }

private: 
    std::queue<Waypoint> mWaypoints; 
};

class ScanPatternGenerator
{
public: 

    static std::unique_ptr<ScanPattern> generate(const std::string& aType, const Pose6D& aCurrentPose)
    {
        // TODO: get this from config when implemented 
        std::array<double, 6> tolerance = {0.3, 0.3, 0.3, 5, 5, 5}; //arrival tolerance (xyz rpy): (m, m, m, deg, deg, deg); 

        std::queue<Waypoint> waypoints;
        Waypoint start; 
        start.goal = aCurrentPose; 
        start.tolerance = tolerance; 
        waypoints.push(start);          // first waypoint is always the current pose

        if("pirouette" == aType)
        {
            int numWaypoints = 10;
            double angleDelta = (2.0 * M_PI) / (double)numWaypoints;

            Pose6D wp;
            wp.x = aCurrentPose.x;
            wp.y = aCurrentPose.y;
            wp.z = aCurrentPose.z;

            // Extract starting yaw from current pose
            Eigen::Quaterniond q_start(
                aCurrentPose.qw, aCurrentPose.qx, aCurrentPose.qy, aCurrentPose.qz
            );
            Eigen::Vector3d rpy_start = q_start.toRotationMatrix().eulerAngles(0, 1, 2);
            double roll_start  = rpy_start[0];
            double pitch_start = rpy_start[1];
            double yaw_start   = rpy_start[2];

            std::cout << "Starting yaw: " << yaw_start << " rad" << std::endl;

            for (int i = 0; i < numWaypoints; i++)
            {
                // Accumulate yaw from start, wrapping smoothly with remainder
                double yaw = std::remainder(yaw_start + angleDelta * i, 2.0 * M_PI);

                // Rebuild quaternion preserving original roll/pitch
                Eigen::Quaterniond q_wp =
                    Eigen::AngleAxisd(roll_start,  Eigen::Vector3d::UnitX()) *
                    Eigen::AngleAxisd(pitch_start, Eigen::Vector3d::UnitY()) *
                    Eigen::AngleAxisd(yaw,         Eigen::Vector3d::UnitZ());

                wp.qx = q_wp.x();
                wp.qy = q_wp.y();
                wp.qz = q_wp.z();
                wp.qw = q_wp.w();

                Waypoint actual; 
                actual.goal = wp; 
                actual.tolerance = tolerance; 
                waypoints.push(actual);
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