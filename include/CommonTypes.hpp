#ifndef COMMONTYPES_HPP
#define COMMONTYPES_HPP

#include <array>

struct Pose6D
{
    double x, y, z; 
    double qx, qy, qz, qw; 
};

struct Waypoint
{
    Pose6D goal; 
    std::array<double, 6> tolerance; 
};

#endif