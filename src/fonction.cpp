#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "fonction.h"

bool test(bool is_cool)
{
    std::cout << "print in test:" << is_cool;
    return is_cool;
}

void from_quaternion_to_euler(Pose& cur_pose)
{
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (cur_pose.orientation.w * cur_pose.orientation.x + cur_pose.orientation.y * cur_pose.orientation.z);
    double cosr_cosp = 1 - 2 * (cur_pose.orientation.x * cur_pose.orientation.x + cur_pose.orientation.y * cur_pose.orientation.y);
    cur_pose.euler.r = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (cur_pose.orientation.w * cur_pose.orientation.y - cur_pose.orientation.z * cur_pose.orientation.x);
    if (std::abs(sinp) >= 1)
        cur_pose.euler.p = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        cur_pose.euler.p = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (cur_pose.orientation.w * cur_pose.orientation.z + cur_pose.orientation.x * cur_pose.orientation.y);
    double cosy_cosp = 1 - 2 * (cur_pose.orientation.y * cur_pose.orientation.y + cur_pose.orientation.z * cur_pose.orientation.z);
    cur_pose.euler.y = std::atan2(siny_cosp, cosy_cosp);
}

void tokenize(std::string const &str, const char delim,
            std::vector<std::string> &out)
{
    size_t start;
    size_t end = 0;
 
    while ((start = str.find_first_not_of(delim, end)) != std::string::npos)
    {
        end = str.find(delim, start);
        out.push_back(str.substr(start, end - start));
    }
}