#ifndef FONCTION_H
#define FONCTION_H

struct Pose 
{
    struct Translation
    {
        double x{0}, y{0}, z{0};
    } position;
    struct Rotation
    {
        double x{0}, y{0}, z{0}, w{1};
    } orientation;
    struct Euler
    {
        double r{0}, p{0}, y{0};
    } euler;
};

bool test(bool is_cool);

void from_quaternion_to_euler(Pose& current_pose);

#endif