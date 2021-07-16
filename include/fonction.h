#ifndef FONCTION_H
#define FONCTION_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

typedef std::pair<int, int> Pair;
typedef std::tuple<double, int, int> Tuple;

struct Pose 
{
    // Store all position data in multi type.

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
    struct Pixel
    {
        int i{0}, j{0};
        double y_pixel{0}; // the "orientation on yaw" in pixel coordinate.
    } pixel;
};

struct Robot_control
{
    // Store all information for motor command in this type.
    struct Motor
    {
        int m1L{0}, m2L{0}, m3L{0};
        int m1R{0}, m2R{0}, m3R{0};
    } motor;
    struct Direction
    {
        int m1L_s{0}, m2L_s{0}, m3L_s{0};
        int m1R_s{0}, m2R_s{0}, m3R_s{0};
    } direction;
    struct Servo
    {
        int SL{0}, SR{0};
    } servo;

    bool isTransmitA{false};
    bool isTransmitB{false};
    int manual_commande_message{0};
    std::string message_microcontrolerA{""};
    std::string message_microcontrolerB{""};

    bool operator==(Robot_control& ctr2)
    {
        return motor.m1L == ctr2.motor.m1L && motor.m2L == ctr2.motor.m2L &&\
               motor.m3L == ctr2.motor.m3L && motor.m1R == ctr2.motor.m1R &&\
               motor.m2R == ctr2.motor.m2R && motor.m3R == ctr2.motor.m3R &&\
               direction.m1L_s == ctr2.direction.m1L_s && direction.m2L_s == ctr2.direction.m2L_s &&\
               direction.m3L_s == ctr2.direction.m3L_s && direction.m1R_s == ctr2.direction.m1R_s &&\
               direction.m2R_s == ctr2.direction.m2R_s && direction.m3R_s == ctr2.direction.m3R_s;
    }
    void operator=(Robot_control& ctr2)
    {
        motor.m1L = ctr2.motor.m1L;
        motor.m2L = ctr2.motor.m2L;
        motor.m3L = ctr2.motor.m3L;
        motor.m1R = ctr2.motor.m1R;
        motor.m2R = ctr2.motor.m2R;
        motor.m3R = ctr2.motor.m3R;

        direction.m1L_s = ctr2.direction.m1L_s;
        direction.m2L_s = ctr2.direction.m2L_s;
        direction.m3L_s = ctr2.direction.m3L_s;
        direction.m1R_s = ctr2.direction.m1R_s;
        direction.m2R_s = ctr2.direction.m2R_s;
        direction.m3R_s = ctr2.direction.m3R_s;
    }
    void compute_message_microA()
    {
        /*
            DESCRIPTION  : Send a message to the controler if it's different.
            INPUT        :
                * current_command    = Type(Np.array)     this is the previous message sent to the micro-controller.
                * new_command        = Type(Np.array)     this is the new command to send.
                * ser                = Type(Serial)       this is the serial object.
        */

        std::string message_string;

        message_string  = "1/";
        message_string += std::to_string(direction.m1L_s) + "/" + std::to_string(motor.m1L) + "/";
        message_string += std::to_string(direction.m2L_s) + "/" + std::to_string(motor.m2L) + "/";
        message_string += std::to_string(direction.m3L_s) + "/" + std::to_string(motor.m3L) + "/";
        message_string += std::to_string(direction.m1R_s) + "/" + std::to_string(motor.m1R) + "/";
        message_string += std::to_string(direction.m2R_s) + "/" + std::to_string(motor.m2R) + "/";
        message_string += std::to_string(direction.m3R_s) + "/" + std::to_string(motor.m3R);
        message_microcontrolerA = message_string;
    }
    void compute_message_microB()
    {
        /*
            DESCRIPTION  : Send a message to the controler if it's different.
            INPUT        :
                * current_command    = Type(Np.array)     this is the previous message sent to the micro-controller.
                * new_command        = Type(Np.array)     this is the new command to send.
                * ser                = Type(Serial)       this is the serial object.
        */

        std::string message_string;

        message_string  = "1/";
        message_string += std::to_string(servo.SL) + "/" + std::to_string(servo.SR);
        message_microcontrolerB = message_string;
    }
    void change_servo(Robot_control new_command_send)
    {
        servo.SL = new_command_send.servo.SL;
        servo.SR = new_command_send.servo.SR;
    }   
    bool isServo_different(Robot_control new_command_send)
    {
        return servo.SL != new_command_send.servo.SL || \
               servo.SR != new_command_send.servo.SR;
    }
    void manual_new_command(int command)
    {
        manual_commande_message = command;

        /* Change command. */
        if(command == 0)
        {
            // STOP.
            motor.m1L = 0;
            direction.m1L_s = 0;
            motor.m1R = 0;
            direction.m1R_s = 0;
            motor.m2L = 0;
            direction.m2L_s = 0;
            motor.m2R = 0;
            direction.m2R_s = 0;
            motor.m3L = 0;
            direction.m3L_s = 0;
            motor.m3R = 0;
            direction.m3R_s = 0;

            // STOP SERVO
            servo.SL = 0;
            servo.SR = 0;
        }
        if(command == 1)
        {
            // FORWARD.
            motor.m1L = 255;
            direction.m1L_s = 0;
            motor.m1R = 255;
            direction.m1R_s = 0;
            motor.m2L = 255;
            direction.m2L_s = 0;
            motor.m2R = 255;
            direction.m2R_s = 0;
            motor.m3L = 255;
            direction.m3L_s = 0;
            motor.m3R = 255;
            direction.m3R_s = 0;
        }
        if(command == 2)
        {
            // BACKWARD.
            motor.m1L = 255;
            direction.m1L_s = 1;
            motor.m1R = 255;
            direction.m1R_s = 1;
            motor.m2L = 255;
            direction.m2L_s = 1;
            motor.m2R = 255;
            direction.m2R_s = 1;
            motor.m3L = 255;
            direction.m3L_s = 1;
            motor.m3R = 255;
            direction.m3R_s = 1;
        }
        if(command == 3)
        {
            // ROTATE LEFT.
            motor.m1L = 255;
            direction.m1L_s = 0;
            motor.m1R = 255;
            direction.m1R_s = 1;
            motor.m2L = 255;
            direction.m2L_s = 0;
            motor.m2R = 255;
            direction.m2R_s = 1;
            motor.m3L = 255;
            direction.m3L_s = 0;
            motor.m3R = 255;
            direction.m3R_s = 1;
        }
        if(command == 4)
        {
            // ROTATE RIGHT.
            motor.m1L = 255;
            direction.m1L_s = 1;
            motor.m1R = 255;
            direction.m1R_s = 0;
            motor.m2L = 255;
            direction.m2L_s = 1;
            motor.m2R = 255;
            direction.m2R_s = 0;
            motor.m3L = 255;
            direction.m3L_s = 1;
            motor.m3R = 255;
            direction.m3R_s = 0;
        }
        if(command == 5)
        {
            // LEFT SMOOTH.
            motor.m1L = 120;
            direction.m1L_s = 0;
            motor.m1R = 255;
            direction.m1R_s = 0;
            motor.m2L = 120;
            direction.m2L_s = 0;
            motor.m2R = 255;
            direction.m2R_s = 0;
            motor.m3L = 120;
            direction.m3L_s = 0;
            motor.m3R = 255;
            direction.m3R_s = 0;
        }
        if(command == 6)
        {
            // RIGHT SMOOTH.
            motor.m1L = 255;
            direction.m1L_s = 0;
            motor.m1R = 120;
            direction.m1R_s = 0;
            motor.m2L = 255;
            direction.m2L_s = 0;
            motor.m2R = 120;
            direction.m2R_s = 0;
            motor.m3L = 255;
            direction.m3L_s = 0;
            motor.m3R = 120;
            direction.m3R_s = 0;
        }
        if(command == 7)
        {
            // SERVO TESTING.
            servo.SL = 255;
            servo.SR = 100;
        }
        if(command == 8)
        {
            // SERVO TESTING.
            servo.SL = 100;
            servo.SR = 255;
        }
    }
};

struct Robot_sensor
{
    struct Ultrasonic
    {
        double ulF0{0}, ulF1{0}, ulF2{0}, ulF3{0};
        double ulB0{0}, ulB1{0}, ulB2{0};
    } ultrasonic;
    struct Energy
    {
        double voltage{0};
        double current{0};
    } energy;

    int proximity_sensor_detection_front()
    {
        /* Check if a sensor is blocked. */

        double threshold = 100; //mm
        if(ultrasonic.ulF0 < threshold) { return 1;}
        if(ultrasonic.ulF1 < threshold) { return 2;}
        if(ultrasonic.ulF2 < threshold) { return 3;}
        if(ultrasonic.ulF3 < threshold) { return 4;}
        return 0;
    }
    int proximity_sensor_detection_back()
    {
        /* Check if a sensor is blocked. */

        double threshold = 100; //mm
        if(ultrasonic.ulB0 < threshold) { return 5;}
        if(ultrasonic.ulB1 < threshold) { return 6;}
        if(ultrasonic.ulB2 < threshold) { return 7;}
        return 0;
    }
};

struct Path_keypoint {
    /*
        DESCRIPTION: this structure include all information
            of path between robot and destination.
    */

    // Pixel coordinate of path keypoint.
    Pair coordinate;
    // Do the robot reach this keypoint.
    bool isReach;
    // Distance from robot to keypoint.
    double distance_RKP;
    // Distance from current keypoint to destination. (Pas à vol d'oiseau)
    double distance_KPD;
    // Angle between robot orientation and robot to keypoint orientation.
    double target_angle; 
    // Angle between last KP, futur KP & with current KP to middle.
    double validation_angle;
    // True if this pixel is try_avoid area.
    bool isTryAvoidArea;
    // Distance to declare that robot reach point.
    double distance_validation;
    
    // Constructor
    Path_keypoint()
        : coordinate()
        , isReach(false)
        , distance_RKP(-1)
        , distance_KPD(-1)
        , target_angle(-1)
        , validation_angle(-1)
        , isTryAvoidArea(false)
        , distance_validation(-1)
        {}
};

struct cell {
    // Row and Column index of its parent
    Pair parent;
    // f = g + h
    double f, g, h, t, w;
    cell()
        : parent()
        , f(-1)
        , g(-1)
        , h(-1)
        , t(-1)
    {
    }
};

bool test(bool is_cool);

void from_quaternion_to_euler(Pose& current_pose);

void tokenize(std::string const &str, const char delim, std::vector<std::string> &out);

int modulo(int a, int b);
#endif