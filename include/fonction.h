#ifndef FONCTION_H
#define FONCTION_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <tuple>

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
    bool goForward{false};
    bool goBackward{false};

    /* Store the processus than put new command.
    (-1) : no data
    (0)  : manual
    (1)  : visual slam
    (2)  : corridor mode 
    (3)  : left wall mode 
    (4)  : right wall mode
    (5)  : safety check. */
    int origin_commande{-1}; // for debug.

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

            goForward = false;
            goBackward = false;
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

            goForward = true;
            goBackward = false;
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

            goForward = false;
            goBackward = true;
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

            goForward = false;
            goBackward = false;
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

            goForward = false;
            goBackward = false;
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

            goForward = true;
            goBackward = false;
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

            goForward = true;
            goBackward = false;
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
    struct Architecture
    {
        double angle_ultrasensor{45.0};
        double space_between_front_sensor{300.0};
        double space_between_front_lateral{20.0};
    } architecture;

    struct Ultrasonic
    {
        double ulF0{0}, ulF1{0}, ulF2{0}, ulF3{0};
        double ulB0{0}, ulB1{0}, ulB2{0};
    } ultrasonic;

    struct Ultrasonic_detection
    {
        int nbulF0{0}, nbulF1{0}, nbulF2{0}, nbulF3{0};
        int nbulB0{0}, nbulB1{0}, nbulB2{0};
    } ultra_detect;

    struct Ultrasonic_obstacle
    {
        bool obsulF0{false}, obsulF1{false}, obsulF2{false}, obsulF3{false};
        bool obsulB0{false}, obsulB1{false}, obsulB2{false};
    } ultra_obstacle;

    struct Energy
    {
        double voltage{0};
        double current{0};
    } energy;

    struct Detection_analyse
    {
        bool isSecurityStop{false};
        bool isInCorridorMode{false};
        int cfg_corridor{-1};
        bool isWallDetectionLeft{false};
        double estimateLeftWall{-1};
        bool isWallDetectionRight{false};
        double estimateRightWall{-1};
        std::chrono::high_resolution_clock::time_point time_stop;
        std::chrono::duration<double, std::milli> elapsed_time_since_stop;
        int wait_time_after_stop{1500}; //en ms
    } detection_analyse;

    std::vector<int> proximity_sensor_detection(int threshold_direct, int threshold_latera, double frequency)
    {   
        /* INPUT:
        threshold_direct  : threshold de detection d'obstacle frontale et arrière. (en mm)
        threshold_latera  : threshold de detection lateral FL/FR/BL/BR. (en mm)
        frequency         : to generate the threshold_comptor. (en Hz)

          OUTPUT: 
        obstacle_position : return vector of id of obstrued ultrason sensor.
        */

        std::vector<int> obstacle_position;
        // int threshold_comptor           = frequency / 6; // it's like say 250ms.
        int threshold_comptor = frequency / 6;

        if(ultrasonic.ulF0 < threshold_latera && ultrasonic.ulF0 != 0) { ultra_detect.nbulF0 += 1;}
        else{ ultra_detect.nbulF0 = 0;}
        if(ultrasonic.ulF1 < threshold_direct && ultrasonic.ulF1 != 0) { ultra_detect.nbulF1 += 1;}
        else{ ultra_detect.nbulF1 = 0;}
        if(ultrasonic.ulF2 < threshold_direct && ultrasonic.ulF2 != 0) { ultra_detect.nbulF2 += 1;}
        else{ ultra_detect.nbulF2 = 0;}
        if(ultrasonic.ulF3 < threshold_latera && ultrasonic.ulF3 != 0) { ultra_detect.nbulF3 += 1;}
        else{ ultra_detect.nbulF3 = 0;}
        if(ultrasonic.ulB0 < threshold_latera && ultrasonic.ulB0 != 0) { ultra_detect.nbulB0 += 1;}
        else{ ultra_detect.nbulB0 = 0;}
        if(ultrasonic.ulB1 < threshold_direct && ultrasonic.ulB1 != 0) { ultra_detect.nbulB1 += 1;}
        else{ ultra_detect.nbulB1 = 0;}
        if(ultrasonic.ulB2 < threshold_latera && ultrasonic.ulB2 != 0) { ultra_detect.nbulB2 += 1;}
        else{ ultra_detect.nbulB2 = 0;}

        /* Update Ultrasonic_obstacle detection. */
        if(ultra_detect.nbulF0 >= threshold_comptor){ ultra_obstacle.obsulF0 = true; obstacle_position.push_back(0);}
        else{ ultra_obstacle.obsulF0 = false;}
        if(ultra_detect.nbulF1 >= threshold_comptor){ ultra_obstacle.obsulF1 = true; obstacle_position.push_back(1);}
        else{ ultra_obstacle.obsulF1 = false;}
        if(ultra_detect.nbulF2 >= threshold_comptor){ ultra_obstacle.obsulF2 = true; obstacle_position.push_back(2);}
        else{ ultra_obstacle.obsulF2 = false;}
        if(ultra_detect.nbulF3 >= threshold_comptor){ ultra_obstacle.obsulF3 = true; obstacle_position.push_back(3);}
        else{ ultra_obstacle.obsulF3 = false;}
        if(ultra_detect.nbulB0 >= threshold_comptor){ ultra_obstacle.obsulB0 = true; obstacle_position.push_back(4);}
        else{ ultra_obstacle.obsulB0 = false;}
        if(ultra_detect.nbulB1 >= threshold_comptor){ ultra_obstacle.obsulB1 = true; obstacle_position.push_back(5);}
        else{ ultra_obstacle.obsulB1 = false;}
        if(ultra_detect.nbulB2 >= threshold_comptor){ ultra_obstacle.obsulB2 = true; obstacle_position.push_back(6);}
        else{ ultra_obstacle.obsulB2 = false;}
        
        return obstacle_position;
    }

    bool detect_corridor_situation()
    {
        /*
            DESCRIPTION: this important function will detect an
            corridor situation and update isInCorridorMode.
        */

        int block_number = 0;

        if(ultra_obstacle.obsulF0) { block_number += 1;}
        if(ultra_obstacle.obsulF3) { block_number += 1;}
        if(ultra_obstacle.obsulB0) { block_number += 1;}
        if(ultra_obstacle.obsulB2) { block_number += 1;}

        if(block_number >= 2)
        {
            detection_analyse.isInCorridorMode = true;
            return true;
        }
        detection_analyse.isInCorridorMode = false;
        return false;
    }

    void sort_vector(std::vector<double> list)
    {
        /*
            DESCRIPTION: sort vector from 0.01 to infinity.
            0.0 is at the end.
        */
    }

    int get_corridor_configuration()
    {
        /*
            DESCRIPTION: we need to know, witch kind of configuration
            corridor we are actually.
            VARIABLE:
            (0) = Front
            (1) = Back
            (2) = Left
            (3) = Right
            (4) = Diagonal Left
            (5) = Diagonal Right
            (-1)= Error
            INT SENSOR:
            (0) = ulF0
            (1) = ulF3
            (2) = ulB0
            (3) = ulB2
        */

        std::vector<Tuple> list;
        auto c0 = std::make_tuple(ultrasonic.ulF3, 0, ultra_obstacle.obsulF0);
        auto c1 = std::make_tuple(ultrasonic.ulF3, 1, ultra_obstacle.obsulF3);        
        auto c2 = std::make_tuple(ultrasonic.ulB0, 2, ultra_obstacle.obsulB0);
        auto c3 = std::make_tuple(ultrasonic.ulB2, 3, ultra_obstacle.obsulB2);       
        list.push_back(c0);
        list.push_back(c1);
        list.push_back(c2);
        list.push_back(c3);

        
        // found the most small.
        int min = 99999;
        int index = -1;
        for(auto i : list)
        {
            // std::cout << "[" << std::get<0>(i) << " " << std::get<1>(i) << " " << std::get<2>(i) << "]\n";
            if(std::get<0>(i) < min && std::get<2>(i) && std::get<0>(i) != 0)
            {
                index = std::get<1>(i);
                min   = std::get<0>(i);
            }
        }

        // found the second one.
        min = 99999;
        int index2 = -1;
        for(auto i : list)
        {
            if(std::get<0>(i) < min && std::get<2>(i) && std::get<0>(i) != 0 && std::get<1>(i) != index)
            {
                index2 = std::get<1>(i);
                min    = std::get<0>(i);
            }
        }

        std::cout << "[INDEX:" << index << "/" << index2 << "\n";
        // if result good.
        if(index != -1 && index2 != -1)
        {
            if((index == 0 && index2 == 1) || (index == 1 && index2 == 0))
            {
                detection_analyse.cfg_corridor = 0; return 0;
            }
            if((index == 3 && index2 == 2) || (index == 2 && index2 == 3))
            {
                detection_analyse.cfg_corridor = 1; return 1;
            }
            if((index == 0 && index2 == 2) || (index == 2 && index2 == 0))
            {
                detection_analyse.cfg_corridor = 2; return 2;
            }
            if((index == 1 && index2 == 3) || (index == 3 && index2 == 1))
            {
                detection_analyse.cfg_corridor = 3; return 3;
            }
            if((index == 0 && index2 == 3) || (index == 3 && index2 == 0))
            {
                detection_analyse.cfg_corridor = 4; return 4;
            }
            if((index == 1 && index2 == 2) || (index == 2 && index2 == 1))
            {
                detection_analyse.cfg_corridor = 5; return 5;
            }
        }
        // detection_analyse.cfg_corridor = -1; 
        return -1;
    }



    bool detect_wall_situation()
    {
        /*
            DESCRIPTION: this function will detect if we are
            going into a wall.
        */

        double D4              = 2 * cos(architecture.angle_ultrasensor*M_PI/180) / ultrasonic.ulF0 ;
        double estimation_area = D4/((D4+architecture.space_between_front_sensor+architecture.space_between_front_lateral) * ultrasonic.ulF1);
        double threshold_area  = 80.0;

        if(ultra_obstacle.obsulF0 && ultra_obstacle.obsulF1 && \
        ultra_obstacle.obsulF2)
        {
            if(ultrasonic.ulF2 >= (estimation_area - threshold_area) && \
            ultrasonic.ulF2 <= (estimation_area + threshold_area))
            {
                detection_analyse.isWallDetectionLeft = true;
            }
            else
            {
                detection_analyse.isWallDetectionLeft = false;
            }
            detection_analyse.estimateLeftWall = estimation_area; //debug
        }
        else
        {
            detection_analyse.estimateLeftWall = -1; //debug
        }

        D4              = 2 * cos(architecture.angle_ultrasensor*M_PI/180) / ultrasonic.ulF3 ;
        estimation_area = D4/((D4+architecture.space_between_front_sensor+architecture.space_between_front_lateral) * ultrasonic.ulF2);

        if(ultra_obstacle.obsulF1 && ultra_obstacle.obsulF2 && \
        ultra_obstacle.obsulF3)
        {
            if(ultrasonic.ulF1 >= (estimation_area - threshold_area) && \
            ultrasonic.ulF1 <= (estimation_area + threshold_area))
            {
                detection_analyse.isWallDetectionRight = true;
            }
            else
            {
                detection_analyse.isWallDetectionRight = false;
            }
            detection_analyse.estimateRightWall = estimation_area; //debug
        }
        else
        {
            detection_analyse.estimateRightWall = -1; //debug
        }
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