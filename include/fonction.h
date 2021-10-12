#ifndef FONCTION_H
#define FONCTION_H

#include <slamcore/slamcore.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <tuple>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

typedef std::pair<int, int> Pair;
typedef std::pair<Pair, Pair> Obstacle_lines;
typedef std::tuple<double, int, int> Tuple;

struct Pose 
{
    // Store all position data in multi type.

    struct Translation
    {
        double x{0}, y{0}, z{0};

        /* compute speed. */
        double robot_speed{0};
        std::chrono::high_resolution_clock::time_point last_time;
    } position;
    struct Transformation
    {
        /* store position of center of robot. */
        double x{0}, y{0};
        double cam_to_center{0.3}; //en m
        Pair FL_sensor_pose;
        Pair RL_sensor_pose;
    } transformation;
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
        int ti{0}, tj{0};  // pixel position of center of robot.
        int vi{0}, vj{0};  // for debug i save the position vector of current orientation.
        double y_pixel{0}; // the "orientation on yaw" in pixel coordinate.
    } pixel;
    /* struct for save last pose and detect slam state. */
    struct Last_pose
    {
        double x{0}, y{0}, yaw{0};
        double threshold_relocalisation{0.5};
        int state_slamcore_tracking{0};
        std::chrono::high_resolution_clock::time_point last_time;
        bool isSame{false};
        bool isRelocalisation{false};
    } last_pose;

    void update_slam_status()
    {
        /*
            DESCRIPTION: this function will check if slam is lost, init
            or if it's working.
            (0) = initialisation
            (1) = working
            (2) = lost
        */

        /* we are in init mode. */
        if(position.x == 0 && position.y == 0)
        {
            last_pose.state_slamcore_tracking = 0;
        }
        else
        {
            if(last_pose.x == position.x && last_pose.y == position.y)
            {
                if(!last_pose.isSame)
                {
                    last_pose.isSame = true;
                    last_pose.last_time = std::chrono::high_resolution_clock::now();
                }
                else
                {
                    auto now = std::chrono::high_resolution_clock::now();
                    std::chrono::duration<double, std::milli> elapsed_time = now - last_pose.last_time;

                    /* if the pose is the same since 100ms, we are lost. */
                    if((int)elapsed_time.count() > 100)
                    {
                        last_pose.state_slamcore_tracking = 2;
                    }
                }
            }
            else
            {
                last_pose.state_slamcore_tracking = 1;   
                last_pose.isSame = false;
            }
        }
        
        last_pose.x = position.x;
        last_pose.y = position.y;
    }

    void detect_relocalisation()
    {
        /*
            DESCRIPTION: IDK if it's usefull but i will detect that.
        */
        if(last_pose.threshold_relocalisation <= sqrt(pow((last_pose.x - position.x), 2.0) \
		+ pow((last_pose.y - position.y), 2.0))) {last_pose.isRelocalisation = true;}
        else{ last_pose.isRelocalisation = false;}  
    }
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
    (1)  : visual slam / compute nav mode
    (2)  : corridor mode 
    (3)  : left wall mode 
    (4)  : right wall mode
    (5)  : safety check.
    (6)  : lost mode 
    (7)  : approach mode
    (8)  : approach mode phase QR 
    (9)  : warning mode
    (10) : reset mode
    (11) : recompute A* mode when obstacle
    (12) : takeoff mode 
    (13) : approach mode phase QR slowly move forward because don't detect qr. 
    (14) : approach mode phase final.
    */
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
    void manual_new_command(int command, int speed_lvl, int origin)
    {
        /*
            INFO:
            *speed_lvl = from 3 (max) to 1(min)
        */

        origin_commande         = origin;
        manual_commande_message = command;
        int soustracteur        = -1;
        if(speed_lvl == 3) { soustracteur = -0;}
        if(speed_lvl == 2) { soustracteur = -30;}
        if(speed_lvl == 1) { soustracteur = -150;}

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
            motor.m1L = 255 + soustracteur;
            direction.m1L_s = 0;
            motor.m1R = 255 + soustracteur;
            direction.m1R_s = 0;
            motor.m2L = 255 + soustracteur;
            direction.m2L_s = 0;
            motor.m2R = 255 + soustracteur;
            direction.m2R_s = 0;
            motor.m3L = 255 + soustracteur;
            direction.m3L_s = 0;
            motor.m3R = 255 + soustracteur;
            direction.m3R_s = 0;

            goForward = true;
            goBackward = false;
        }
        if(command == 2)
        {
            // BACKWARD.
            motor.m1L = 255 + soustracteur;
            direction.m1L_s = 1;
            motor.m1R = 255 + soustracteur;
            direction.m1R_s = 1;
            motor.m2L = 255 + soustracteur;
            direction.m2L_s = 1;
            motor.m2R = 255 + soustracteur;
            direction.m2R_s = 1;
            motor.m3L = 255 + soustracteur;
            direction.m3L_s = 1;
            motor.m3R = 255 + soustracteur;
            direction.m3R_s = 1;

            goForward = false;
            goBackward = true;
        }
        if(command == 3)
        {
            // ROTATE LEFT.
            motor.m1L = 255 + soustracteur;
            direction.m1L_s = 0;
            motor.m1R = 255 + soustracteur;
            direction.m1R_s = 1;
            motor.m2L = 255 + soustracteur;
            direction.m2L_s = 0;
            motor.m2R = 255 + soustracteur;
            direction.m2R_s = 1;
            motor.m3L = 255 + soustracteur;
            direction.m3L_s = 0;
            motor.m3R = 255 + soustracteur;
            direction.m3R_s = 1;

            goForward = false;
            goBackward = false;
        }
        if(command == 4)
        {
            // ROTATE RIGHT.
            motor.m1L = 255 + soustracteur;
            direction.m1L_s = 1;
            motor.m1R = 255 + soustracteur;
            direction.m1R_s = 0;
            motor.m2L = 255 + soustracteur;
            direction.m2L_s = 1;
            motor.m2R = 255 + soustracteur;
            direction.m2R_s = 0;
            motor.m3L = 255 + soustracteur;
            direction.m3L_s = 1;
            motor.m3R = 255 + soustracteur;
            direction.m3R_s = 0;

            goForward = false;
            goBackward = false;
        }
        if(command == 5)
        {
            // LEFT SMOOTH.
            motor.m1L = 120 + soustracteur;
            direction.m1L_s = 0;
            motor.m1R = 255 + soustracteur;
            direction.m1R_s = 0;
            motor.m2L = 120 + soustracteur;
            direction.m2L_s = 0;
            motor.m2R = 255 + soustracteur;
            direction.m2R_s = 0;
            motor.m3L = 120 + soustracteur;
            direction.m3L_s = 0;
            motor.m3R = 255 + soustracteur;
            direction.m3R_s = 0;

            goForward = true;
            goBackward = false;
        }
        if(command == 6)
        {
            // RIGHT SMOOTH.
            motor.m1L = 255 + soustracteur;
            direction.m1L_s = 0;
            motor.m1R = 120 + soustracteur;
            direction.m1R_s = 0;
            motor.m2L = 255 + soustracteur;
            direction.m2L_s = 0;
            motor.m2R = 120 + soustracteur;
            direction.m2R_s = 0;
            motor.m3L = 255 + soustracteur;
            direction.m3L_s = 0;
            motor.m3R = 120 + soustracteur;
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
        double distance_co_captor{20.0};
        double distance_centraux{270.0};
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

        /* security stop timing variable. */
        std::chrono::high_resolution_clock::time_point time_stop;
        std::chrono::duration<double, std::milli> elapsed_time_since_stop;
        int wait_time_after_stop{2500}; //en ms
    
        /* pixel position of obstacle. */
        std::vector<Pair> obstacles;
        std::vector<Obstacle_lines> lines;
        Pair last_obstacle;
        bool obstacle_add{false};

        void save_last_obstacle(int i, int j)
        {
            /* DESCRIPTION: save the last obstacle. */
            last_obstacle.first = i;
            last_obstacle.second = j;
            obstacle_add = true;
        }

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

        /* Output. */
        std::vector<int> obstacle_position;

        /* Threshold that represent 1/6 seconds. */
        int threshold_comptor = frequency / 6;

        /* If sensor value less than threshold detect increment contact, and if number of contact
        is bigger than than threshold_comptor, declare contact as an real object. */
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
         
        return -1;
    }

    bool detect_wall_situation()
    {
        /*
            DESCRIPTION: this function will detect if we are
                going into a wall.

            TODO : Complete todo because in this case, in corridor option we can
                detect all right and left wall configuration.
        */

        /* LEFT WALL. */
        double alpha           = atan((ultrasonic.ulF1-ultrasonic.ulF0*sin(architecture.angle_ultrasensor*M_PI/180))/(ultrasonic.ulF0*cos(architecture.angle_ultrasensor*M_PI/180-architecture.distance_co_captor)));
        double estimation_area = tan(alpha)*(architecture.distance_co_captor+architecture.distance_centraux)+ultrasonic.ulF1;
        double threshold_area  = 100.0;

        // TODO : put obstacle verification, and Goforward validation.
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

        /* RIGHT WALL. */
        alpha           = atan((ultrasonic.ulF2-ultrasonic.ulF3*sin(architecture.angle_ultrasensor*M_PI/180))/(ultrasonic.ulF3*cos(architecture.angle_ultrasensor*M_PI/180-architecture.distance_co_captor)));
        estimation_area = tan(alpha)*(architecture.distance_co_captor+architecture.distance_centraux)+ultrasonic.ulF2;
        
        // TODO : same that LEFT WALL.
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
    /*
        DESCRIPTION: this structure is using for A* algorythm.
    */

    // Row and Column index of its parent
    Pair parent;
    // f = g + h + t
    double f, g, h, t;
    cell()
        : parent()
        , f(0)
        , g(0)
        , h(0)
        , t(0)
    {
    }
};

struct System_param{
    /*
        DESCRIPTION: this structure will store all the fixe and
            brute data on the system.
    */

    struct Identity
    {
        std::string modele{""};
        std::string version{""};
        int matricule{-1};
        std::string exploitation{""};
        std::string prenom{""};
    }identity;
    
    struct Map
    {
        std::string localisation{""};
        int id_map{-1};
        bool StoredMapIsGood{false};
    }map;

    struct FilePath
    {
        std::string path_to_navigation_info{"../data_robot/Navigation/robot_navigation.yaml"};
        std::string path_to_current_session{"../data_robot/Navigation/map.session"};
        std::string path_to_weighted_map{"../data_robot/Navigation/map.png"};
        std::string path_to_identification{"../data_robot/Identification/robot_information.yaml"};
    }filePath;

    struct SelectTargetKeypoint
    {
        double weight_distance_RKP{0};
        double weight_target_angle{0.7};
        double weight_distance_KPD{0.5};
        double weight_isReach{-0.4};
        double mode_stKP{0};

        void mode(int mode)
        {
            /* DESCRIPTION: change value. */
            // mode normal.
            mode_stKP = mode;
            if(mode == 0)
            {
                weight_distance_RKP = 0.3;
                weight_target_angle = 0.7;
                weight_distance_KPD = 0.6;
                weight_isReach = -0.4;
            }
            // mode obstacle.
            if(mode == 1)
            {
                weight_distance_RKP = 1.0;
                weight_target_angle = 0.1;
                weight_distance_KPD = 0.1;
                weight_isReach = -0.5;
            }
        }
    } param_stKP;
};

struct Stream_cam{
    /*
        DESCRIPTION: this function will store all information
            from the current video stream.
    */

    struct QR_var{
        cv::Mat bbox, rectifiedImage;
        cv::QRCodeDetector qrDecoder = cv::QRCodeDetector();
        bool qrIsDetected{false};
        std::chrono::high_resolution_clock::time_point last_detect_time;
        std::chrono::duration<double, std::milli> elapsed_time_since_detect;
        int time_threshold{333};
        double horizontal_position{-1};  //in pixel.
        double horizontal_threshold{30}; //in pixel.
        double horizontal_center;
    }qr_var;

    int width{-1}, height{-1};

    void display_debug(cv::Mat &im, cv::Mat &bbox)
    {
        int n = bbox.rows;
        for(int i = 0 ; i < n ; i++)
        {
            cv::line(im, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>(i,2), bbox.at<float>(i,3)), cv::Scalar(0,0,0), 3);
            cv::line(im, cv::Point2i(bbox.at<float>(i,0),bbox.at<float>(i,1)), cv::Point2i(bbox.at<float>(i,6), bbox.at<float>(i,7)), cv::Scalar(0,0,0), 3);
            cv::line(im, cv::Point2i(bbox.at<float>(i,2),bbox.at<float>(i,3)), cv::Point2i(bbox.at<float>(i,4), bbox.at<float>(i,5)), cv::Scalar(0,0,0), 3);
            cv::line(im, cv::Point2i(bbox.at<float>(i,6),bbox.at<float>(i,7)), cv::Point2i(bbox.at<float>(i,4), bbox.at<float>(i,5)), cv::Scalar(0,0,0), 3);
        }
        cv::imshow("Result", im);
    }

    void detect_QR_code(cv::Mat inputImage)
    {
        /*
            DESCRIPTION: this function is call in slam call back and 
                will check if we detect a QR CODE.
        */

        // std::string data = qr_var.qrDecoder.detectAndDecode(inputImage, qr_var.bbox, qr_var.rectifiedImage);
                
        // /* if we detect QR code update value. */
        // if(qr_var.bbox.size[0] != 0.0)
        // {
        //     qr_var.horizontal_position = (qr_var.bbox.at<float>(0,0) + qr_var.bbox.at<float>(0,2))/2;
        //     qr_var.qrIsDetected        = true;
        //     qr_var.last_detect_time    = std::chrono::high_resolution_clock::now();
        // }

        // /* if we didn't detect qr code for many time we lost qr. */
        // auto now = std::chrono::high_resolution_clock::now();
        // qr_var.elapsed_time_since_detect = now - qr_var.last_detect_time;
        // if((int)qr_var.elapsed_time_since_detect.count() > qr_var.time_threshold)
        // {
        //     qr_var.horizontal_position = -1;
        //     qr_var.qrIsDetected        = false;
        // }

        /* debug display of camera. */
        // display_debug(inputImage, qr_var.bbox);
    }
};

struct Mtimer{
    /* DESCRIPTION : this structure will store all timing variable for
    the code in a proper structure. */

    std::chrono::duration<double, std::milli>      duration_t;

    std::chrono::high_resolution_clock::time_point tp_1;
    int                                            thres_1{2000}; //time to detect QR code after phase 1 approach mode.
    
    bool                                           timer_2_activate{false};               
    std::chrono::high_resolution_clock::time_point tp_2;
    int                                            thres_2{2000}; //robot will slowly move forward because it don't detect qr.

    bool                                           timer_3_activate{false};               
    std::chrono::high_resolution_clock::time_point tp_3;
    int                                            thres_3{3000}; //robot will slowly move forward because it don't detect qr.

    /* Since robot is ON. */
    std::chrono::duration<double, std::milli>      duration_tX;
    std::chrono::high_resolution_clock::time_point tp_X;

    void init_timer_approach_mode()
    {
        /* NOTE: 
        for the procedure of docking, we need to execute some action in a certain
        way to ensure stability and robustess, the bools variables are importante in
        this process and need to be push to false each time the robot do a tentative
        of docking.*/

        timer_2_activate = false;
    }
};

bool test(bool is_cool);

void from_quaternion_to_euler(Pose& cur_pose);

void tokenize(std::string const &str, const char delim, std::vector<std::string> &out);

int modulo(int a, int b);
#endif