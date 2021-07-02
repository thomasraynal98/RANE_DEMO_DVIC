#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>
#include "math.h"
#include <array>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <utility>
#include <slamcore/slamcore.hpp>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "fonction.h"

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

class Robot_system
{
    private:
        // VARIABLE.
        std::string robot_id;
        double battery_voltage;
        double robot_speed;
        std::string robot_general_state;
        double distance_between_keypoint;

        // NAVIGATION VARIABLE.
        cv::Mat map_weighted;
        Pose robot_position;
        std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slamcore;
        int state_slamcore_tracking = 0;
        std::vector<Path_keypoint> keypoints_path;
        Path_keypoint* target_keypoint;
        vector<Path_keypoint*> possible_candidate_target_keypoint;

        // VARIABLE FICHIER.
        std::string path_to_cpu_heat        = "/sys/class/thermal/thermal_zone1/temp";
        std::string path_to_cpu_load        = "/proc/loadavg";
        std::string path_to_fan_power       = "/";
        std::string path_to_param_yaml      = "../data/yaml/param.yaml";
        std::string path_to_weighted_map    = "../data/map_weighted/dvic1.occupancy.png";
        std::string path_to_current_session = "../data/session/dvic1.session";

        // VARIABLE MICROCONTROLER. (A=COMMANDE/B=SENSOR)
        LibSerial::SerialPort** __serial_port_controle_A;
        LibSerial::SerialPort** __serial_port_sensor_B;
        std::string port_A_name;                                 // note: this is the string name of the serial port.
        std::string port_B_name;
        std::string controler_A_pong = "1/A";                    // note: the pong message is the answer from microcontroler after get "1/X" ping.
        std::string controler_B_pong = "1/B";
        int state_A_controler = 0;  
        int state_B_controler = 0;

        // VARIABLE DEBUG.
        cv::Mat debug_visual_map;

        // VARIABLE INTERNE SENSOR.
        int state_sensor_cpu = 0;  //(1=CONNECT/2=DISCONNECT)
        int state_sensor_fan = 0;
        double cpu_heat;
        double cpu_load;
        double fan_power;

        // VARIABLE THREAD.
        std::thread thread_1_localisation;
        std::thread thread_2_commande;
        std::thread thread_3_listener_MICROA;                    // note: this threads will manage ping/pong & standards communication.
        std::thread thread_4_speaker_MICROA;
        std::thread thread_5_listener_MICROB;
        std::thread thread_6_speaker_MICROB;
        std::thread thread_7_listener_SERVER;
        std::thread thread_8_speaker_SERVER;
        std::thread thread_9_thread_ANALYSER;

        std::chrono::high_resolution_clock::time_point thread_1_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_2_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_3_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_4_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_5_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_6_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_7_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_8_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_9_last_hz_update;

        double thread_1_hz;
        double thread_2_hz;
        double thread_3_hz;
        double thread_4_hz;
        double thread_5_hz;
        double thread_6_hz;
        double thread_7_hz;
        double thread_8_hz;
        double thread_9_hz;

        double time_since_we_consider_thread_disconnect = 500;   // note: time in ms.

    public:
        typedef std::pair<int, int> Pair;
        typedef std::tuple<double, int, int> Tuple;
        struct cell {
            // Row and Column index of its parent
            Pair parent;
            // f = g + h
            double f, g, h;
            cell()
                : parent()
                , f(-1)
                , g(-1)
                , h(-1)
            {
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

        // CONSTRUCTEUR.
        Robot_system(std::string val_id);

        // FONCTION.
        std::string get_id();
        LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message, bool wait_option);
        bool match_ping_pong(std::string ping, std::string pong);
        float round(float var);
        void get_interne_data();
        void init_slam_sdk();

        // FONCTION NAVIGATION.
        void aStarSearch(cv::Mat grid, const Pair& src, const Pair& dest);
        double calculateHValue(const Pair& src, const Pair& dest);
        bool isDestination(const Pair& position, const Pair& dest);
        bool isUnBlocked(cv::Mat grid, const Pair& point);
        bool isValid(cv::Mat grid, const Pair& point);
        void from_3DW_to_2DM();
        Pair from_3DW_to_2DM2(double x, double y);
        void from_global_path_to_keypoints_path(std::stack<Pair> Path);
        double compute_distance_validation();
        double compute_target_angle();
        double compute_vector_RKP(const Pair& kp);
        double compute_vector_RKP_2(const Pair& kpCurrent, const Pair& kp2);
        double compute_distance_RPK(const Pair& kp);
        double compute_validation_angle(const Pair& kpPrev, const Pair& kpCurrent, const Pair& kpNext);
        void select_target_keypoint();
        void return_nearest_path_keypoint(int threshold);

        // FONCTION DRAW ANALYSE / DEBUG
        void add_texte(cv::Mat image);
        void add_state(cv::Mat image, int A, std::string th_state, double hz, cv::Scalar fond);
        void add_state_slamcore(cv::Mat image);
        void add_lines(cv::Mat image);
        void add_intern_sensors(cv::Mat image);
        void debug_add_robot_pose(cv::Mat copy_debug_visual_map);
        void debug_add_path_keypoint(cv::Mat copy_debug_visual_map);
        // THREAD.
        void thread_LOCALISATION(int frequency);
        void thread_COMMANDE(int frequency);
        void thread_LISTENER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string message_pong, std::string micro_name);
        void thread_SPEAKER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string pong_message, std::string micro_name);
        void thread_SERVER_LISTEN(int frequency);
        void thread_SERVER_SPEAKER(int frequency);
        void thread_ANALYSER(int frequency);

        // FONCTION DEBUG.
        void debug_message_server();
        void debug_init_debug_map();
};

class Robot_state{
    public:
        // ALL STATE.
        Robot_state();
        inline static const std::string initialisation = "initialisation";
        inline static const std::string waiting        = "waiting";
        inline static const std::string manual         = "manual";
        inline static const std::string autonomous_nav = "autonomous_nav";
        inline static const std::string follow         = "follow";
        inline static const std::string home           = "home";
        inline static const std::string compute_nav    = "compute_nav";
        inline static const std::string cleaning       = "cleaning";
        inline static const std::string patrolling     = "patrolling";
        inline static const std::string reset          = "reset";
        inline static const std::string debug          = "debug";
};
#endif