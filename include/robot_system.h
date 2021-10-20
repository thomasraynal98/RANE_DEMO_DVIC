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
#include <vector>
#include <mutex>
#include <slamcore/slamcore.hpp>

#include <sio_client.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "fonction.h"
#include "../include/connection_listener.h"

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H

class Robot_system
{  
    private:
        // VARIABLE.
        std::string robot_id;
        std::string robot_general_state;
        std::string robot_general_state_before_lost;
        double distance_between_keypoint = 0.3;
        Stream_cam camera_data;
        Robot_sensor robot_sensor_data;
        Mtimer robot_timer;

        // VARIABLE COMMANDE.
        Robot_control robot_control;
        Robot_control robot_control_last_send;

        // VARIABLE LIDAR.
        std::vector<Data_lidar> data_lidar_sample;
        // std::vector<Point_2D> list_points;

        // VARIABLE NAVIGATION.
        // TODO : bien décrire fonctionnement des variable en dessous.
        bool slam_process_state = false; // represent pure working process.
        cv::Mat map_weighted;
        cv::Mat map_weighted_obstacle;
        Pose robot_position;
        std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slamcore;
        std::vector<Path_keypoint> keypoints_path;
        std::vector<Path_keypoint*> possible_candidate_target_keypoint;
        Path_keypoint* target_keypoint;
        Pair destination_point;
        int takeoff_phase = 1;
        bool takeoff_begin = false;
        std::chrono::high_resolution_clock::time_point takeoff_begin_time;
        bool approach_orientation_isGood = false;
        double approach_orientation_angle = -1;
        double approach_orientation_threshold = 10;
        /*
            this variable will make difference between:
            (0) = normal autonomous_nav
            (1) = go home autonomous_nav
        */
        int autonomous_nav_option = 0;
        int landing_attempt       = 0;
        int phase_approach        = -1;

        // VARIABLE FICHIER & SYSTEM.
        System_param parametre;
        std::string path_to_cpu_heat        = "/sys/class/thermal/thermal_zone1/temp";
        std::string path_to_cpu_load        = "/proc/loadavg";
        std::string path_to_fan_power       = "/";
        std::string path_to_param_yaml      = "../data/yaml/param.yaml";
        std::string path_to_weighted_map    = "../data/map_weighted/dvic2.occupancy.png";
        std::string path_to_current_session = "../data/session/dvic2.session";

        // VARIABLE MICROCONTROLER. (A=COMMANDE/B=SENSOR)
        LibSerial::SerialPort** __serial_port_controle_A = new LibSerial::SerialPort*;
        LibSerial::SerialPort** __serial_port_sensor_B = new LibSerial::SerialPort*;
        std::string port_A_name;                                 // note: this is the string name of the serial port.
        std::string port_B_name;
        std::string controler_A_pong = "0/A";                    // note: the pong message is the answer from microcontroler after get "1/X" ping.
        std::string controler_B_pong = "0/B";
        int state_A_controler = 0;                               // (0=Init/1=Connect/2=Disconnect/3=Mute)
        int state_B_controler = 0;

        // VARIABLE DEBUG.
        cv::Mat debug_visual_map;
        cv::Mat debug_sensor;
        cv::Mat debug_autonomous_ultra;
        cv::Mat debug_lidar;

        // VARIABLE INTERNE SENSOR.
        int state_sensor_cpu = 0;  //(1=CONNECT/2=DISCONNECT)
        int state_sensor_fan = 0;
        double cpu_heat;
        double cpu_load;
        double fan_power;

        // VARIABLE COMMUNICATION.
        sio::socket::ptr current_socket;
        sio::client h;
        connection_listener *l;

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
        std::thread thread_10_LIDAR;

        std::chrono::high_resolution_clock::time_point thread_1_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_2_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_3_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_4_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_5_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_6_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_7_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_8_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_9_last_hz_update;
        std::chrono::high_resolution_clock::time_point thread_10_last_hz_update;

        double thread_1_hz;
        double thread_2_hz;
        double thread_3_hz;
        double thread_4_hz;
        double thread_5_hz;
        double thread_6_hz;
        double thread_7_hz;
        double thread_8_hz;
        double thread_9_hz;
        double thread_10_hz;

        double time_since_we_consider_thread_disconnect = 500;   // note: time in ms.

    public:
        // CONSTRUCTEUR.
        Robot_system();

        // FONCTION COMMUNICATION.
        bool update_yaml(std::string new_localisation, int new_map_id);
        bool update_map(std::string new_localisation, int new_map_id, std::string update_link_session, std::string update_link_png);
        void check_map();
        void bind_events();
        void send_data_to_server();
        void send_debug_data();
        sio::message::ptr generate_keypoint_vector_message();
        sio::message::ptr generate_data_lidar_for_server();

        // FONCTION MICROCONTROLER.
        std::string get_id();
        LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message, bool wait_option);
        bool match_ping_pong(std::string ping, std::string pong);
        void get_interne_data();

        // FONCTION INITIALISATION.
        bool init_slam_sdk();
        bool init_map();
        int init_microcontroler();
        void init_thread_system();
        void init_socketio();
        bool init_basic_data();

        // FONCTION NAVIGATION.
        bool aStarSearch(cv::Mat grid, Pair& src, Pair& dest);
        double calculateHValue(const Pair src, const Pair dest);
        bool isDestination(const Pair& position, const Pair& dest);
        bool isUnBlocked(cv::Mat grid, const Pair& point);
        bool isValid(cv::Mat grid, const Pair& point);
        void from_3DW_to_2DM();
        Pair from_3DW_to_2DM2(double x, double y);
        void from_global_path_to_keypoints_path(std::stack<Pair> Path);
        double compute_distance_validation(Path_keypoint current_keypoint);
        double compute_target_angle(const Pair kp);
        double compute_vector_RKP(const Pair& kp);
        double compute_vector_RKP_2(const Pair& kpCurrent, const Pair& kp2);
        double compute_distance_RPK(const Pair& kp);
        double compute_validation_angle(const Pair& kpPrev, const Pair& kpCurrent, const Pair& kpNext);
        void select_target_keypoint();
        void return_nearest_path_keypoint(double threshold);
        bool cellIsReach();
        bool destination_reach();
        bool isInVect(std::vector<int> vector, int stuf);

        // FONCTION MODE.
        void manual_mode_process();
        void mode_checking();

        // FONCTION MOTOR.
        void secure_command_transmission();
        void compute_motor_autocommande();
        void compute_motor_autocommandeNico();

        // FONCTION DRAW ANALYSE / DEBUG
        void add_texte(cv::Mat image);
        void add_state(cv::Mat image, int A, std::string th_state, double hz, cv::Scalar fond);
        void add_state_slamcore(cv::Mat image);
        void add_lines(cv::Mat image);
        void add_intern_sensors(cv::Mat image);
        void debug_add_robot_pose(cv::Mat copy_debug_visual_map);
        void debug_add_path_keypoint(cv::Mat copy_debug_visual_map);
        void add_lines_sensor(cv::Mat image);
        void add_energy_sensor(cv::Mat image);
        void debug_message_server();
        void debug_init_debug_map();
        void debug_init_sensor();
        void debug_autonav(cv::Mat image);
        void debug_init_lidar();
        void debug_lidar_interface(cv::Mat image);
        float round(float var);

        // FONCTION SYSTEM.
        void change_mode(std::__cxx11::string& state);
        static void my_handler(int var);
        int test();

        // THREAD.
        void thread_LOCALISATION(int frequency);
        void thread_COMMANDE(int frequency);
        void thread_LISTENER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string message_pong, std::string micro_name);
        void thread_SPEAKER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string pong_message, std::string micro_name);
        void thread_SERVER_LISTEN(int frequency);
        void thread_SERVER_SPEAKER(int frequency);
        void thread_ANALYSER(int frequency);
        void thread_LIDAR(int frequency);

};

class Robot_state{
    public:
        // ALL STATE.
        Robot_state();
        inline static std::string initialisation = "initialisation";
        inline static std::string waiting        = "waiting";
        inline static std::string manual         = "manual";
        inline static std::string autonomous_nav = "autonomous_nav";
        inline static std::string compute_nav    = "compute_nav";
        inline static std::string follow         = "follow";
        inline static std::string home           = "home";
        inline static std::string approach       = "approach";
        inline static std::string charging       = "charging";
        inline static std::string cleaning       = "cleaning";
        inline static std::string patrolling     = "patrolling";
        inline static std::string reset          = "reset";
        inline static std::string debug          = "debug";
        inline static std::string warning        = "warning";
        inline static std::string lost           = "lost";
        inline static std::string takeoff        = "takeoff";
};
#endif