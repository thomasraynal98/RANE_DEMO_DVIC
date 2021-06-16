#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H
 
class Robot_system
{
    private:
        // VARIABLE.
        std::string robot_id;

        // VARIABLE MICROCONTROLER. (A=COMMANDE/B=SENSOR)
        LibSerial::SerialPort** __serial_port_controle_A;
        LibSerial::SerialPort** __serial_port_sensor_B;
        std::string port_A_name;                                 // note: this is the string name of the serial port.
        std::string port_B_name;
        std::string controler_A_pong = "1/A";                    // note: the pong message is the answer from microcontroler after get "1/X" ping.
        std::string controler_B_pong = "1/B";
        int state_A_controler = 0;
        int state_B_controler = 0;

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
        // CONSTRUCTEUR.
        Robot_system(std::string val_id);

        // FONCTION.
        std::string get_id();
        LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message, bool wait_option);
        bool match_ping_pong(std::string ping, std::string pong);
        float round(float var);

        // FONCTION ANALYSE.
        void add_texte(cv::Mat image);
        void add_state(cv::Mat image, int A, std::string th_state, double hz, cv::Scalar fond);
        void add_lines(cv::Mat image);

        // THREAD.
        void thread_LOCALISATION(int frequency);
        void thread_COMMANDE(int frequency);
        void thread_LISTENER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string message_pong, std::string micro_name);
        void thread_SPEAKER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string pong_message, std::string micro_name);
        void thread_SERVER_LISTEN(int frequency);
        void thread_SERVER_SPEAKER(int frequency);
        void thread_ANALYSER(int frequency);

};
 
#endif