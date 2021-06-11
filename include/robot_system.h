#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H
 
class Robot_system
{
    private:
        // VARIABLE.
        std::string robot_id;

        // VARIABLE MICROCONTROLER.
        LibSerial::SerialPort** __serial_port_controle_A;
        LibSerial::SerialPort** __serial_port_sensor_B;

        // VARIABLE THREAD.
        std::thread thread_1_localisation;
        std::thread thread_2_commande;
        std::thread thread_3_listener_MICROA;       // note: this thread will manage ping/pong & standards communication.
        std::thread thread_4_speaker_MICROA;
        std::thread thread_5_listener_MICROB;
        std::thread thread_6_speaker_MICROB;
        std::thread thread_7_listener_SERVER;
        std::thread thread_8_speaker_SERVER;
        std::thread thread_9_speaker_SERVER;

    public:
        // CONSTRUCTEUR.
        Robot_system(std::string val_id);

        // FONCTION.
        std::string get_id();
        LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message, bool wait_option);
        bool match_ping_pong(std::string ping, std::string pong);

        // THREAD.
        void thread_LOCALISATION();
        void thread_COMMANDE();
        void thread_LISTENER(LibSerial::SerialPort** serial_port, int& state, std::string message_pong);
        void thread_SPEAKER(LibSerial::SerialPort** serial_port, int& state, std::string pong_message);

};
 
#endif