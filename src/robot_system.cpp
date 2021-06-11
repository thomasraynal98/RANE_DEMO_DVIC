#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>

// TODO: remove 99 100


#include "../include/robot_system.h"


// CONSTRUCTOR.
Robot_system::Robot_system(std::string val_id)
{   
    // initialisation process.
    robot_id              = val_id;

    // initialisation microcontroler.
    LibSerial::SerialPort* _serial_port_controle_A;
    LibSerial::SerialPort* _serial_port_sensor_B;
    __serial_port_controle_A = &_serial_port_controle_A;
    __serial_port_sensor_B = &_serial_port_sensor_B;

    // initialisation all thread.
    thread_1_localisation = std::thread(&Robot_system::thread_LOCALISATION, this);
    thread_2_commande     = std::thread(&Robot_system::thread_COMMANDE, this);

    thread_1_localisation.join();
    thread_2_commande.join();
}

// FONCTION.
std::string Robot_system::get_id()
{
    return robot_id;
}

bool Robot_system::match_ping_pong(std::string ping, std::string pong)
{
    /*
        DESCRIPTOR      : this function will check if the pong message 
            is the expected message.
    */
   ping = ping + "\n";
   return ping.compare(pong) == 0;
}

LibSerial::SerialPort* Robot_system::get_available_port(const int debug_mode, const std::string& message, bool wait_option)
{
    /*
        DESCRIPTOR      : this function will found the available ports witch 
            pong the good message. 
        INPUT           :
        * debug_mode        > you can see more information if debug_mode = 1.
        * message           > message pong to get.
        * wait_option       > allow to wait 2000ms between open and write/read.
        OUTPUT          :
        * _                 > pointor to object SerialPort.
    */
    
    // Check all ttyACMX.
    for (int i=0; i<4; i++)
    {
        LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
        std::string name_port = "/dev/ttyACM" + std::__cxx11::to_string(i);
        bool is_openable = true;

        if(debug_mode==1) {std::cout << "pointeur:" << &serial_port <<"\n";}

        try{ serial_port->Open(name_port);}
        catch (LibSerial::OpenFailed ex)
        {
            if(debug_mode==1) {std::cout << "Failed to open SerialPort : " << name_port << std::endl;}
            is_openable = false;
        }
        catch (LibSerial::AlreadyOpen ex)
        {
            if(debug_mode==1) {std::cout << "SerialPort already open : " << name_port << std::endl;}
            is_openable = false;
        }

        if(is_openable)
        {
            if(wait_option){ usleep(2000000);}
            if(debug_mode==1) {std::cout << "Succes to open SerialPort : " << name_port << std::endl;}
            if(debug_mode==1) {std::cout << "message:" << message << "\n";}
            try{ serial_port->Write(message);}
            catch(std::runtime_error ex) { std::cout << "nop\n"; }

            std::string reponse;
            serial_port->ReadLine(reponse);
            if(debug_mode==1) {std::cout << "reponse:" << reponse;}

            if(match_ping_pong(message, reponse))
            {   
                std::string port_A_name;
                std::string port_B_name;
                // TODO: move this cheat code.
                if(match_ping_pong("1/A", reponse)){ port_A_name = name_port; }
                if(match_ping_pong("1/B", reponse)){ port_B_name = name_port; }
                // TODOEND.

                return serial_port;
            }
            else{serial_port->Close();}
        }
    }

    // Check all ttyUSBX.
    for (int i=0; i<4; i++)
    {
        LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
        std::string name_port = "/dev/ttyUSB" + std::__cxx11::to_string(i);
        bool is_openable = true;

        if(debug_mode==1) {std::cout << "pointeur:" << serial_port <<"\n";}

        try{ 
            serial_port->Open(name_port);
        }
        catch (LibSerial::OpenFailed ex)
        {
            if(debug_mode==1) {std::cout << "Failed to open SerialPort : " << name_port << std::endl;}
            is_openable = false;
        }
        catch (LibSerial::AlreadyOpen ex)
        {
            if(debug_mode==1) {std::cout << "SerialPort already open : " << name_port << std::endl;}
            is_openable = false;
        }

        if(is_openable)
        {
            if(wait_option){ usleep(2000000);}
            if(debug_mode==1) {std::cout << "Succes to open SerialPort : " << name_port << std::endl;}
            if(debug_mode==1) {std::cout << "message:" << message << "\n";}
            try{ serial_port->Write(message);}
            catch(std::runtime_error ex) { std::cout << "nop\n"; }

            std::string reponse;
            serial_port->ReadLine(reponse);
            if(debug_mode==1) {std::cout << "reponse:" << reponse;}

            if(match_ping_pong(message, reponse))
            {
                return serial_port;
            }
            else{serial_port->Close();}
        }
    }

    // If we found nothing
    return NULL;
}

// THREAD.
void Robot_system::thread_LOCALISATION()
{
    /*
        DESCRIPTION: this thread will compute the SLAM algorythme
            and get the position of robot on the current map.
    */

    while(true)
    {
        std::cout << "[THREAD-1] run localisation.\n";
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void Robot_system::thread_COMMANDE()
{
    /*
        DESCRIPTION: this thread will get all input data and user
            information to command all composant.
    */

    while(true)
    {
        std::cout << "[THREAD-2] run commande. \n";
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void Robot_system::thread_SPEAKER(LibSerial::SerialPort** serial_port, int& state, std::string pong_message)
{   
    /*
        DESCRIPTION: this thread will send ping message all 1000ms,
            it will also manage the deconnection and reconnection
            of microcontroler.
    */
    bool is_lost = false;
    std::chrono::high_resolution_clock::time_point timer_start, current_timer;
    std::chrono::duration<double, std::milli> time_span;
    int time_since_lost = 500;

    while(true)
    {   
        // send ping all 500ms.
        usleep(500000);
        std::string message = "1/X";

        if(*serial_port != NULL)
        {
            try{
                (**serial_port).Write(message);
                is_lost = false;
            }
            catch(LibSerial::NotOpen ex){std::cout << "notopen\n";}
            catch(std::runtime_error ex)
            {
                if(!is_lost)
                {
                    // we stard timer.
                    timer_start = std::chrono::high_resolution_clock::now();
                    is_lost = true;
                }

                // if lost for more than 500ms we close it.
                current_timer = std::chrono::high_resolution_clock::now();
                time_span = current_timer - timer_start;
                if((int)time_span.count() > time_since_lost)
                {
                    // close connection.
                    (**serial_port).Close();
                    *serial_port = NULL;
                }
            }
        }
        else{
            // we are disconnect.
            state = 2;

            // we try to found it.
            *serial_port = get_available_port(0, pong_message, true);
        }
    }
}

void Robot_system::thread_LISTENER(LibSerial::SerialPort** serial_port, int& state, std::string message_pong)
{
    //last_ping
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double, std::milli> time_span;

    //max time without listen
    int time_since_lost = 1000;                                  // in ms.

    while(true)
    {   
        std::string reponse;
        char stop = '\n';   
        const unsigned int msTimeout = 100;                      // wait 100ms before pass to next.

        if(*serial_port != NULL)
        {
            if((**serial_port).IsOpen())
            {
                try{(**serial_port).ReadLine(reponse, stop, msTimeout);}
                catch(std::runtime_error ex){;}

                // if(reponse.size() > 0)
                // {
                //     std::cout << "reponse:" << reponse << std::endl;
                // }
                
                if(match_ping_pong(message_pong, reponse))
                {
                    t1 = std::chrono::high_resolution_clock::now();
                }

                // Comparator.
                t2 = std::chrono::high_resolution_clock::now();
                time_span = t2 - t1;
                if((int)time_span.count() > time_since_lost)
                {
                    state = 3;
                }
                else{
                    state = 1;
                }
            }
        }
        else{
            // to evoid speed loop, wait 200ms.
            usleep(200000);
        }
    }
}
