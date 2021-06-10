#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <thread>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "../include/fonction.h"


#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std::chrono;
 
// int main ()
// {
 
//   high_resolution_clock::time_point t1 = high_resolution_clock::now();
 
//   std::cout << "printing out 1000 stars...\n";
//   for (int i=0; i<1000000000; ++i) std::cout << "*";
//   std::cout << std::endl;
//   // usleep(1000000);
//   high_resolution_clock::time_point t2 = high_resolution_clock::now();
 
//   duration<double, std::milli> time_span = t2 - t1;
 
//   std::cout << "It took me " << time_span.count() << " milliseconds.";
//   std::cout << std::endl;
 
//   return 0;
// }

bool check_if_ping_pong_work(std::string ping, std::string pong)
{
    /*
        DESCRIPTOR      : this function will check if the pong message 
            is the expected message.
    */
   ping = ping + "\n";
   return ping.compare(pong) == 0;
}

LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message, bool wait_option)
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

            if(check_if_ping_pong_work(message, reponse))
            {   
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

            if(check_if_ping_pong_work(message, reponse))
            {
                return serial_port;
            }
            else{serial_port->Close();}
        }
    }

    // If we found nothing
    return NULL;
}

void thread_SPEAKER(LibSerial::SerialPort** serial_port, int& state, std::string pong_message)
{   
    /*
        DESCRIPTION: this thread will send ping message all 1000ms,
            it will also manage the deconnection and reconnection
            of microcontroler.
    */
    bool is_lost = false;
    high_resolution_clock::time_point timer_start, current_timer;
    duration<double, std::milli> time_span;
    int time_since_lost = 1000;

    while(true)
    {   
        // send ping all 1000ms.
        usleep(1000000);
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
                    timer_start = high_resolution_clock::now();
                    is_lost = true;
                }

                // if lost for more than 1000ms we close it.
                current_timer = high_resolution_clock::now();
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

void thread_LISTENER(LibSerial::SerialPort** serial_port, int& state, std::string message_pong)
{
    //last_ping
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    high_resolution_clock::time_point t2;
    duration<double, std::milli> time_span;

    //max time without listen
    int time_since_lost = 2000;                                  // in ms.

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

                if(reponse.size() > 0)
                {
                    std::cout << "reponse:" << reponse << std::endl;
                }
                if(check_if_ping_pong_work(message_pong, reponse))
                {
                    t1 = high_resolution_clock::now();
                }

                // Comparator.
                t2 = high_resolution_clock::now();
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

using namespace cv;
void thread_ANALYSER(int& state_A_controler, int& state_B_controler)
{   
    // INIT WINDOWS.
    Mat image(100, 800, CV_8UC3, Scalar(255, 255, 255));
    Point p1(0, 50), p2(800, 50);
    Point p3(0, 0), p4(100, 0), p5(300,0), p6(300,300), p7(550,0), p8(550,300);
    line(image, p1, p2, Scalar(0, 0, 0), 2, LINE_8);
    line(image, p5, p6, Scalar(0, 0, 0), 2, LINE_8);
    line(image, p7, p8, Scalar(0, 0, 0), 2, LINE_8);
    cv::putText(image, //target image
            "UNIT SENSOR", //text
            cv::Point(10, 35), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);

    cv::putText(image, //target image
            "UNIT COMMAND", //text
            cv::Point(10, 85), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);

    while(true)
    {
        // std::cout << "THREAD ANALYSER" << "\n";
        // imshow("Display window", image);
        // waitKey(0);
        usleep(1000000);
        std::cout << "stateA:" << state_A_controler << " ,stateB:" << state_B_controler << "\n";
    }
}

int main(){

    LibSerial::SerialPort** __serial_port_controle_A;
    LibSerial::SerialPort* _serial_port_controle_A;
    __serial_port_controle_A = &_serial_port_controle_A;

    LibSerial::SerialPort** __serial_port_sensor_B;
    LibSerial::SerialPort* _serial_port_sensor_B;
    __serial_port_sensor_B = &_serial_port_sensor_B;


    int state_A_controler = 0; // 0=init, 1=connect, 2=disconnect, 3=mute.
    int state_B_controler = 0; // 0=init, 1=connect, 2=disconnect, 3=mute.

    std::string controler_A_pong= "1/A";
    std::string controler_B_pong= "1/B";
    
    *__serial_port_controle_A = get_available_port(0, controler_A_pong, false);
    *__serial_port_sensor_B   = get_available_port(0, controler_B_pong, false);

    auto thread1  = std::thread(&thread_LISTENER, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong); 
	auto thread2  = std::thread(&thread_SPEAKER , __serial_port_controle_A, controler_A_pong); 
    auto thread1B = std::thread(&thread_LISTENER, __serial_port_sensor_B  , std::ref(state_B_controler), controler_B_pong); 
	auto thread2B = std::thread(&thread_SPEAKER , __serial_port_sensor_B  , controler_B_pong); 
    auto thread3  = std::thread(&thread_ANALYSER, std::ref(state_A_controler)     , std::ref(state_B_controler));

	thread1.join();
    thread2.join();
    thread1B.join();
    thread2B.join();
    thread3.join();

    return 0;
}