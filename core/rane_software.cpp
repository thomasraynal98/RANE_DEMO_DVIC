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

LibSerial::SerialPort* get_available_port(const int debug_mode, const std::string& message)
{
    /*
        DESCRIPTOR      : this function will found the available ports witch 
            pong the good message. 
        INPUT           :
        * message           > message pong to get.
        OUTPUT          :
        * 
    */
    
    // Check all ttyACMX.
    for (int i=0; i<4; i++)
    {
        LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
        std::string name_port = "/dev/ttyACM" + std::__cxx11::to_string(i);
        bool is_openable = true;

        if(debug_mode==1) {std::cout << "pointeur:" << serial_port <<"\n";}

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
}

void thread_SPEAKER(LibSerial::SerialPort* serial_port)
{
    while(true)
    {   
        // Listen ping.
        usleep(1000000);
        std::string message = "1/X";
        if(serial_port->IsOpen())
        {
            try{serial_port->Write(message);}
            catch(std::runtime_error ex){;}
        }
    }
}

void thread_LISTENER(LibSerial::SerialPort* serial_port, int& state, std::string message_pong)
{
    //last_ping
    high_resolution_clock::time_point t1 = high_resolution_clock::now();

    //max time without listen
    int time_since_lost = 1500;                                  // in ms.

    while(true)
    {   
        std::string reponse;
        char stop = '\n';   
        const unsigned int msTimeout = 100;                      // wait 100ms before pass to next.

        if(serial_port->IsOpen())
        {
            try{serial_port->ReadLine(reponse, stop, msTimeout);}
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
            high_resolution_clock::time_point t2 = high_resolution_clock::now();
            duration<double, std::milli> time_span = t2 - t1;
            if((int)time_span.count() > time_since_lost)
            {
                state = 2;
            }
            else{
                state = 1;
            }
        }
        else{
            state = 2;
            serial_port = get_available_port(0, message_pong);
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
    high_resolution_clock::time_point t1 = high_resolution_clock::now();
    usleep(10000);
    high_resolution_clock::time_point t2 = high_resolution_clock::now();
    duration<double, std::milli> time_span = t2 - t1;
    std::cout << (int)time_span.count() << "\n";

    LibSerial::SerialPort* serial_port_controle_A;
    LibSerial::SerialPort* serial_port_sensor_B;

    int state_A_controler = 0; // 0=init, 1=online, 2=offline.
    int state_B_controler = 0; // 0=init, 1=online, 2=offline.

    std::string controler_A_pong= "1/A";
    std::string controler_B_pong= "1/B";
    
    serial_port_controle_A = get_available_port(1, controler_A_pong);
    serial_port_sensor_B   = get_available_port(1, controler_B_pong);

    auto thread1  = std::thread(&thread_LISTENER, std::ref(serial_port_controle_A), std::ref(state_A_controler), controler_A_pong); 
	auto thread2  = std::thread(&thread_SPEAKER , std::ref(serial_port_controle_A)); 
    auto thread1B = std::thread(&thread_LISTENER, std::ref(serial_port_sensor_B)  , std::ref(state_B_controler), controler_B_pong); 
	auto thread2B = std::thread(&thread_SPEAKER , std::ref(serial_port_sensor_B)); 
    auto thread3  = std::thread(&thread_ANALYSER, std::ref(state_A_controler)     , std::ref(state_B_controler));

	thread1.join();
    thread2.join();
    thread1B.join();
    thread2B.join();
    thread3.join();
    std::cout << "repere3\n";

    return 0;
}