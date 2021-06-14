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

#include "../include/robot_system.h"


// CONSTRUCTOR.
Robot_system::Robot_system(std::string val_id)
{   
    // initialisation process.
    robot_id                 = val_id;

    // initialisation microcontroler.
    LibSerial::SerialPort* _serial_port_controle_A;
    LibSerial::SerialPort* _serial_port_sensor_B;
    __serial_port_controle_A  = &_serial_port_controle_A;
    __serial_port_sensor_B    = &_serial_port_sensor_B;
    *__serial_port_controle_A = get_available_port(0, controler_A_pong, true);
    *__serial_port_sensor_B   = get_available_port(0, controler_B_pong, true);

    // initialisation all thread.
    thread_1_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_2_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_3_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_4_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_5_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_6_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_7_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_8_last_hz_update = std::chrono::high_resolution_clock::now();
    thread_9_last_hz_update = std::chrono::high_resolution_clock::now();

    thread_1_localisation     = std::thread(&Robot_system::thread_LOCALISATION  , this);
    thread_2_commande         = std::thread(&Robot_system::thread_COMMANDE      , this);
    thread_3_listener_MICROA  = std::thread(&Robot_system::thread_LISTENER      , this, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, "A"); 
    thread_4_speaker_MICROA   = std::thread(&Robot_system::thread_SPEAKER       , this, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, 2.0, "A"); 
    thread_5_listener_MICROB  = std::thread(&Robot_system::thread_LISTENER      , this,   __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, "B"); 
    thread_6_speaker_MICROB   = std::thread(&Robot_system::thread_SPEAKER       , this,   __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, 2.0, "B");
    thread_7_listener_SERVER  = std::thread(&Robot_system::thread_SERVER_LISTEN , this);
    thread_8_speaker_SERVER   = std::thread(&Robot_system::thread_SERVER_SPEAKER, this); 
    thread_9_thread_ANALYSER  = std::thread(&Robot_system::thread_ANALYSER      , this); 

    thread_1_localisation.join();
    thread_2_commande.join();
    thread_3_listener_MICROA.join();
    thread_4_speaker_MICROA.join();
    thread_5_listener_MICROB.join();
    thread_6_speaker_MICROB.join();
    thread_7_listener_SERVER.join();
    // thread_8_speaker_SERVER.join();
    thread_9_thread_ANALYSER.join();
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

    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span = x-last_loop_time;
        thread_1_hz = 1000/(double)time_span.count();
        thread_1_last_hz_update = std::chrono::high_resolution_clock::now();
        last_loop_time = std::chrono::high_resolution_clock::now();

        // std::cout << "[THREAD-1] run localisation. " << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

void Robot_system::thread_COMMANDE()
{
    /*
        DESCRIPTION: this thread will get all input data and user
            information to command all composant.
    */

    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span = x-last_loop_time;
        thread_2_hz = 1000/(double)time_span.count();
        thread_2_last_hz_update = std::chrono::high_resolution_clock::now();
        last_loop_time = std::chrono::high_resolution_clock::now();

        // std::cout << "[THREAD-2] run commande. \n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Robot_system::thread_SPEAKER(LibSerial::SerialPort** serial_port, int& state, std::string pong_message, double ping_frequence, std::string micro_name)
{   
    /*
        DESCRIPTION: this thread will send ping message all XX00ms,
            it will also manage the deconnection and reconnection
            of microcontroler.
        INPUT      : 
        * serial_port    > the object with serial connection.
        * state          > the state of robot.
                         >> 0 = INIT.
                         >> 1 = CONNECT.
                         >> 2 = DISCONNECT.
                         >> 3 = MUTE.
        * pong_message   > the answer of microcontroler after a ping message.
        * ping_frequence > the frequence of ping.
        * micro_name     > the name of the microcontroler (A) ou (B).
    */

    int sleep_time       = (1/ping_frequence) * 1000000;                          // time wait until ping.
    int time_since_lost  = 500;                                                  // time to declare the port lost and close.
    bool is_lost         = false;

    std::chrono::high_resolution_clock::time_point timer_start, current_timer;
    std::chrono::duration<double, std::milli> time_span;

    std::string message = "1/X";

    // ANALYSE STATS.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span2;

    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span2 = x-last_loop_time;
        if(micro_name == "A"){
            thread_4_hz = 1000/(double)time_span2.count();
            thread_4_last_hz_update = std::chrono::high_resolution_clock::now();
        }
        if(micro_name == "B"){
            thread_6_hz = 1000/(double)time_span2.count();
            thread_6_last_hz_update = std::chrono::high_resolution_clock::now();
        }
        last_loop_time = std::chrono::high_resolution_clock::now();

        // send ping all 500ms.
        usleep(sleep_time);

        if(*serial_port != NULL)
        {
            try{
                (**serial_port).Write(message);
                is_lost = false;
            }
            catch(LibSerial::NotOpen ex){std::cout << "Port " << pong_message << " not open.\n";}
            catch(std::runtime_error ex)
            {
                if(!is_lost)
                {
                    // we stard timer.
                    timer_start = std::chrono::high_resolution_clock::now();
                    is_lost = true;
                }

                // if lost for more than XX00ms we close it.
                current_timer = std::chrono::high_resolution_clock::now();
                time_span = current_timer - timer_start;
                if((int)time_span.count() > time_since_lost)
                {
                    // close connection.
                    (**serial_port).Close();
                    *serial_port = NULL;
                    state = 2;
                }
            }
        }
        else{
            // we are disconnect.
            state = 2;
            // we try to found it.
            *serial_port = get_available_port(1, pong_message, true);
        }
    }
}

void Robot_system::thread_LISTENER(LibSerial::SerialPort** serial_port, int& state, std::string message_pong, std::string micro_name)
{
    //last_ping
    std::chrono::high_resolution_clock::time_point t1 = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point t2;
    std::chrono::duration<double, std::milli> time_span;

    //max time without listen
    int time_since_mute = 1000;                                  // in ms.
    int time_since_null = 200;                                   // in ms.

    // ANALYSE STATS.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span2;

    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span2 = x-last_loop_time;
        if(micro_name == "A"){
            thread_3_hz = 1000/(double)time_span2.count();
            thread_3_last_hz_update = std::chrono::high_resolution_clock::now();
        }
        if(micro_name == "B"){
            thread_5_hz = 1000/(double)time_span2.count();
            thread_5_last_hz_update = std::chrono::high_resolution_clock::now();
        }
        last_loop_time = std::chrono::high_resolution_clock::now();
        std::cout << micro_name << "\n";
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
                if((int)time_span.count() > time_since_mute)
                {
                    // STATE=MUTE.
                    state = 3;
                }
                else{
                    // STATE=CONNECT.
                    state = 1;
                }
            }
        }
        else{
            // to evoid speed loop, wait 200ms if serial_port is lost.
            usleep(time_since_null);
        }
    }
}

void Robot_system::thread_SERVER_LISTEN()
{
    /*
        DESCRIPTION: this thread will listen the server and the different order.
    */

    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span = x-last_loop_time;
        thread_7_hz = 1000/(double)time_span.count();
        thread_7_last_hz_update = std::chrono::high_resolution_clock::now();
        last_loop_time = std::chrono::high_resolution_clock::now();

        // std::cout << "[THREAD-7] server listen. \n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Robot_system::thread_SERVER_SPEAKER()
{
    /*
        DESCRIPTION: this thread will speak to the server about all sensor and
            data from the robot.
    */

    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    while(true)
    {   
        // TIMING VARIABLE.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span = x-last_loop_time;
        thread_8_hz = 1000/(double)time_span.count();
        thread_8_last_hz_update = std::chrono::high_resolution_clock::now();
        last_loop_time = std::chrono::high_resolution_clock::now();

        // std::cout << "[THREAD-8] server speaker. \n";
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void Robot_system::thread_ANALYSER()
{
    /*
        DESCRIPTION: this thread will analyse all system data, thread and show 
            some debug/analyst interface.
    */

    // VARIABLE.
    std::string state_A, state_B;
    std::string port_A_name_show, port_B_name_show;
    cv::Scalar fond_A( 255, 255, 255);
    cv::Scalar fond_B( 255, 255, 255);
    cv::Scalar fond_th1( 255, 255, 255);
    cv::Scalar fond_th2( 255, 255, 255);
    cv::Scalar fond_th3( 255, 255, 255);
    cv::Scalar fond_th4( 255, 255, 255);
    cv::Scalar fond_th5( 255, 255, 255);
    cv::Scalar fond_th6( 255, 255, 255);
    cv::Scalar fond_th7( 255, 255, 255);
    cv::Scalar fond_th8( 255, 255, 255);
    cv::Scalar fond_th9( 255, 255, 255);
    std::string th_state = "/";

    std::chrono::high_resolution_clock::time_point comparator_thread;
    std::chrono::duration<double, std::milli> time_thread;
    double frequence_th1, frequence_th2, frequence_th3, frequence_th4, frequence_th5, frequence_th6, frequence_th7, frequence_th8, frequence_th9;
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;

    // VISUAL FEATURE.
    cv::Mat image(1050, 1000, CV_8UC3, cv::Scalar(255, 255, 255));

    cv::putText(image, //target image
            "ROBOT SYSTEM ANALYTIQUE", //text
            cv::Point(10, 35), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            3);
    cv::putText(image, //target image
            "HARDWARE", //text
            cv::Point(10, 85), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);
    cv::putText(image, //target image
            "A - SENSOR SYSTEM", //text
            cv::Point(10, 135), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "B - COMMAND SYSTEM", //text
            cv::Point(10, 185), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "CAMERA", //text
            cv::Point(10, 235), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "PROXIMITY SENSOR", //text
            cv::Point(10, 285), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "VOLTAGE SENSOR", //text
            cv::Point(10, 335), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 
    cv::putText(image, //target image
            "NVIDIA HEAT", //text
            cv::Point(10, 385), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);      
    cv::putText(image, //target image
            "NVIDIA FAN", //text
            cv::Point(10, 435), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 
    cv::putText(image, //target image
            "SOFTWARE", //text
            cv::Point(10, 485), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2); 
    cv::putText(image, //target image
            "TH1 - LOCALISATION", //text
            cv::Point(10, 535), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);     
    cv::putText(image, //target image
            "TH2 - COMMANDE", //text
            cv::Point(10, 585), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);    
    cv::putText(image, //target image
            "TH3 - LISTEN MICRO A", //text
            cv::Point(10, 635), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH4 - SPEAKER MICRO A", //text
            cv::Point(10, 685), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);  
    cv::putText(image, //target image
            "TH5 - LISTEN MICRO B", //text
            cv::Point(10, 735), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH6 - SPEAKER MICRO B", //text
            cv::Point(10, 785), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 
    cv::putText(image, //target image
            "TH7 - LISTEN SERVER", //text
            cv::Point(10, 835), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH8 - SPEAKER SERVER", //text
            cv::Point(10, 885), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 
    cv::putText(image, //target image
            "TH9 - ANALYSE", //text
            cv::Point(10, 935), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "RESEAU", //text
            cv::Point(10, 985), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2); 
    cv::putText(image, //target image
            "INTERFACE CONNECTION", //text
            cv::Point(10, 1035), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 

    while(true)
    {
        cv::Mat affichage = image.clone();
        // MICRO CONTROLER SHOW.
        if(state_A_controler == 0){fond_A = (0, 0, 0); state_A = "init"; port_A_name_show = "/";}
        if(state_A_controler == 1){fond_A = cv::Scalar(0,255,0); state_A = "Connect"; port_A_name_show = port_A_name;}
        if(state_A_controler == 2){fond_A = cv::Scalar(0,0,255); state_A = "Disconnect"; port_A_name_show = "/";}
        if(state_A_controler == 3){fond_A = cv::Scalar(255,0,255); state_A = "Mute"; port_A_name_show = port_A_name;}

        if(state_B_controler == 0){fond_B = (0, 0, 0); state_B= "init"; port_B_name_show = "/";}
        if(state_B_controler == 1){fond_B = cv::Scalar(0,255,0); state_B = "Connect"; port_B_name_show = port_B_name;}
        if(state_B_controler == 2){fond_B = cv::Scalar(0,0,255); state_B = "Disconnect"; port_B_name_show = "/";}
        if(state_B_controler == 3){fond_B = cv::Scalar(255,0,255); state_B = "Mute"; port_B_name_show = port_B_name;}

        rectangle(affichage, cv::Point(750, 100), cv::Point(1000, 150),
            fond_A,
            -1, cv::LINE_8);

        rectangle(affichage, cv::Point(750, 150), cv::Point(1000, 200),
            fond_B,
            -1, cv::LINE_8);
        cv::putText(affichage, //target image
            state_A, //text
            cv::Point(760, 135), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);
        cv::putText(affichage, //target image
            state_B, //text
            cv::Point(760, 185), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);
        cv::putText(affichage, //target image
            port_A_name_show, //text
            cv::Point(460, 135), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);
        cv::putText(affichage, //target image
            port_B_name_show, //text
            cv::Point(460, 185), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);

        // THREAD VISUALISATION CHECKING.
        std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
        time_span = x-last_loop_time;
        thread_9_hz = 1000/(double)time_span.count();
        thread_9_last_hz_update = std::chrono::high_resolution_clock::now();
        last_loop_time = std::chrono::high_resolution_clock::now();

        time_span = x-thread_1_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th1 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th1 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 500), cv::Point(1000, 550),
            fond_th1,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_1_hz)+ " Hz", //text
            cv::Point(460, 535), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2); 
        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 535), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);  

        time_span = x-thread_2_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th2 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th2 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 550), cv::Point(1000, 600),
            fond_th2,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 585), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_2_hz)+ " Hz", //text
            cv::Point(460, 585), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);    
        
        time_span = x-thread_3_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th3 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th3 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 600), cv::Point(1000, 650),
            fond_th3,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 635), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_3_hz)+ " Hz", //text
            cv::Point(460, 635), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);  

        time_span = x-thread_4_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th4 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th4 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 650), cv::Point(1000, 700),
            fond_th4,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 685), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_4_hz)+ " Hz", //text
            cv::Point(460, 685), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);

        time_span = x-thread_5_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th5 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th5 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 700), cv::Point(1000, 750),
            fond_th5,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 735), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_5_hz)+ " Hz", //text
            cv::Point(460, 735), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);

        time_span = x-thread_6_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th6 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th6 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 750), cv::Point(1000, 800),
            fond_th6,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 785), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_6_hz)+ " Hz", //text
            cv::Point(460, 785), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);
        
        time_span = x-thread_7_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th7 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th7 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 800), cv::Point(1000, 850),
            fond_th7,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 835), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_7_hz)+ " Hz", //text
            cv::Point(460, 835), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);  
        
        time_span = x-thread_8_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th8 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th8 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 850), cv::Point(1000, 900),
            fond_th8,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 885), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
            std::__cxx11::to_string(thread_8_hz)+ " Hz", //text
            cv::Point(460, 885), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2); 

        time_span = x-thread_9_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th9 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th9 = cv::Scalar(0,255,0);
            th_state = "Running";
        }
        
        rectangle(affichage, cv::Point(750, 900), cv::Point(1000, 950),
            fond_th9,
        -1, cv::LINE_8);

        cv::putText(affichage, //target image
            th_state, //text
            cv::Point(760, 935), //top-left position
            0, //font
            1.0,
            CV_RGB(255, 255, 255), //font color
            2);   
        cv::putText(affichage, //target image
                std::__cxx11::to_string(thread_9_hz)+ " Hz", //text
                cv::Point(460, 935), //top-left position
                0, //font
                1.0,
                CV_RGB(0, 0, 0), //font color
                2);   

        cv::line(affichage, cv::Point(0, 50), cv::Point(1000, 50), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 100), cv::Point(1000, 100), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 150), cv::Point(1000, 150), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 200), cv::Point(1000, 200), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 250), cv::Point(1000, 250), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 300), cv::Point(1000, 300), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 350), cv::Point(1000, 350), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 400), cv::Point(1000, 400), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 450), cv::Point(1000, 450), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 500), cv::Point(1000, 500), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 550), cv::Point(1000, 550), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 600), cv::Point(1000, 600), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 650), cv::Point(1000, 650), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 700), cv::Point(1000, 700), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 750), cv::Point(1000, 750), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 800), cv::Point(1000, 800), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 850), cv::Point(1000, 850), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 900), cv::Point(1000, 900), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(0, 950), cv::Point(1000, 950), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
        cv::line(affichage, cv::Point(0,1000), cv::Point(1000,1000), cv::Scalar(0, 0, 0), 2, cv::LINE_8);

        cv::line(affichage, cv::Point(450, 100), cv::Point( 450, 450), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(450, 500), cv::Point( 450, 950), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(450,1000), cv::Point( 450,1050), cv::Scalar(0, 0, 0), 1, cv::LINE_8);

        cv::line(affichage, cv::Point(750, 100), cv::Point( 750, 450), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(750, 500), cv::Point( 750, 950), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        cv::line(affichage, cv::Point(750,1000), cv::Point( 750,1050), cv::Scalar(0, 0, 0), 1, cv::LINE_8);

        cv::imshow("Interface analyse vision.", affichage);
        char c=(char)cv::waitKey(25);
	    if(c==27)
	      break;
    }
}

