#define _USE_MATH_DEFINES
#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>
#include <fstream>
#include "math.h"
#include <array>
#include <cmath>
#include <chrono>
#include <cstring>
#include <iostream>
#include <queue>
#include <set>
#include <stack>
#include <tuple>
#include <utility>
#include <slamcore/slamcore.hpp>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <vector>
#include <thread>
#include <mutex>

#include <sio_client.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>

#include "CYdLidar.h"

#include "../include/robot_system.h"
#include "../include/fonction.h"
#include "../include/connection_listener.h"

// GLOBAL VARIABLE.
std::mutex _lock;
Robot_system* therobot;

// FONCTION COMMUNICATION.
bool Robot_system::update_yaml(std::string new_localisation, int new_map_id)
{
    /*
        DESCRIPTION: this function it's call when we get a new map.
    */

    /* Update navigation yaml. */
    cv::FileStorage fsSettings2(parametre.filePath.path_to_navigation_info, cv::FileStorage::WRITE);
    if(!fsSettings2.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }

    fsSettings2 << "Param_localisation" << new_localisation;
    fsSettings2 << "Param_id_map" << new_map_id;

    fsSettings2.release();
    return true;
}

bool Robot_system::update_map(std::string new_localisation, int new_map_id, std::string update_link_session, std::string update_link_png)
{
    /*
        DESCRIPTION: we call this function when the current stored map is not
            the good one, so we need to update all our file.
    */

    bool error = false;

    /* Clean Folder with navigation .session and .png before add new one. */
    std::string command_1 = "rm ../data_robot/Navigation/map.png";
    std::string command_2 = "rm ../data_robot/Navigation/map.session";
    if(system(command_1.c_str()) != 0){ error = true;};
    if(system(command_2.c_str()) != 0){ error = true;};

    std::cout << "[LIEN]" << update_link_session << " [LIEN_PNJ]" << update_link_png << "\n";
    /* Try to download new .session and .pnj from server. */
    std::string wget_session = "wget -P ../data_robot/Navigation/ ";
    wget_session += update_link_session;
    if(system(wget_session.c_str()) != 0){ error = true;}; 

    std::string wget_png = "wget -P ../data_robot/Navigation/ ";
    wget_png += update_link_png;
    if(system(wget_png.c_str()) != 0){ error = true;}; 

    /* Update parametre with this new value. */
    parametre.map.id_map        = new_map_id;
    parametre.map.localisation  = new_localisation;

    /* Update yaml. */
    update_yaml(parametre.map.localisation, parametre.map.id_map);

    /* Update robot variable. */
    if(!init_map()) {change_mode(Robot_state().warning); error = true;}

    return !error;
}

void Robot_system::check_map()
{
    /*
        DESCRIPTION: the main purpose of this map is to create and send a message
            that inform server what is the current map using by system.
    */

    sio::message::ptr test = sio::object_message::create();

    test->get_map()["localisation"] = sio::string_message::create(parametre.map.localisation);
    test->get_map()["map_id"]       = sio::int_message::create(parametre.map.id_map);
    
    current_socket->emit("check_map", test);
}

void Robot_system::bind_events()
{
    /*
        DESCRIPTION: this function store all kind of message that we can receive 
            from the main API.
    */

    /* If our current map is the good one. */
    current_socket->on("good", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock.lock();
        parametre.map.StoredMapIsGood = true;
        _lock.unlock();
    }));

    /* If our current map is not the good one, we need to get a new one. */
    current_socket->on("download", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock.lock();
        int new_id                      = data->get_map()["id"]->get_int();
        std::string new_localisation    = data->get_map()["localisation"]->get_string();
        std::string update_link_session = data->get_map()["link_session"]->get_string();
        std::string update_link_png     = data->get_map()["link_png"]->get_string();

        /* So update map. */
        if(!parametre.map.StoredMapIsGood)
        {
            if(update_map(new_localisation, new_id, update_link_session, update_link_png))
            {
                parametre.map.StoredMapIsGood   = true;
            }
        }
        _lock.unlock();
    }));

    /* In manual mode we need that robot do a precise command. */
    current_socket->on("command_to_do", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock.lock();
        change_mode(Robot_state().manual);
        robot_control.manual_commande_message = std::stoi(data->get_string());
        _lock.unlock();
    }));

    /* In autonav mode we need that robot reach a new point. */
    current_socket->on("position_to_reach", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        _lock.lock();

        destination_point.first  = data->get_map()["i"]->get_int();
        destination_point.second = data->get_map()["j"]->get_int();

        /* normal autonomous mode. */
        autonomous_nav_option = 0;
        change_mode(Robot_state().compute_nav);
        _lock.unlock();
    }));

    /* Ping pong from API. */
    current_socket->on("ping", sio::socket::event_listener_aux([&](std::string const& name, sio::message::ptr const& data, bool isAck, sio::message::list &ack_resp)
    {
        current_socket->emit("pong");
    }));
}

void Robot_system::send_data_to_server()
{
    /*
        DESCRIPTION: this function is call from thread_SERVER_SPEAKER and will send
            to the API all current state variable of robot. 
    */

    sio::message::ptr data_robot = sio::object_message::create();

    /* add position and orientation information. */
    data_robot->get_map()["pose_i"]          = sio::int_message::create(robot_position.pixel.i);
    data_robot->get_map()["pose_j"]          = sio::int_message::create(robot_position.pixel.j);
    data_robot->get_map()["pose_ti"]         = sio::int_message::create(robot_position.pixel.ti);
    data_robot->get_map()["pose_tj"]         = sio::int_message::create(robot_position.pixel.tj);
    data_robot->get_map()["pose_vi"]         = sio::int_message::create(robot_position.pixel.vi);
    data_robot->get_map()["pose_vj"]         = sio::int_message::create(robot_position.pixel.vj);

    /* add microcontroler information. */
    data_robot->get_map()["microA_state"]    = sio::int_message::create(state_A_controler);
    data_robot->get_map()["microB_state"]    = sio::int_message::create(state_B_controler);

    /* motor command. */ 
    data_robot->get_map()["motor_command"]   = sio::int_message::create(robot_control.manual_commande_message);

    /* current mode. */
    data_robot->get_map()["robot_state"]     = sio::string_message::create(robot_general_state);
    data_robot->get_map()["slam_state"]      = sio::int_message::create(robot_position.last_pose.state_slamcore_tracking);
    
    /* intern data. */
    data_robot->get_map()["cpu_heat"]        = sio::int_message::create(cpu_heat);
    data_robot->get_map()["cpu_load"]        = sio::int_message::create(cpu_load);
    data_robot->get_map()["fan_power"]       = sio::int_message::create(fan_power);

    /* Sensor information. */
    data_robot->get_map()["voltage"]         = sio::double_message::create(robot_sensor_data.energy.current);
    data_robot->get_map()["current"]         = sio::double_message::create(robot_sensor_data.energy.voltage);
    
    /* Speed data. */
    data_robot->get_map()["robot_speed"]     = sio::double_message::create(robot_position.position.robot_speed);

    /* Utilisation data. */
    data_robot->get_map()["robot_use_time"]  = sio::int_message::create((int)robot_timer.duration_tX.count());

    /* Lidar data. */
    data_robot->get_map()["lidar_data"]      = generate_data_lidar_for_server();

    /* send it. */
    current_socket->emit("global_data", data_robot);
}

void Robot_system::send_debug_data()
{
    /* 
        DESCRIPTION: this function will only be usefull in debug mode when we need
            to debug a lot of specific data.
    */

    sio::message::ptr data_debug_robot = sio::object_message::create();

    /* path and target keypoint. */
    data_debug_robot->get_map()["keypoints_path"] = generate_keypoint_vector_message();

    /* current target keypoint. */
    data_debug_robot->get_map()["target_keypoints_x"] = sio::int_message::create(target_keypoint->coordinate.first);
    data_debug_robot->get_map()["target_keypoints_y"] = sio::int_message::create(target_keypoint->coordinate.second);

    /* which process send command motor. */
    data_debug_robot->get_map()["origin_command"] = sio::int_message::create(robot_control.origin_commande);

    /* send it. */
    current_socket->emit("data_debug_robot", data_debug_robot);
}

sio::message::ptr Robot_system::generate_keypoint_vector_message()
{
    /*
        DESCRIPTION: this function will fill in the message from keypoint path.
    */

    sio::message::ptr array_to_transmit = sio::array_message::create();

    if(!keypoints_path.empty())
    {
        for(auto keypoint : keypoints_path)
        {
            array_to_transmit->get_vector().push_back(sio::int_message::create(keypoint.coordinate.first));
            array_to_transmit->get_vector().push_back(sio::int_message::create(keypoint.coordinate.second));
        }
    }

    return array_to_transmit;
}

sio::message::ptr Robot_system::generate_data_lidar_for_server()
{
    /*
        DESCRIPTION:
    */

    sio::message::ptr array_to_transmit = sio::array_message::create();
    double new_angle;

    if(!data_lidar_sample.empty())
    {
        for(auto sample : data_lidar_sample)
        {   
            if(sample.angle < -M_PI_2)
            {
                new_angle = sample.angle - M_PI_2;
            }
            if(sample.angle > M_PI_2)
            {
                new_angle = M_PI - ( -sample.angle) + M_PI_2;
            }
            array_to_transmit->get_vector().push_back(sio::double_message::create(new_angle));
            array_to_transmit->get_vector().push_back(sio::double_message::create(sample.value));
        }
    }

    return array_to_transmit;
}
 
// THREAD.
void Robot_system::thread_LOCALISATION(int frequency)
{
    /*
        DESCRIPTION: this thread will compute the SLAM algorythme
            and get the position of robot on the current map.
    */

    /* TIMING VARIABLE TO OPTIMISE FREQUENCY. */
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

    /* START SLAM. */
    while(true)
    {   
        /* TIMING VARIABLE. */
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_1_hz                = 1000/(double)time_span.count();
        thread_1_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)(time_of_loop));
        std::this_thread::sleep_until(next);
        /* END TIMING VARIABLE. */

        if(slam_process_state)
        {
            slamcore->start();

            while(slamcore->spinOnce())
            {   
                // If we are in autonav, update distance_RKP and target_angle.
                if(!possible_candidate_target_keypoint.empty() && robot_general_state == Robot_state().autonomous_nav)
                {
                    for(int i = 0; i < keypoints_path.size(); i++)
                    {
                        keypoints_path[i].distance_RKP = compute_distance_RPK(keypoints_path[i].coordinate)*0.05;
                        keypoints_path[i].target_angle = compute_target_angle(keypoints_path[i].coordinate);
                    }
                }
            }
        }
    }
}

void Robot_system::thread_COMMANDE(int frequency)
{
    /*
        DESCRIPTION: this thread will get all input data and user
            information to command all composant. It's the major
            and most important thread.
    */

    /* TIMING VARIABLE TO OPTIMISE FREQUENCY. */
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

    while(true)
    {   
        /* TIMING VARIABLE. */
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_2_hz                = 1000/(double)time_span.count();
        thread_2_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        /* END TIMING VARIABLE. */

        /* Statue print. */
        std::cout << "[ROBOT_STATE:" << robot_general_state << "] [LAST_COMMAND:"<< robot_control.manual_commande_message << \
        "] [POSITION:" << robot_position.position.x << "," << robot_position.position.y << "] \n";

        /* List of process to do before each action. */
        from_3DW_to_2DM();
        mode_checking();

        /* All mode of the robot. */
        if(robot_general_state == Robot_state().initialisation)
        {
            /*
                MODE DESCRIPTION:
                    This mode is automatocly activate when the robot start
                    and are checking for map.
            */

            /* Check if our current stored map is the good one, and if the tracking 
            is available. */
            if(parametre.map.StoredMapIsGood)
            {
                // change_mode(Robot_state().takeoff);
                change_mode(Robot_state().waiting);
            }
            else
            {
                check_map(); // Need verification from server.
            }
        }
        if(robot_general_state == Robot_state().waiting)
        {
            /*
                MODE DESCRIPTION:
                    This mode is activate when users don't send new information
                    but the initialisation process of robot is completed.
            */

            robot_control.manual_new_command(0, 3, -1);
        }
        if(robot_general_state == Robot_state().approach)
        {
            /*
                MODE DESCRIPTION:
                    This mode is activate when robot is charging, is a version 
                    of waiting mode.
            */
        }
        if(robot_general_state == Robot_state().compute_nav)
        {   
            /*
                MODE DESCRIPTION:
                    This mode is activate when users send a new destination_point,
                    or when the autonav system need to recompute new path because
                    there are an obstacle.
                    So compute a new path with A* path planning algorythme.
            */

            /* Create Pair object from our position. */
            Pair current_pose(robot_position.pixel.ti , robot_position.pixel.tj);

            /* Block the robot during this process. */
            robot_control.manual_new_command(0, 3, 1);
            secure_command_transmission();

            /* Compute the path. */
            if(aStarSearch(map_weighted, current_pose, destination_point, 0))
            {
                /* Succes so get target keypoint. */
                parametre.param_stKP.mode(0);
                select_target_keypoint();

                /* Check if we reach it. */
                cellIsReach();

                /* Change mode. */
                change_mode(Robot_state().autonomous_nav);
            }
            else
            {
                /* We get a problem on path planning process. */
                change_mode(Robot_state().warning);
            }
        }
        
        if(robot_general_state == Robot_state().autonomous_nav)
        {   
            /*
                MODE DESCRIPTION:
                    This mode is run after compute_nav mode if this one is successful
                    and a keypoints_path was computed. This goal is to found the current
                    target_keypoint, check slam command, check ultrason command, and 
                    choose witch command procision will be send to robot.
            */

            // TODO : when get data from API change autonomous_nav_option to 0. (not ici)
            
            /* Select current target keypoint and check if we reach them. */
            select_target_keypoint();
            cellIsReach();

            /* Lidar integration. */
            bool lidar_problem = false;
            lidar_problem = autonomous_nav_mode_lidar_integration();

            /* Check if destination keypoints is reach. 
            Else, compute motor commande. */
            if(destination_reach()) 
            { 
                robot_control.manual_new_command(0, 3, 1); 

                /* note: pas obliger de le faire pour le cas ou autonomous_nav_option
                vaut 0 car dans destination_reach() il passe automatiquement en mode
                waiting. */
                if(autonomous_nav_option == 1)
                {   
                    /* We reach the point d'approche. */
                    change_mode(Robot_state().approach);
                }
            }
            else
            { 
                if(lidar_problem)
                {
                    // compute_motor_autocommandeNico(1);
                }
                else
                {
                    // compute_motor_autocommandeNico(0);
                }
            }    

            /* Security check. */
            if(robot_sensor_data.lidar.dangerous_obstacle)
            {
                robot_control.manual_new_command(0, 3, 1);
            }   
        }
        if(robot_general_state == Robot_state().home)
        {
            /*
                MODE DESCRIPTION:
                    When this mode is call, we change the option of autonomous_nav
                    mode. So when autonomous_nav reaches the destination point
                    it switches directly to approach mode.
            */
        }
        if(robot_general_state == Robot_state().approach)
        {
            /*
                MODE DESCRIPTION:
                    This mode allow robot to dock on the docking pad for
                    charging process with the help of camera streaming
                    and QR code detection process. During this process, 
                    the slam navigation process from slamcore is not use.
            */
        }
        if(robot_general_state == Robot_state().manual)
        {
            /*
                MODE DESCRIPTION:
                    This mode allow user to manualy control the robot from the
                    API, in this thread we only change the robot_control
                    variable and this information is send in thread_SPEAKER().
            */

            /* Manual process. */
            manual_mode_process();

            /* Important function to call when you add new command. */
            secure_command_transmission();
        }
        if(robot_general_state == Robot_state().warning)
        {
            /*
                MODE DESCRIPTION:
                    This mode is automaticly activate when a problem is 
                    detected during robot process.
            */

           robot_control.manual_new_command(0, 3, 9);
        }
        if(robot_general_state == Robot_state().reset)
        {
            /*
                MODE DESCRIPTION:
                    This mode allow user to reset the robot.
            */

           robot_control.manual_new_command(0, 3, 10);
        }
        if(robot_general_state == Robot_state().lost)
        {
            /*
                MODE DESCRIPTION:
                    This mode is automatly call when slam process is lost.
            */
        }
        if(robot_general_state == Robot_state().takeoff)
        {
            /*
                MODE DESCRIPTION:
                    This special mode is call when robot it's charging on this
                    base. It allow robot to go backward.
            */
        }
        if(robot_general_state == Robot_state().charging)
        {
            /*
                DESCRIPTION: this mode it's call when robot reach is charging pad
                    and during this moment all system is in low consommation mode.
            */

            // TODO :
            std::cout << "[CHARGING]" << std::endl;
        }

        /* Transmit new command to microcontroler. */
        secure_command_transmission();
    }
}

void Robot_system::thread_SPEAKER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string pong_message, std::string micro_name)
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
    bool debug = true;

    // TIME VARIABLE
    int time_of_ping     = 500;                                                  // (2Hz) time wait until ping.
    int time_of_loop     = 1000/frequency;                                       // this is the real factor of Hz of this thread.
    int time_since_lost  = 800;                                                  // time to declare the port lost and close.
    bool is_lost         = false;

    std::chrono::high_resolution_clock::time_point timer_start, current_timer;
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();
    auto last_ping_time = std::chrono::high_resolution_clock::now();

    // MESSAGE.
    std::string message = "0/";

    // ANALYSE STATS.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span2;

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

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
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);

        /* Ping checking. */
        auto tc = std::chrono::high_resolution_clock::now();
        time_span = tc - last_ping_time;
        if((int)time_span.count() > time_of_ping)
        {
            if(*serial_port != NULL)
            {
                // std::cout << "[PING:" << message+micro_name << "]\n";
                try{
                    (**serial_port).Write(message+micro_name);
                    is_lost = false;
                    last_ping_time = std::chrono::high_resolution_clock::now();
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
            else
            {
                last_ping_time = std::chrono::high_resolution_clock::now();
                // we are disconnect.
                state = 2;
                // we try to found it.
                *serial_port = get_available_port(0, pong_message, true);
            }
        }

        if(*serial_port != NULL)
        {
            // for Microcontroler A.
            if(micro_name == "A" && !(robot_control.isTransmitA))
            {
                robot_control.compute_message_microA();

                try{
                    (**serial_port).Write(robot_control.message_microcontrolerA);
                }
                catch(LibSerial::NotOpen ex){std::cout << "Port " << pong_message << " not open.\n";}
                catch(std::runtime_error ex){}
                
                // std::cout << "[MESSAGE_MICROA_SEND:" << robot_control.message_microcontrolerA << "]\n";
            }
            // for Microcontroler B.
            if(micro_name == "B")
            {
                if(!(robot_control.isTransmitB))
                {
                    robot_control.compute_message_microB();

                    try{
                        (**serial_port).Write(robot_control.message_microcontrolerB);
                    }
                    catch(LibSerial::NotOpen ex){std::cout << "Port " << pong_message << " not open.\n";}
                    catch(std::runtime_error ex){}
                    
                    // std::cout << "[MESSAGE_MICROB_SEND:" << robot_control.message_microcontrolerB << "]\n";
                }

                /* Robot are asking for sensor information al frequency/2 cadense. */
                if(debug)
                {
                    debug = !debug;
                    try{
                        (**serial_port).Write("2/X");
                    }
                    catch(LibSerial::NotOpen ex){}
                    catch(std::runtime_error ex){}
                }
                else{
                    debug = !debug;
                }
            }
        }
    }
}

void Robot_system::thread_LISTENER(int frequency, LibSerial::SerialPort** serial_port, int& state, std::string message_pong, std::string micro_name)
{   
    // note: special use of frequency parameter in this case.

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

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

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
        
        std::string reponse;
        char stop = '\n';   
        const unsigned int msTimeout = 1000/frequency;                      // wait 100ms before pass to next.

        if(*serial_port != NULL)
        {
            if((**serial_port).IsOpen())
            {
                try{(**serial_port).ReadLine(reponse, stop, msTimeout);}
                catch(std::runtime_error ex){;}

                if(reponse.size() > 0)
                {
                    // read int de categorie.
                    const char delim = '/';
                    std::vector<std::string> data_brute;
                    tokenize(reponse, delim, data_brute);

                    std::string::size_type sz;     // alias of size_t
                    // int de categorie = 0 : ping message.

                    try
                    {           
                        if(std::stold(data_brute[0],&sz) == 0)
                        {
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
                        
                        // int de categorie = 1 : command pong message.
                        if(std::stold(data_brute[0],&sz) == 1)
                        {
                            if(match_ping_pong(robot_control.message_microcontrolerA, reponse) && micro_name == "A")
                            {
                                robot_control.isTransmitA = true;
                                robot_control_last_send  = robot_control;
                            }
                            if(match_ping_pong(robot_control.message_microcontrolerB, reponse) && micro_name == "B")
                            {
                                robot_control.isTransmitB = true;
                                robot_control_last_send.change_servo(robot_control);
                            }
                        }
                    }
                    catch(...)
                    {}                    
                }
            }
        }
        else{
            // to evoid speed loop, wait 200ms if serial_port is lost.
            usleep(time_since_null);
        }
    }
}

void Robot_system::thread_SERVER_LISTEN(int frequency)
{

    /*
        DESCRIPTION: this thread will listen the server and the different order.
    */

    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

    while(true)
    {   
        // TIMING VARIABLE.
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_7_hz                = 1000/(double)time_span.count();
        thread_7_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);

        // END TIMING VARIABLE.
        // std::cout << "[THREAD-7]\n";

        // TODO: NO EXPLICATION REQUIRED.
        // destination_point.first  = 96;
        // destination_point.second = 76;

        std::cout << "READ DESTINATION>";
        int rien;
        std::cin >> rien;
        destination_point.first  = 1032;
        destination_point.second = 793;
        change_mode(Robot_state().compute_nav);

        // std::cout << robot_general_state << std::endl;
        // robot_control.manual_commande_message = rien;
        // robot_general_state = Robot_state().manual;
    }

}

void Robot_system::thread_SERVER_SPEAKER(int frequency)
{

    /*
        DESCRIPTION: this thread will speak to the server about all sensor and
            data from the robot. Is
    */

    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

    signal(SIGINT, my_handler);
    signal(SIGQUIT, my_handler);

    while(true)
    {   
        // TIMING VARIABLE.
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_8_hz                = 1000/(double)time_span.count();
        thread_8_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        // END TIMING VARIABLE.

        // GET INTERNE VARIABLE BEFORE SEND.

        /* Update interne information data. */
        get_interne_data();

        /* Send data to server. */
        robot_timer.duration_tX = std::chrono::high_resolution_clock::now() - robot_timer.tp_X;
        send_data_to_server();
        //  send_debug_data();

        // std::cout << "[THREAD-8]\n";
    }
}

void Robot_system::thread_LIDAR(int frequency)
{
    /* DESCRIPTION:
        this thread allow the data recuperation of lidar sensor.
    */

    /* DESCRIPTION:
        this function will init all lidar variable.
        1. check one by one the port to detect lidar.
    */
    ydlidar::os_init();

    CYdLidar laser;

    int i = 0;
    // TODO : step 1.
    while(i<5)
    {

        //////////////////////string property/////////////////
        /// Lidar ports
        std::string port = "/dev/ttyUSB" + std::__cxx11::to_string(i);

        /// lidar port
        laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        /// ignore array
        std::string ignore_array;
        ignore_array.clear();
        laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                        ignore_array.size());

        //////////////////////int property/////////////////
        /// lidar baudrate
        int optval = 128000;
        laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
        /// tof lidar
        optval = TYPE_TRIANGLE;
        laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
        /// device type
        optval = YDLIDAR_TYPE_SERIAL;
        laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
        /// sample rate
        optval = 5;
        laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
        /// abnormal count
        optval = 4;
        laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

        //////////////////////bool property/////////////////
        /// fixed angle resolution
        bool b_optvalue = false;
        laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
        /// rotate 180
        laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
        /// Counterclockwise
        laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
        b_optvalue = true;
        laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
        /// one-way communication
        b_optvalue = false;
        laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
        /// intensity
        b_optvalue = false;
        laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
        /// Motor DTR
        b_optvalue = true;
        laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

        //////////////////////float property/////////////////
        /// unit: °
        float f_optvalue = 180.0f;
        laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = -180.0f;
        laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
        /// unit: m
        f_optvalue = 16.f;
        laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.1f;
        laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
        /// unit: Hz
        f_optvalue = 8.f;
        laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));


        // initialize SDK and LiDAR
        bool ret = laser.initialize();
        if (ret) {//success
            //Start the device scanning routine which runs on a separate thread and enable motor.
            ret = laser.turnOn();
        } else {
            fprintf(stderr, "%s\n", laser.DescribeError());
            fflush(stderr);
        }
        
        if(ret && ydlidar::os_isOk())
        {   
            // Memorise port name.
            robot_sensor_data.lidar.port  = port;
            robot_sensor_data.lidar.laser = &laser;
            robot_sensor_data.lidar.ret   = ret;
            robot_sensor_data.lidar.is_On = true;
            break;
        }
        else
        {
            // Stop the device scanning thread and disable motor.
            laser.turnOff();
            // Uninitialize the SDK and Disconnect the LiDAR.
            laser.disconnecting();
            // try an other port.
            i+=1;
        }
    }

    //
    while (robot_sensor_data.lidar.ret && ydlidar::os_isOk()) 
    {
        LaserScan scan;
        if (robot_sensor_data.lidar.laser->doProcessSimple(scan)) 
        {
            // fprintf(stdout, "Scan received[%llu]: %u ranges is [%f]Hz\n",
            //         scan.stamp,
            //         (unsigned int)scan.points.size(), 1.0 / scan.config.scan_time);
            // fflush(stdout);
            int pixel_x, pixel_y;

            data_lidar_sample.clear();
            list_points.clear();
            robot_sensor_data.lidar.dangerous_obstacle = false;
            for(auto point : scan.points)
            {
                if(abs(point.angle)>M_PI_2)
                {
                    Data_lidar observation;
                    observation.angle = -point.angle;
                    observation.value = point.range;

                    pixel_x = sin(-point.angle)*point.range*400/5+400;
                    pixel_y = cos(-point.angle)*point.range*400/5+400;

                    // detect if they are a obstacle just in front of robot.
                    if(robot_sensor_data.lidar.is_In_Security_zone(pixel_x, pixel_y))
                    {
                        robot_sensor_data.lidar.dangerous_obstacle = true;
                    }
                    
                    data_lidar_sample.push_back(observation);

                    Point_2D pt_observation;
                    pt_observation.i = pixel_x;
                    pt_observation.j = pixel_y;

                    list_points.push_back(pt_observation);
                    // std::cout << "[" << point.angle << "," << point.range << "\n"; 
                }
            }
        } 
        else 
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }
}

// CONSTRUCTOR FOR THE MAIN CLASS.
Robot_system::Robot_system()
{   
    /* STEP 1. Initialisation basic process. */
    change_mode(Robot_state().initialisation);
    if(!init_basic_data()){change_mode(Robot_state().warning);}
    robot_position.position.robot_speed = -1;
    cpu_heat                  = -1;
    cpu_load                  = -1;
    fan_power                 = -1;
    robot_timer.tp_X          = std::chrono::high_resolution_clock::now();
    therobot = this;

    /* STEP 2. Initialisation SLAM process. */
    if(!init_slam_sdk()) {change_mode(Robot_state().warning); slam_process_state = false;}
    else{ slam_process_state = true;}
    from_3DW_to_2DM();

    /* STEP 3. Initialisation navigation map from folder. */
    if(!init_map())      {change_mode(Robot_state().warning);}

    /* STEP 4. Initialisation debug interface. */
    if(true)
    {
        debug_init_debug_map(); // Initialisation of debug map.
        debug_init_sensor();    // Initialisation of debug sensor.
        debug_init_lidar();     // Initialisation of debug lidar.
    }

    /* STEP 5. Initialisation microcontroler communication. 
    mais cela ne va pas marcher car on attend le démarage de lidar.*/
    if(init_microcontroler() != 2){}// {change_mode(Robot_state().warning);}

    /* STEP 6. Initialisation connection server. */
    init_socketio();

    /* STEP 7. Initialisation all thread */
    init_thread_system();
}

Robot_state::Robot_state()
{}

// FONCTION MICROCONTROLER.
std::string Robot_system::get_id()
{
    return robot_id;
}

bool Robot_system::match_ping_pong(std::string ping, std::string pong)
{
    /*
        DESCRIPTOR      : this function will check if the pong message 
            is the expected message.
        "pong message"  = is the message received from microntroler if it
            are currently connected and working.
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
    
    // Check all ttyUSBX.
    if(robot_sensor_data.lidar.is_On)
    {
        for (int i=0; i<4; i++)
        {
            LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
            // LibSerial::SerialPort* serial_port2 = new LibSerial::SerialPort;
            std::string name_port = "/dev/ttyUSB" + std::__cxx11::to_string(i);
            bool is_openable = true;

            // TODO: dont open lidar port again.
            if(name_port.compare(robot_sensor_data.lidar.port) != 0)
            {
                if(debug_mode==1) {std::cout << "pointeur:" << serial_port <<"\n";}

                try{ serial_port->Open(name_port);}
                catch (LibSerial::AlreadyOpen ex)
                {
                    if(debug_mode==1) {std::cout << "SerialPort already open : " << name_port << std::endl;}
                    is_openable = false;
                }
                catch (LibSerial::OpenFailed ex)
                {
                    if(debug_mode==1) {std::cout << "Failed to open SerialPort : " << name_port << std::endl;}
                    is_openable = false;
                }

                if(is_openable)
                {
                    if(wait_option){ usleep(500000);}

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
                        if(match_ping_pong(controler_A_pong, reponse)){ port_A_name = name_port; }
                        if(match_ping_pong(controler_B_pong, reponse)){ port_B_name = name_port; }
                        // TODOEND.
                        std::cout << "[MATCH]" << serial_port << std::endl;
                        return serial_port;
                    }
                    else{serial_port->Close(); std::cout << "[CLOSE_PORT]" << std::endl;}
                }
            }
        }
    }

    // If we found nothing
    return NULL;
}

void Robot_system::get_interne_data()
{  
    std::string data = "-1"; 

    // // Update cpu load.
    std::ifstream ifile(path_to_cpu_load);
    ifile >> data;
    ifile.close();
    cpu_load = atof(data.c_str()) * 100;
    data = "-1";

    // Update cpu heat.
    std::ifstream ifile_B(path_to_cpu_heat);
    ifile_B >> data;
    ifile_B.close();
    cpu_heat = atof(data.c_str()) / 1000;

    if(cpu_heat == -1){state_sensor_cpu = 2;}
    else{state_sensor_cpu = 1;}

    data = "-1";

    // Update fan power.
    std::ifstream ifile_C(path_to_fan_power);
    ifile_C >> data;
    ifile_C.close();
    fan_power = std::stod(data);

    if(fan_power == -1){state_sensor_fan = 2;}
    else{state_sensor_fan = 1;}
}

// INITIALISATION FONCTION.
bool Robot_system::init_basic_data()
{
    /*
        DESCRIPTION: this function will read all information stored in 
            the folder data_robot and fullfil the system variable.
    */

    /* Read robot information. */
    cv::FileStorage fsSettings(parametre.filePath.path_to_identification, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings identification." << std::endl;
        return false;
    }
    fsSettings["Param_modele"] >> parametre.identity.modele;
    fsSettings["Param_version"] >> parametre.identity.version;
    fsSettings["Param_matricule"] >> parametre.identity.matricule;
    fsSettings["Param_exploitation"] >> parametre.identity.exploitation;
    fsSettings["Param_prenom"] >> parametre.identity.prenom;
    fsSettings.release();

    /* Read navigation map information. */
    cv::FileStorage fsSettings2(parametre.filePath.path_to_navigation_info, cv::FileStorage::READ);
    if(!fsSettings2.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }
    std::string tempo;
    fsSettings2["Param_localisation"] >> parametre.map.localisation;
    fsSettings2["Param_id_map"] >> parametre.map.id_map;
    fsSettings2.release();

    /* Read navigation parameter. */
    cv::FileStorage fsSettings3(parametre.filePath.path_to_navigation_param, cv::FileStorage::READ);
    if(!fsSettings3.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings navigation info." << std::endl;
        return false;
    }
    fsSettings3["Param_K"] >> parametre.paramNavigation.K;
    fsSettings3["Param_V"] >> parametre.paramNavigation.V;
    fsSettings3["Param_F"] >> parametre.paramNavigation.F;
    fsSettings3["Param_back_angle"] >> parametre.paramNavigation.back_angle;
    fsSettings3["Param_stall_pwm"] >> parametre.paramNavigation.stall_pwm;
    fsSettings3["Param_unstall_pwm"] >> parametre.paramNavigation.unstall_pwm;
    fsSettings3.release();

    return true;
}

bool Robot_system::init_slam_sdk()
try
{
    /*
        DESCRIPTION: this function will store all the procedure to start
            and run the slamcore sdk for localisation in previous session.
    */
    // ******************************************************************
    // Initialise SLAMcore API
    // ******************************************************************
    slamcore::slamcoreInit(
    slamcore::LogSeverity::Info, [](const slamcore::LogMessageInterface& message) {
      const time_t time = std::chrono::system_clock::to_time_t(message.getTimestamp());
      struct tm tm;
      localtime_r(&time, &tm);

      std::cerr << "[" << message.getSeverity() << " " << std::put_time(&tm, "%FT%T%z")
                << "] " << message.getMessage() << "\n";
    });
    // ******************************************************************
    // Create/Connect SLAM System
    // ******************************************************************
    slamcore::v0::SystemConfiguration sysCfg;
    std::unique_ptr<slamcore::SLAMSystemCallbackInterface> slam =
        slamcore::createSLAMSystem(sysCfg);
    if (!slam)
    {
        std::cerr << "Error creating SLAM system!" << std::endl;
        slamcore::slamcoreDeinit();
        return false;
    }

    std::cout << "Starting SLAM..." << std::endl;
    // ******************************************************************
    // Open the device
    // ******************************************************************
    slam->openWithSession(parametre.filePath.path_to_current_session.c_str());
    // ******************************************************************
    // Enable all the streams
    // ******************************************************************
    slam->setStreamEnabled(slamcore::Stream::Pose, true);
    slam->setStreamEnabled(slamcore::Stream::Video, true);
    // slam->setStreamEnabled(slamcore::Stream::MetaData, true);
    // *****************************************************************
    // Register callbacks!
    // *****************************************************************
    slam->registerCallback<slamcore::Stream::ErrorCode>(
    [](const slamcore::ErrorCodeInterface::CPtr& errorObj) {
        const auto rc = errorObj->getValue();
        std::cout << "Received: ErrorCode" << std::endl;
        std::cout << "\t" << rc.message() << " / " << rc.value() << " / "
                    << rc.category().name() << std::endl;
    });

    slam->registerCallback<slamcore::Stream::Video>(
    [&camera_data = camera_data](const slamcore::MultiFrameInterface::CPtr& multiFrameObj) {
        int i = 0;
        for (auto& img : *multiFrameObj)
        {
            i += 1;
            if(i == 2)
            {
                camera_data.width   = img.getWidth();
                camera_data.height  = img.getHeight();
                camera_data.qr_var.horizontal_center = camera_data.width/2;

                auto machin = img.getData();
                unsigned char* machin2 = (unsigned char*)machin;
                cv::Mat mat2(camera_data.height, camera_data.width, CV_8U, machin2);

                camera_data.detect_QR_code(mat2);
            }
        }
    });

    slam->registerCallback<slamcore::Stream::Pose>(
    [&robot_position = robot_position](const slamcore::PoseInterface<slamcore::camera_clock>::CPtr& poseObj) 
    {   
        /* compute speed of robot. */
        auto now = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> elapse = now - robot_position.position.last_time;
        robot_position.position.robot_speed = (sqrt(pow((robot_position.position.x - poseObj->getTranslation().x()), 2.0)
				+ pow((robot_position.position.y - poseObj->getTranslation().y()), 2.0)) / ((double)elapse.count()/1000)) * 3600 / 1000;

        /* update translation. */
        robot_position.position.x    = poseObj->getTranslation().x();
        robot_position.position.y    = poseObj->getTranslation().y();
        robot_position.position.z    = poseObj->getTranslation().z();
        robot_position.position.last_time = std::chrono::high_resolution_clock::now();

        /* update orientation. */
        robot_position.orientation.x = poseObj->getRotation().x();
        robot_position.orientation.y = poseObj->getRotation().y();
        robot_position.orientation.z = poseObj->getRotation().z();
        robot_position.orientation.w = poseObj->getRotation().w();
        from_quaternion_to_euler(robot_position);

        /* add transformation from camera to robot center. */
        robot_position.transformation.x = robot_position.position.x + sin(M_PI-robot_position.euler.y)*robot_position.transformation.cam_to_center;
        robot_position.transformation.y = robot_position.position.y + cos(M_PI-robot_position.euler.y)*robot_position.transformation.cam_to_center;
    
        /* add slam status. (not slam_process_state) but state_slamcore_tracking. */
        robot_position.update_slam_status();
        robot_position.detect_relocalisation();
    });

    // TODO: [BUG] resolve this problem.
    // slam->registerCallback<slamcore::Stream::MetaData>(
    // [&state_slamcore_tracking = state_slamcore_tracking](const slamcore::MetaDataInterface::CPtr& metaObj) 
    // {
    //     slamcore::MetaDataID ID = metaObj->getID();
    //     switch (ID)
    //     {
    //     case slamcore::MetaDataID::TrackingStatus:
    //     {
    //         metaObj->getValue(state_slamcore_tracking);
    //         std::cout << "STATUS : " << state_slamcore_tracking;
    //     }
    //     break;
    //     }
    // });

    // INIT ATTRIBU OBJECT.
    slamcore = std::move(slam);
    return true;
}
catch (const slamcore::slam_exception& ex)
{
  std::cerr << "system_error exception! " << ex.what() << " / " << ex.code().message()
            << " / " << ex.code().value() << std::endl;
  slamcore::slamcoreDeinit();
  return false;
}
catch (const std::exception& ex)
{
  std::cerr << "Uncaught std::exception! " << ex.what() << std::endl;
  slamcore::slamcoreDeinit();
  return false;
}
catch (...)
{
  std::cerr << "Uncaught unknown exception!" << std::endl;
  slamcore::slamcoreDeinit();
  return false;
}

bool Robot_system::init_map()
{
    /*
        DESCRIPTION: this function will init the map of the current session.
    */

    map_weighted = cv::imread(parametre.filePath.path_to_weighted_map, cv::IMREAD_GRAYSCALE);
    if(map_weighted.empty())
    {
        std::cout << "[ERROR] Could not read the image: " << parametre.filePath.path_to_weighted_map << std::endl;
        return false;
    }

    return true;
}

int Robot_system::init_microcontroler()
{
    /*
        DESCRIPTION: this function will init all system microcontroler.
        OUTPUT     :
        (-1)= Problem found.
        (0) = No microcontroler found.
        (1) = 1 microcontroler found.
        (2) = All microcontrolers found.
    */

    int output = -1;

    *__serial_port_sensor_B   = get_available_port(0, controler_B_pong, true);
    *__serial_port_controle_A = get_available_port(0, controler_A_pong, true);

    if(*__serial_port_controle_A != NULL)
    {
        if(output == -1) { output = 1;}
        else{ output += 1;}
    }
    if(*__serial_port_sensor_B != NULL)
    {
        if(output == -1) { output = 1;}
        else{ output += 1;}
    }

    return output;
}

void Robot_system::init_thread_system()
{
    /*
        DESCRIPTION: init all thread system.
    */

    /* Setup all timer. */
    thread_1_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_2_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_3_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_4_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_5_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_6_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_7_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_8_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_9_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_10_last_hz_update  = std::chrono::high_resolution_clock::now();

    /* Setup all thread. */
    thread_1_localisation     = std::thread(&Robot_system::thread_LOCALISATION  , this,  50);
    thread_2_commande         = std::thread(&Robot_system::thread_COMMANDE      , this, 100);
    thread_3_listener_MICROA  = std::thread(&Robot_system::thread_LISTENER      , this,  10, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, "A"); 
    thread_4_speaker_MICROA   = std::thread(&Robot_system::thread_SPEAKER       , this,  20, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, "A"); 
    thread_5_listener_MICROB  = std::thread(&Robot_system::thread_LISTENER      , this,  10,  __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, "B"); 
    thread_6_speaker_MICROB   = std::thread(&Robot_system::thread_SPEAKER       , this,  20,  __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, "B");
    thread_7_listener_SERVER  = std::thread(&Robot_system::thread_SERVER_LISTEN , this,  20);
    thread_8_speaker_SERVER   = std::thread(&Robot_system::thread_SERVER_SPEAKER, this,  10); 
    thread_9_thread_ANALYSER  = std::thread(&Robot_system::thread_ANALYSER      , this,  10); 
    thread_10_LIDAR           = std::thread(&Robot_system::thread_LIDAR         , this,  10); 

    /* Join all thread. */
    thread_1_localisation.join();
    thread_2_commande.join();
    thread_3_listener_MICROA.join();
    thread_4_speaker_MICROA.join();
    thread_5_listener_MICROB.join();
    thread_6_speaker_MICROB.join();
    thread_7_listener_SERVER.join();
    thread_8_speaker_SERVER.join();
    thread_9_thread_ANALYSER.join();
    thread_10_LIDAR.join();
}

void Robot_system::init_socketio()
{
    /*
        DESCRIPTION: this function will setup the socket io library and 
            connection from server API to robot.
    */


    connection_listener li(h);
    l = &li;

    h.set_open_listener (std::bind(&connection_listener::on_connected, l));
    h.set_close_listener(std::bind(&connection_listener::on_close    , l, std::placeholders::_1));
    h.set_fail_listener (std::bind(&connection_listener::on_fail     , l));
    h.connect("http://api-devo-docker.herokuapp.com/");
    // h.connect("http://127.0.0.1:5000");

    /* Stop if we are not connect. */
    _lock.lock();
    if(!l->get_connect_finish())
    {
        l->get_cond()->wait(_lock);
    }
    _lock.unlock();

    /* Inform server. */
    current_socket = h.socket();

    // TODO :
    // current_socket->emit("robot", parametre.identity.modele + parametre.identity.version);
    std::string envoie = "MK2R2_1";
    current_socket->emit("robot", envoie);
    
    /* Initialisation server listening. */
    bind_events();
}

// FONCTION NAVIGATION.
bool Robot_system::aStarSearch(cv::Mat grid, Pair& src, Pair& dest, int option_lidar)
{
	// If the source is out of range
	if (!isValid(grid, src)) {
		printf("Source is invalid\n");
		return false;
	}
    
	// If the destination is out of range
	if (!isValid(grid, dest)) {
		printf("Destination is invalid\n");
		return false;
	}

	// Either the source or the destination is blocked
	if (!isUnBlocked(grid, src)) {
		printf("Source is blocked\n");
		return false;
	}
	if (!isUnBlocked(grid, dest)) {
		printf("Dest is blocked\n");
		return false;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src, dest)) {
		printf("We are already at the destination\n");
		return false;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	bool closedList[grid.cols][grid.rows];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
    // constexpr auto p = static_cast<int>(grid.cols);
    // const int pp = grid.rows;
	// array<array<cell, COL>, ROW> cellDetails;
    int cols = grid.rows;
    int rows = grid.cols;
    
    std::vector<std::vector<cell>> cellDetails(rows, std::vector<cell>(cols));

	int i, j;
	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
    cellDetails[i][j].t = 0.0;
	cellDetails[i][j].parent = { i, j };

	/*
	Create an open list having information as-
	<f, <i, j>>
	where f = g + h,
	and i, j are the row and column index of that cell
	Note that 0 <= i <= ROW-1 & 0 <= j <= COL-1
	This open list is implenented as a set of tuple.*/
	std::priority_queue<Tuple, std::vector<Tuple>,std::greater<Tuple> >openList;


	// Put the starting cell on the open list and set its
	// 'f' as 0
	openList.emplace(0.0, i, j);


	// We set this boolean value as false as initially
	// the destination is not reached.
    // high_resolution_clock::time_point t1 = high_resolution_clock::now();
	while (!openList.empty()) {

		const Tuple& p = openList.top();
		// Add this vertex to the closed list
		i = std::get<1>(p); // second element of tupla
		j = std::get<2>(p); // third element of tupla
        Pair parent(i,j);
		// Remove this vertex from the open list
		openList.pop();
		closedList[i][j] = true;
		/*
				Generating all the 8 successor of this cell
						N.W N N.E
						\ | /
						\ | /
						W----Cell----E
								/ | \
						/ | \
						S.W S S.E

				Cell --> Popped Cell   ( i  , j  )
				N    --> North	       ( i-1, j  )
				S    --> South	       ( i+1, j  )
				E    --> East	       ( i  , j+1)
				W    --> West	       ( i  , j-1)
				N.E  --> North-East    ( i-1, j+1)
				N.W  --> North-West    ( i-1, j-1)
				S.E  --> South-East    ( i+1, j+1)
				S.W  --> South-West    ( i+1, j-1)
		*/
		for (int add_x = -1; add_x <= 1; add_x++) {
			for (int add_y = -1; add_y <= 1; add_y++) {
				Pair neighbour(i + add_x, j + add_y);
				// Only process this cell if this is a valid one.
				if (isValid(grid, neighbour)) {
					// If the destination cell is the same as the current successor.
					if (isDestination(neighbour, dest)) 
                    {   
                        // Set the Parent of the destination cell.
						cellDetails[neighbour.first][neighbour.second].parent = { i, j };

                        /* Process the complete path. */
                        std::stack<Pair> Path;
                        int row = dest.first;
                        int col = dest.second;
                        Pair next_node = cellDetails[row][col].parent;
                        do {
                            Path.push(next_node);
                            next_node = cellDetails[row][col].parent;
                            row = next_node.first;
                            col = next_node.second;
                        } while (cellDetails[row][col].parent != next_node);
                        
                        /* Transform the brut path to keypoint path. */

                        if(option_lidar == 0)
                        {from_global_path_to_keypoints_path(Path);}
                        if(option_lidar == 1)
                        {
                            //lidar.
                            select_PATKP(Path);
                        }

						return true;
					}
					// If the successor is already on the
					// closed list or if it is blocked, then
					// ignore it. Else do the following
					else if (!closedList[neighbour.first][neighbour.second] && isUnBlocked(grid, neighbour)) 
                    {
						double gNew, hNew, fNew, tNew;

                        /* Add weight to gNew if path is in proximity area. */

                        if((int)grid.at<uchar>(neighbour.second, neighbour.first) == 255)
						    {fNew = cellDetails[i][j].f + 1.0;}
                        if((int)grid.at<uchar>(neighbour.second, neighbour.first) == 200)
						    {fNew = cellDetails[i][j].f + 4.0;}

                        if(add_y != 0 && add_x !=0){fNew += 0.01;}
                        fNew = fNew + calculateHValue(neighbour, parent);
                        
                        /* Compute distance from source and destination. */
                        
						// hNew = calculateHValue(neighbour, dest);
                        // tNew = calculateHValue(neighbour, src);

                        // fNew = gNew + hNew + tNew;
						// If it isn’t on the open list, add
						// it to the open list. Make the
						// current square the parent of this
						// square. Record the f, g, and h
						// costs of the square cell
						//			 OR
						// If it is on the open list
						// already, check to see if this
						// path to that square is better,
						// using 'f' cost as the measure.
						if (cellDetails[neighbour.first][neighbour.second].f == 0 || cellDetails[neighbour.first][neighbour.second].f > fNew) 
                        {
                            hNew = calculateHValue(neighbour, dest);
                            tNew = 0;//calculateHValue(neighbour, src);
							openList.emplace(fNew + hNew + tNew, neighbour.first,neighbour.second);

							// Update the details of this
							// cell
							// cellDetails[neighbour.first][neighbour.second].g      = gNew;
							// cellDetails[neighbour.first][neighbour.second].h      = hNew;
							cellDetails[neighbour.first][neighbour.second].f      = fNew;
                            // cellDetails[neighbour.first][neighbour.second].t      = tNew;
							cellDetails[neighbour.first][neighbour.second].parent = { i, j };
						}
					}
				}
			}
		}
	}

	// When the destination cell is not found and the open
	// list is empty, then we conclude that we failed to
	// reach the destiantion cell. This may happen when the
	// there is no way to destination cell (due to
	// blockages)

	printf("Failed to find the Destination Cell\n");
    return false;
}

double Robot_system::calculateHValue(const Pair src, const Pair dest)
{
	// h is estimated with the two points distance formula
	return sqrt(pow((src.first - dest.first), 2.0)
				+ pow((src.second - dest.second), 2.0));
}

bool Robot_system::isDestination(const Pair& position, const Pair& dest)
{
	return position == dest;
}

bool Robot_system::isUnBlocked(cv::Mat grid, const Pair& point)
{
	// Returns true if the cell is not blocked else false
    // std::cout << ">> " << point.first << ", " << point.second << "\n";
    // std::cout << ">> " << int(grid.at<uchar>(point.second, point.first)) << "\n";
	return isValid(grid, point) && ((grid.at<uchar>(point.second, point.first) == 255) || (grid.at<uchar>(point.second,point.first) == 200));
}

bool Robot_system::isValid(cv::Mat grid, const Pair& point)
{ 
    // Returns true if row number and column number is in range.
    return (point.first >= 0) && (point.first < grid.size[1]) && (point.second >= 0) && (point.second < grid.size[0]);
}

void Robot_system::from_3DW_to_2DM()
{
    /*
        DESCRIPTION: this function will convert the 3D world pose into
            the 2D map pixel coordinate.
    */

    //DEBUG.
    robot_position.pixel.i = 85;
    robot_position.pixel.j = 200;

    //TODO: REMOVE THE DEBUG UP.
    robot_position.pixel.i = (int)((robot_position.position.x +  30.3)  / 0.05);
    robot_position.pixel.j = (int)((17.50 - robot_position.position.y)  / 0.05);

    // TRANSFORMATION
    robot_position.pixel.ti = (int)((robot_position.transformation.x +  30.3)  / 0.05);
    robot_position.pixel.tj = (int)((17.50 - robot_position.transformation.y)  / 0.05);
}

Pair Robot_system::from_3DW_to_2DM2(double x, double y)
{
    /*
        DESCRIPTION: this function will convert the 3D world pose into
            the 2D map pixel coordinate. But this version take x, y in param
            Param is in world coordinate system.
    */

    Pair point_pixel(0,0);
    point_pixel.first  = (int)((x +  30.3)  / 0.05);
    point_pixel.second = (int)((17.50 - y)  / 0.05);

    /* for server visualisation. */
    robot_position.pixel.vi           = point_pixel.first;
    robot_position.pixel.vj           = point_pixel.second;

    return point_pixel;
}

void Robot_system::from_global_path_to_keypoints_path(std::stack<Pair> Path)
{
    /*
        DESCRIPTION: the goal of this function is to take the brute global map
            from A* algorythme, and create a usefull list of keypoint and add
            some information for navigation process like distance_KPD, validation_angle
            isTryAvoidArea and distance_validation.
    */

    // Transform stack into vector and compute distance from destination.
    std::vector<Pair> vector_global_path;
    std::vector<double> vector_distances_from_destination;
    bool isDestination = true;
    double the_distance_from_destination = 0;
    Pair previous_p = Path.top();
    while(!Path.empty())
    {
        Pair p = Path.top();
        Path.pop();

        the_distance_from_destination += calculateHValue(previous_p, p)* 0.05;

        vector_distances_from_destination.push_back(the_distance_from_destination); 
        vector_global_path.push_back(p);
        
        previous_p = p;
    }
    
    // Variable.
    bool first_keypoint = true;

    // Clear vector.
    keypoints_path.clear();

    // Select Keypoints and compute data.
    for(int i = 0; i < vector_global_path.size(); i++)
    {   
        Path_keypoint current_keypoint;

        // OK it's the start point.
        if(i == 0)
        {
            // fix variable.
            current_keypoint.coordinate          = vector_global_path[i];
            current_keypoint.distance_KPD        = vector_distances_from_destination[vector_distances_from_destination.size()-i];
            current_keypoint.isTryAvoidArea      = map_weighted.at<uchar>(current_keypoint.coordinate.first, current_keypoint.coordinate.second);
            current_keypoint.distance_validation = compute_distance_validation(current_keypoint);
            
            // non fix variable.
            current_keypoint.isReach             = true;
            current_keypoint.target_angle        = compute_target_angle(current_keypoint.coordinate);
            current_keypoint.distance_RKP        = compute_distance_RPK(current_keypoint.coordinate)*0.05;
        
            // push.
            keypoints_path.push_back(current_keypoint);
        }
        // Ok it's the last point.
        else if(i == vector_global_path.size()-1)
        {
            // fix variable.
            current_keypoint.coordinate          = vector_global_path[i];
            current_keypoint.distance_KPD        = 0;
            current_keypoint.isTryAvoidArea      = 200; // for validation.
            current_keypoint.distance_validation = compute_distance_validation(current_keypoint);
            
            // non fix variable.
            current_keypoint.isReach             = false;
            current_keypoint.target_angle        = compute_target_angle(current_keypoint.coordinate);
            current_keypoint.distance_RKP        = compute_distance_RPK(current_keypoint.coordinate)*0.05;
        
            // push.
            keypoints_path.push_back(current_keypoint);
        }
        // Ok it's other point.
        else
        {
            if(calculateHValue(keypoints_path.back().coordinate, vector_global_path[i])*0.05 >= distance_between_keypoint)
            {
                // fix variable.
                current_keypoint.coordinate          = vector_global_path[i];
                current_keypoint.distance_KPD        = vector_distances_from_destination[vector_distances_from_destination.size()-i];
                current_keypoint.isTryAvoidArea      = map_weighted.at<uchar>(current_keypoint.coordinate.first, current_keypoint.coordinate.second);
                current_keypoint.distance_validation = compute_distance_validation(current_keypoint);
                
                // non fix variable.
                current_keypoint.isReach             = false;
                current_keypoint.target_angle        = compute_target_angle(current_keypoint.coordinate);
                current_keypoint.distance_RKP        = compute_distance_RPK(current_keypoint.coordinate)*0.05;

                // push.
                keypoints_path.push_back(current_keypoint);
            }
        }
    }

    // compile the validation_angle when all the path is know.
    for(int i = 0; i < keypoints_path.size(); i++)
    {
        if(i == 0){keypoints_path[i].validation_angle = 0;}
        if(i == keypoints_path.size()-1){keypoints_path[i].validation_angle = 180;}
        if(i > 0 && i < keypoints_path.size()-1) {keypoints_path[i].validation_angle = compute_validation_angle( keypoints_path[i-1].coordinate, \
                                                                                                        keypoints_path[i].coordinate, \      
                                                                                                        keypoints_path[i+1].coordinate);}
    }
}

double Robot_system::compute_distance_validation(Path_keypoint kp)
{
    /*
        DESCRIPTION: this function will compute and return the distance of validation.
            this distance means the minimun distance between robot and focus points
            to be considered like reach.
        INFORMATION: this distance take in consideration, the nature of the current 
            area (try_avoid or not) and the validation angle. 
        TODO       : take also in consideration the speed in futur.
        COMPUTE    : the value is between 0.05m in worst case where precision in needed
            and 0.40m in case that don't necessite precision.
    */

    double value = 0.05;
    if(!kp.isTryAvoidArea)          { value += 0.20; }
    if(kp.validation_angle <= 90/4) { value += 0.15; }
    return value;
}

double Robot_system::compute_target_angle(Pair kp)
{
    /*
        DESCRIPTION: this function will compute and return the angle between the 
            current orientation of robot and the angle of pose of robot and pose
            of path_keypoint pose.
        COMPUTE    : the value is between 0° deg and 180° deg only > 0.
    */

    double angle_RKP         = compute_vector_RKP(kp);
    double angle_ORIENTATION = robot_position.pixel.y_pixel;

    double distance_deg      = -1;
    
    if(angle_RKP >= angle_ORIENTATION)
    {
        distance_deg         = angle_RKP - angle_ORIENTATION;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}
    }
    else
    {
        distance_deg         = angle_ORIENTATION - angle_RKP;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}    
    }

    return distance_deg;
}

double Robot_system::compute_vector_RKP(const Pair& kp)
{
    /*
        DESCRIPTION: compute the angle in world map referenciel of vector 
            from robot to keypoint. (like North, West, East, South)
    */
    
    double x_sum = kp.first  - robot_position.pixel.ti;
    double y_sum = kp.second - robot_position.pixel.tj;

    double angle_degree = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    if(y_sum < 0)
    {
        angle_degree = (M_PI - angle_degree) + M_PI;
    }
    return angle_degree * (180/M_PI);
}

double Robot_system::compute_vector_RKP_2(const Pair& kpCurrent, const Pair& kp2)
{
    /*
        DESCRIPTION: same as version 1 but will take two points in param.
    */

    double x_sum = kp2.first  - kpCurrent.first;
    double y_sum = kp2.second - kpCurrent.second;

    double angle_degree = acos((x_sum)/(sqrt(pow(x_sum, 2.0) + pow(y_sum, 2.0))));
    if(y_sum < 0)
    {
        angle_degree = (M_PI - angle_degree) + M_PI;
    }
    return angle_degree * (180/M_PI);
}

double Robot_system::compute_distance_RPK(const Pair& kp)
{
    /*
        DESCRIPTION: return the distance between robot en kp.
    */
    return sqrt(pow((kp.first - robot_position.pixel.ti), 2.0)
            + pow((kp.second - robot_position.pixel.tj), 2.0));
}

double Robot_system::compute_validation_angle(const Pair& kpPrev, const Pair& kpCurrent, const Pair& kpNext)
{
    /*
        DESCRIPTION: this function will compute validation angle, it's an angle form from 3 points,
            the current one, the previously and the next one.
    */
    
    double angle_RPREV   = compute_vector_RKP_2(kpPrev, kpCurrent);
    double angle_RNEXT   = compute_vector_RKP_2(kpNext, kpCurrent);
    double distance_deg  = -1;

    if(angle_RPREV >= angle_RNEXT)
    {
        distance_deg         = angle_RPREV - angle_RNEXT;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}
    }
    else
    {
        distance_deg         = angle_RNEXT - angle_RPREV;
        if(distance_deg > 180){ distance_deg = 360 - distance_deg;}    
    }

    return 180 - distance_deg; 
}

void Robot_system::select_target_keypoint()
{
    /*
        DESCRIPTION: this fundamentale function will take all state data 
            en compte for select the better target keypoint between all
            path_keypoint in keypoints_path vector.
        HEURISTIC  : there are multiple variable and the heuristic need 
            to be fine tunned but this is the order of importence for 
            all variable
                > 1. (YES) distance_RKP
                > 2. (YES) target_angle
                > 3. (YES) isReach  
                > 4. (YES) distance_KPD
        PS         : that can be a good feature to integrate the neural 
            network in this process. Or to integrate a variable that say
            if there are an object between robot and kp.
    */

    /* PART 1. Get all pointor from keypoint vector that are in a range
    of threshold from robot. */
    double threshold = 3.0; //in meter.
    return_nearest_path_keypoint(threshold);

    /* PART 2. We need to normalise all variable so first we get the max
    value of all variable. */
    double max_distance_RKP = 0;
    double max_distance_KPD = 0;
    for(int i = 0; i < possible_candidate_target_keypoint.size(); i++)
    {
        if(possible_candidate_target_keypoint[i]->distance_RKP > max_distance_RKP)
        {
            max_distance_RKP = possible_candidate_target_keypoint[i]->distance_RKP;
        }
        if(possible_candidate_target_keypoint[i]->distance_KPD > max_distance_KPD)
        {
            max_distance_KPD = possible_candidate_target_keypoint[i]->distance_KPD;
        }
    }

    /* PART 3. Compute the target keypoint score of all this data. */
    std::vector<double> possible_candidate_target_keypoint_note;

    double weight_distance_RKP = parametre.param_stKP.weight_distance_RKP;
    double weight_target_angle = parametre.param_stKP.weight_target_angle;
    double weight_distance_KPD = parametre.param_stKP.weight_distance_KPD;
    double weight_isReach      = parametre.param_stKP.weight_isReach;

    for(int i = 0; i < possible_candidate_target_keypoint.size(); i++)
    {
        double candidate_note    = 0;
        double note_distance_RKP = 1 - (possible_candidate_target_keypoint[i]->distance_RKP/max_distance_RKP);
        double note_target_angle = 1 - (possible_candidate_target_keypoint[i]->target_angle/180);
        double note_distance_KPD = 1 - (possible_candidate_target_keypoint[i]->distance_RKP/max_distance_KPD);

        candidate_note = weight_distance_RKP*note_distance_RKP + weight_target_angle*note_target_angle + weight_distance_KPD*note_distance_KPD;
        candidate_note += weight_isReach*possible_candidate_target_keypoint[i]->isReach;

        possible_candidate_target_keypoint_note.push_back(candidate_note);
    }

    /* PART 4. Select the better one. */
    double max_note = 0;
    int index_candidate = 0;
    for(int i = 0; i < possible_candidate_target_keypoint_note.size(); i++)
    {
        if(max_note < possible_candidate_target_keypoint_note[i])
        {   
            max_note = possible_candidate_target_keypoint_note[i];
            index_candidate = i;
        }
    }

    /* PART 5. Init the current target_keypoint. */
    target_keypoint = possible_candidate_target_keypoint[index_candidate];
}

void Robot_system::return_nearest_path_keypoint(double threshold)
{
    /*
        DESCRIPTION: this function will run the keypoints_path vector
            and send back the pointer of keypoints in a distance of less
            then "threshold". If they are no point less far than the 
            threshold, we send back the pointer of the less far of all.
    */

    /* clean this vector. */
    possible_candidate_target_keypoint.clear();

    /* save the nearest kp in case of no kp is in threshold to avoid bug. */
    bool isEmpty               = true;
    Path_keypoint* nearest_kp  = NULL;
    double distance_nearest_kp = 9999;

    for(int i = 0; i < keypoints_path.size(); i++)
    {
        if(keypoints_path[i].distance_RKP < threshold) 
        { 
            isEmpty = false;
            possible_candidate_target_keypoint.push_back(&keypoints_path[i]);
        }
        else
        {
            if(keypoints_path[i].distance_RKP < distance_nearest_kp)
            {
                /* this one is the nearest outside threshold. */
                nearest_kp          = &keypoints_path[i];
                distance_nearest_kp = keypoints_path[i].distance_RKP;
            }
        }
    }

    if(isEmpty)
    {
        possible_candidate_target_keypoint.push_back(nearest_kp);
    }
}

bool Robot_system::cellIsReach()
{
    /*
        DESCRIPTION: this function will check if we reach the current
            target_keypoint. If we reach it, we change 'false' value
            to 'true' in the keypoints_path vector.
    */
    
    if(target_keypoint->distance_RKP < target_keypoint->distance_validation)
    {
        target_keypoint->isReach = true;
        return true;
    }
    return false;
}

bool Robot_system::destination_reach()
{
    /*
        DESCRIPTION: This function will check if the destination is reach and 
            clean all data using for this process for free memory.
    */

    if(keypoints_path[keypoints_path.size()-1].isReach)
    {
        change_mode(Robot_state().waiting);
        keypoints_path.clear();
        possible_candidate_target_keypoint.clear();
        return true;
    }

    return false;
}   

bool Robot_system::isInVect(std::vector<int> vector, int stuf)
{
    for(auto value : vector)
    {
        if(value == stuf) return true;
    }
    return false;
}

// FONCTION MODE.

void Robot_system::manual_mode_process()
{
    /*
        DESCRIPTION: this function is call in manual mode to transform
            user request to motor command.
    */

    if(robot_control.manual_commande_message == 0)  { robot_control.manual_new_command(0, 3, 0);}
    if(robot_control.manual_commande_message == 1)  { robot_control.manual_new_command(1, 3, 0);}
    if(robot_control.manual_commande_message == 2)  { robot_control.manual_new_command(2, 3, 0);}
    if(robot_control.manual_commande_message == 3)  { robot_control.manual_new_command(3, 1, 0);}
    if(robot_control.manual_commande_message == 4)  { robot_control.manual_new_command(4, 1, 0);}
    if(robot_control.manual_commande_message == 5)  { robot_control.manual_new_command(5, 2, 0);}
    if(robot_control.manual_commande_message == 6)  { robot_control.manual_new_command(6, 2, 0);}
    if(robot_control.manual_commande_message == 7)  { robot_control.manual_new_command(7, 3, 0);}
    if(robot_control.manual_commande_message == 8)  { robot_control.manual_new_command(8, 3, 0);}
}

void Robot_system::mode_checking()
{
    /*
        DESCRIPTION: this function it's call in thread_COMMANDE and will shake
            the slam status and change robot mode if it's necesary.
    */

    if(slam_process_state)
    {
        /* We are not lost, or we just recover. */
        if(robot_position.last_pose.state_slamcore_tracking == 1)
        {
            robot_general_state = robot_general_state_before_lost;
        }
        /* We are lost. */
        if((robot_position.last_pose.state_slamcore_tracking == 2 || \
        robot_position.last_pose.state_slamcore_tracking == 0) && \
        (robot_general_state != Robot_state().takeoff && \ 
        robot_general_state != Robot_state().approach && \
        robot_general_state != Robot_state().manual) && takeoff_begin)
        {
            change_mode(Robot_state().lost);
        }
    }
}

bool Robot_system::autonomous_nav_mode_lidar_integration()
{
    /* DESCRIPTION:
        this function is call in Thread_command just after the selection of target
        Keypoint with SLAM PROCESS. This function will integrate the lidar data to
        the navigation process.

        OUTPUT: return false if lidar don't detect problem.
    */

    /* Project Target Keypoint and next Keypoints on the lidar referencial. */
    std::vector<Point_2D> projected_keypoint = project_keypoint_in_lidar_referencial();

    /* Detect if this Keypoints are blocked by lidar data. */
    bool is_block = detect_path_obstruption(projected_keypoint);

    if(true)
    {
        /* __ If(yes) transform brut lidar data to local grid map with optional resolution. return to visualise debug. */
        cv::Mat current_lidar_grid = update_local_grid(projected_keypoint);

        /* __ If(yes) generate multi destination cell in local grip map. */
        std::vector<Pair> list_destination;
        list_destination = generate_multi_destination(projected_keypoint);

        // std::cout << "taille:" << list_destination.size() << " point(" << list_destination[0].first << "," << list_destination[0].second << ")\n";
        // cv::circle(current_lidar_grid, cv::Point((int)(list_destination[0].first),(int)(list_destination[0].second)),0, cv::Scalar(0,0,255), cv::FILLED, 0, 0);
        // cv::resize(current_lidar_grid, current_lidar_grid, cv::Size(0,0),16,16,cv::INTER_LINEAR);
        // cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
        // cv::imshow("test", current_lidar_grid);
        // char d=(char)cv::waitKey(25);

        /* __ If(yes) run A* on this local grid map. */
        generate_PATKP(list_destination, copy_local_grey_grid);

    //     // REMOVE:
    //     if(true)
    //     {   
    //         for(auto pt : projected_keypoint)
    //         {
    //             cv::circle(copy_local_grid, cv::Point((int)(pt.i),(int)(pt.j)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);
    //         }
    //         cv::resize(copy_local_grid, copy_local_grid, cv::Size(0,0),16,16,cv::INTER_LINEAR);
    //         cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
    //         cv::imshow("test", copy_local_grid);
    //         char d=(char)cv::waitKey(25);
    //     }
        
    //     return true;
    }
    return false;
}

void Robot_system::select_PATKP(std::stack<Pair> Path)
{
    /* DESCRIPTION:
        this function take in argument the path generate by lidar A* planning.
        and will generate a PATKP.
    */

    std::vector<Pair> vector_global_path;
    while(!Path.empty())
    {
        Pair p = Path.top();
        Path.pop();

        vector_global_path.push_back(p);

        // REMOVE: debug interface
        if(true)
        {
            // test.
            cv::circle(copy_local_grid, cv::Point((int)(p.first),(int)(p.second)),0, cv::Scalar(0,255,0), cv::FILLED, 0, 0);
        }
    }

    /* Now, choose a KP in this list and transform to PATKP. */
    // one cell = 20 cm.
    int number = 7; //1.50m

    Pair my_pt = vector_global_path[number];
    // my_pt.first = (int)(((my_pt.first*16)+((my_pt.first+1)*16))/2);
    // my_pt.second = (int)(((my_pt.second*16)+((my_pt.second+1)*16))/2);
    my_pt.first = (int)(((my_pt.first*16)));
    my_pt.second = (int)(((my_pt.second*16)));
    
    // REMOVE: debug interface
    if(true)
    {
        cv::circle(copy_local_grid, cv::Point((int)(my_pt.first/16),(int)(my_pt.second/16)),0, cv::Scalar(0,0,255), cv::FILLED, 0, 0);
        cv::resize(copy_local_grid, copy_local_grid, cv::Size(0,0),16,16,cv::INTER_LINEAR);
        cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
        cv::imshow("test", copy_local_grid);
        char d=(char)cv::waitKey(25);
    }

    transform_lidarRef_to_globalRef(my_pt);
}

void Robot_system::transform_lidarRef_to_globalRef(Pair point_lidarRef)
{
    /* DESCRIPTION:
        this function will take the PATKP and transform it to global ref
        for motor control descision. (plus vraiment)
    */

    int index_i = point_lidarRef.first - 400;
    int index_j = 400 - point_lidarRef.second;

    // division par zeros.
    if(index_i != 0) { angle_PATKP = atan(index_j/index_i);} //in rad (-PI to PI)
    else
    {
        index_i = 0.01;
        angle_PATKP = atan(index_j/index_i);
    }
}

void Robot_system::generate_PATKP(std::vector<Pair> list_destination, cv::Mat current_lidar_grid)
{
    /* DESCRIPTION:
        this function will generate A* on the local grid and select a Priority Alternative
        Target Keypoint.
    */

    if(list_destination.size() != 0)
    {
        Pair source;
        source.first  = 24;
        source.second = 24;

        Pair destination;
        destination.first  = list_destination[0].first;
        destination.second = list_destination[0].second;

        // but we need to transpose the RGB matrice to a Gray matrice.

        aStarSearch(current_lidar_grid, source, destination, 1);
    }
}

std::vector<Pair> Robot_system::generate_multi_destination(std::vector<Point_2D> projected_keypoint)
{
    /* DESCRIPTION: 
        this function generate 3 cases destinations for futurs A* in local grid.
    */

    double more_far_distance = 0;
    Pair destination_cell(-1,-1);
    std::vector<Pair> list_destination_cell;

    for(auto kp : projected_keypoint)
    {
        // the far one but visible on local grid.
        if(pow(pow(24-(kp.i/16),2)+pow(24-(kp.j/16),2),0.5) > more_far_distance && (kp.i >= 0 && kp.i < 800) && (kp.j >= 0 && kp.j < 400))
        {
            destination_cell.first  = kp.i;
            destination_cell.second = kp.j;
            more_far_distance = pow(pow(24-(kp.i/16),2)+pow(24-(kp.j/16),2),0.5);
        }
    }

    /* Now detect category : Front, Left, Right, Back, End */

    // end.
    if(destination_cell.first < 700 && destination_cell.first > 100 && destination_cell.second > 100 && destination_cell.second < 300)
    {
        Pair cell;
        cell.first    = (int)(destination_cell.first/16);
        cell.second   = (int)(destination_cell.second/16);
        list_destination_cell.push_back(cell);
    }

    // front.
    if(destination_cell.first > 0 && destination_cell.first < 800 && destination_cell.second > 0 && destination_cell.second < 100)
    {
        Pair cell;
        cell.first    = (int)(destination_cell.first/16);
        cell.second   = (int)(0);
        list_destination_cell.push_back(cell);
    }

    // back.
    if(destination_cell.first > 0 && destination_cell.first < 800 && destination_cell.second > 300 && destination_cell.second < 400)
    {
        Pair cell;
        cell.first    = (int)(destination_cell.first/16);
        cell.second   = (int)(24);
        list_destination_cell.push_back(cell);
    }

    // left.
    if(destination_cell.first > 0 && destination_cell.first < 100 && destination_cell.second > 0 && destination_cell.second < 400)
    {
        Pair cell;
        cell.first    = (int)(0);
        cell.second   = (int)(destination_cell.second/16);
        list_destination_cell.push_back(cell);
    }

    // right.
    if(destination_cell.first > 700 && destination_cell.first < 800 && destination_cell.second > 0 && destination_cell.second < 400)
    {
        Pair cell;
        cell.first    = (int)(49);
        cell.second   = (int)(destination_cell.second/16);
        list_destination_cell.push_back(cell);
    }

    //TODO:
    // create 2 more cell.
    // prevoir quand les destinations celle sont pleine

    return list_destination_cell;
}

cv::Mat Robot_system::update_local_grid(std::vector<Point_2D> projected_keypoint)
{
    /* DESCRIPTION:
        this function will update the local grid map for futur path planning process.
    */

    // for visual debug
    copy_local_grid = local_grid.clone();

    // for A*
    copy_local_grey_grid = local_grey_grid.clone();

    // REMOVE: debug
    // for(auto kp : projected_keypoint)
    // {
    //     cv::circle(copy_local_grid, cv::Point((int)(kp.i/16),(int)(kp.j/16)),0, cv::Scalar(0,255,0), cv::FILLED, 0, 0);
    // }
    // cv::circle(copy_local_grid, cv::Point((int)(projected_keypoint.back().i/16),(int)(projected_keypoint.back().j/16)),0, cv::Scalar(255,0,0), cv::FILLED, 0, 0);

    for(auto pt : list_points)
    {
        cv::circle(copy_local_grid, cv::Point((int)(pt.i/16),(int)(pt.j/16)),0, cv::Scalar(0,0,0), cv::FILLED, 0, 0);
        cv::circle(copy_local_grey_grid, cv::Point((int)(pt.i/16),(int)(pt.j/16)),1, cv::Scalar(0), cv::FILLED, 0, 0);
    }

    // test. REMOVE:
    // cv::resize(copy_local_grid, copy_local_grid, cv::Size(0,0),16,16,cv::INTER_LINEAR);
    // cv::namedWindow("test",cv::WINDOW_AUTOSIZE);
    // cv::imshow("test", copy_local_grid);
    // char d=(char)cv::waitKey(25);

    return copy_local_grid;
}

bool Robot_system::detect_path_obstruption(std::vector<Point_2D> projected_keypoint)
{
    /* DESCRIPTION:
        this function will detect if keypoint are blocked by lidar keypoint.
    */

    cv::Mat image_with_big_point = debug_lidar.clone();

    // lidar data.
    for(auto point : list_points)
    {
        cv::circle(image_with_big_point, cv::Point(point.i,point.j),10, cv::Scalar(0,0,0), cv::FILLED, 1, 0);
    } 

    for(auto kp : projected_keypoint)
    {   
        cv::Vec3b pixelColor = image_with_big_point.at<cv::Vec3b>(kp.j, kp.i);
        if(pixelColor.val[0] != 255 || pixelColor.val[1] != 255 || pixelColor.val[2] != 255)
        {
            return true;
        }
    }
 
    return false;
}

std::vector<Point_2D> Robot_system::project_keypoint_in_lidar_referencial()
{
    /* DESCRIPTION:
        this function will take target keypoint in 5 meters range and project then in lidar
        referencial.
    */

    /* Get keypoint in selection range. */
    double kp_selection_range = 5.0; //m
    std::vector<Path_keypoint*> keypoints_list_for_projection = get_kp_list(kp_selection_range);

    /* Project them in lidar referencial. The target KP to. */
    /* We need to change referencial angle. */
    return transform_angle_in_lidar_ref(keypoints_list_for_projection);
}

std::vector<Path_keypoint*> Robot_system::get_kp_list(double selection_range)
{
    /* DESCRIPTION:
        this function will select KP in selection range and return pointor of this list.
    */

    std::vector<Path_keypoint*> result;
    for(int i = 0; i < keypoints_path.size(); i++)
    {
        if(keypoints_path[i].distance_RKP <= selection_range)
        {
            result.push_back(&keypoints_path[i]);
        }
    }
    return result;
}

std::vector<Point_2D> Robot_system::transform_angle_in_lidar_ref(std::vector<Path_keypoint*> keypoints_list_for_projection)
{
    /* DESCRIPTION:
        this function will take a list of keypoint to project and transform the angle to be 
        project into the lidar ref grid.
    */

    /* need this 2 value to determine orientation of angle. */
    double angle_ORIENTATION = robot_position.pixel.y_pixel;
    double angle_RKP = 0;

    double transform_angle;

    /* project keypoints. */
    std::vector<Point_2D> projected_keypoint;
    for(int i = 0; i < keypoints_list_for_projection.size()+1; i++)
    {
        Point_2D projected_kp;

        /* put the target keypoint at the end of this list. */
        if(i == keypoints_list_for_projection.size())
        {
            Pair keypoint_format_pair(target_keypoint->coordinate.first, target_keypoint->coordinate.second);
            angle_RKP            = compute_vector_RKP(keypoint_format_pair);
        }
        else
        {
            Pair keypoint_format_pair(keypoints_list_for_projection[i]->coordinate.first, keypoints_list_for_projection[i]->coordinate.second);
            angle_RKP            = compute_vector_RKP(keypoint_format_pair);
        }
        
        if( i != keypoints_list_for_projection.size() )
        {
            if(angle_ORIENTATION <= angle_RKP)
            {
                if(angle_RKP - angle_ORIENTATION <= 180)
                {
                    // Right 
                    angle_RKP    = (180 - keypoints_list_for_projection[i]->target_angle);
                }
                else
                {
                    // Left 
                    angle_RKP    = -180 + keypoints_list_for_projection[i]->target_angle;
                }
            }
            else
            {
                if(angle_ORIENTATION - angle_RKP <= 180)
                {
                    // Left 
                    angle_RKP    = -180 + keypoints_list_for_projection[i]->target_angle;
                }
                else
                {
                    // Right 
                    angle_RKP    = (180 - keypoints_list_for_projection[i]->target_angle);
                }
            }
        }
        else
        {
            if(angle_ORIENTATION <= angle_RKP)
            {
                if(angle_RKP - angle_ORIENTATION <= 180)
                {
                    // Right 
                    angle_RKP    = 180 - target_keypoint->target_angle;
                }
                else
                {
                    // Left 
                    angle_RKP    = (-180) + target_keypoint->target_angle;
                }
            }
            else
            {
                if(angle_ORIENTATION - angle_RKP <= 180)
                {
                    // Left 
                    angle_RKP    = (-180) + target_keypoint->target_angle;
                }
                else
                {
                    // Right 
                    angle_RKP    = 180 - target_keypoint->target_angle;
                }
            }
        }

        if(i == keypoints_list_for_projection.size())
        {
            projected_kp.i = sin(angle_RKP*M_PI/180)*target_keypoint->distance_RKP*400/5+400;
            projected_kp.j = cos(angle_RKP*M_PI/180)*target_keypoint->distance_RKP*400/5+400;

            if(abs(target_keypoint->target_angle) <= 90)
            {
                projected_keypoint.push_back(projected_kp);
            }
        }
        if(i != keypoints_list_for_projection.size())
        {
            if(abs(keypoints_list_for_projection[i]->target_angle) <= 90)
            {
                projected_kp.i = sin(angle_RKP*M_PI/180)*keypoints_list_for_projection[i]->distance_RKP*400/5+400;
                projected_kp.j = cos(angle_RKP*M_PI/180)*keypoints_list_for_projection[i]->distance_RKP*400/5+400;
                projected_keypoint.push_back(projected_kp);
            }
        }
    }

    return projected_keypoint;
}

// FONCTION MOTOR.
void Robot_system::compute_motor_autocommande()
{
    /*
        DESCRIPTION: this function will compute the new commande for motor
            based on the angle with the target keypoint.
    */

    double angle_ORIENTATION = robot_position.pixel.y_pixel;
    double angle_RKP         = compute_vector_RKP(target_keypoint->coordinate);

    double distance_deg      = target_keypoint->target_angle;

    /* TODO : Reflechir à une maniere to integrate other variable in this calcule, 
    like speed or area type. */
    double dynamic_threshold = 10;
    
    /* target_angle variable is good but is it between 0 and 180 degres.
    We don't know if we need to go left or right so we recompute a version on target angle
    between -180 and 180.*/
    if(distance_deg > dynamic_threshold)
    {
        /* Can't go forward, need to rotate or be smooth. */
        if(angle_ORIENTATION <= angle_RKP)
        {
            if(angle_RKP - angle_ORIENTATION <= 180)
            {
                // Right rotation or smooth.
                if(distance_deg > 20) { robot_control.manual_new_command(4, 1, 1);}
                else { robot_control.manual_new_command(6, 2, 1);}

            }
            else
            {
                // Left rotation or smooth.
                if(distance_deg > 20) { robot_control.manual_new_command(3, 1, 1);}
                else { robot_control.manual_new_command(5, 2, 1);}
            }
        }
        else
        {
            if(angle_ORIENTATION - angle_RKP <= 180)
            {
                // Left rotation or smooth.
                if(distance_deg > 20) { robot_control.manual_new_command(3, 1, 1);}
                else { robot_control.manual_new_command(5, 2, 1);}
            }
            else
            {
                // Right rotation or smooth.
                if(distance_deg > 20) { robot_control.manual_new_command(4, 1, 1);}
                else { robot_control.manual_new_command(6, 2, 1);};
            }
        }
    }
    else
    {
        // Forward.
        robot_control.manual_new_command(1, 2, 1);
    }
}

void Robot_system::compute_motor_autocommandeNico(int option)
{
    /*
        DESCRIPTION: http://faculty.salina.k-state.edu/tim/robot_prog/MobileBot/Steering/pointFwd.html
    */
    double angle_ORIENTATION;
    double angle_RKP;

    if(option == 0)
    {
        angle_ORIENTATION = robot_position.pixel.y_pixel;
        angle_RKP         = compute_vector_RKP(target_keypoint->coordinate);
    }
    /* TODO : Reflechir à une maniere to integrate other variable in this calcule, 
    like speed or area type. */
    // TODO: influencer par distance de validation
    const double V = parametre.paramNavigation.V;// desired velocity, u can make V depend on distance to target to slow down when close
    const double K = parametre.paramNavigation.K;// turning gain [0.5:1]
    double F = parametre.paramNavigation.F; // influence of straight line component
    const int back_angle = parametre.paramNavigation.back_angle; // angle to consider that we are moving backwards in rad
    const int stall_pwm = parametre.paramNavigation.stall_pwm;
    const int unstall_pwm = parametre.paramNavigation.unstall_pwm;
    /* target_angle variable is good but is it between 0 and 180 degres.
    We don't know if we need to go left or right so we recompute a version on target angle
    between -180 and 180.*/
    double alpha;

    if(option == 0)
    {
        alpha = (angle_RKP - angle_ORIENTATION) * M_PI / 180; // difference in rad between robot angle and targetvector angle
        alpha += M_PI;
        alpha = atan2(sin(alpha), cos(alpha)); //[-PI:PI]
    }
    if(option == 1)
    {
        alpha = angle_PATKP;
    }
    // we don't have sensors behind so don't move backwards
    // if (abs(alpha)>back_angle){
    //     F = 0;
    // }
    int rightspeed = (int)(V*(F*cos(alpha)+K*sin(alpha)));
    int leftspeed = (int)(V*(F*cos(alpha)-K*sin(alpha)));
    //make sure the robot will move
    if(abs(rightspeed)<stall_pwm){
        rightspeed = unstall_pwm * rightspeed/abs(rightspeed);
    }
    if(abs(leftspeed)<stall_pwm){
        leftspeed = unstall_pwm * leftspeed/abs(leftspeed);
    }
    std::cout <<  angle_RKP << " " << angle_ORIENTATION << " " << alpha << "[LS:" << leftspeed << ", RS:" << rightspeed << "\n";
    // send to robot
    robot_control.manual_new_commandNico(leftspeed,rightspeed);
}

void Robot_system::secure_command_transmission()
{
    /*
        DESCRIPTION: This important function will insure the transmission of data
            verifying that the data was well received and understood by the different 
            microcontrollers.
    */

    if(!(robot_control == robot_control_last_send)) {robot_control.isTransmitA = false;}
    if(robot_control.isServo_different(robot_control_last_send)) {robot_control.isTransmitB = false;}
}

// FONCTION SYSTEM.

void Robot_system::change_mode(std::__cxx11::string& state)
{
    /*
        DESCRIPTION: this function will change the mode and update
            the robot_general_state_before_lost variable.
    */

    if(state != Robot_state().lost && state != Robot_state().takeoff)
    {
        robot_general_state_before_lost = state;
    }
    robot_general_state = state;
}

int Robot_system::test()
{
    return 42;
}

void Robot_system::my_handler(int i) {
    /*
        DESCRIPTION: this function is call when the user will use
            control-c event to hard exit the program. During this
            method, the program will send stop message to all 
            microcontroler of the robot to stop all activity.
    */

    printf("[CLEAN CONTROL-C EVENT EXIT]: signal is %d.\n", i);

    /* setup stop message for all controler. */
    therobot->robot_control.manual_new_command(0, 3, -1);
    therobot->robot_control.compute_message_microA();
    therobot->robot_control.compute_message_microB();
    therobot->robot_control.isTransmitA = false;
    therobot->robot_control.isTransmitB = false;

    /* first try to send it. */
    if((*(therobot->__serial_port_controle_A)) != NULL)
    {(**(therobot->__serial_port_controle_A)).Write(therobot->robot_control.message_microcontrolerA);}
    if((*(therobot->__serial_port_sensor_B)) != NULL)
    {(**(therobot->__serial_port_sensor_B)).Write(therobot->robot_control.message_microcontrolerB);}

    /* be shure than the controler get the information. */
    std::string reponseA, reponseB;
    char stop = '\n';   
    const unsigned int msTimeout = 100; 
    std::string::size_type sz;
    while(!therobot->robot_control.isTransmitA && !therobot->robot_control.isTransmitB)
    {
        usleep(100000); // wait 100ms.

        /* if not connected consider that is transmit. */
        if((*(therobot->__serial_port_controle_A)) == NULL){ therobot->robot_control.isTransmitA = true;}
        if((*(therobot->__serial_port_sensor_B)) == NULL){ therobot->robot_control.isTransmitB = true;}

        if((*(therobot->__serial_port_controle_A)) != NULL){(**(therobot->__serial_port_controle_A)).ReadLine(reponseA, stop, msTimeout);}
        if((*(therobot->__serial_port_sensor_B)) != NULL){(**(therobot->__serial_port_sensor_B)).ReadLine(reponseB, stop, msTimeout);}

        if(reponseA.size() > 0 && reponseB.size() > 0)
        {
            if(therobot->match_ping_pong(therobot->robot_control.message_microcontrolerA, reponseA))
            {
                therobot->robot_control.isTransmitA = true;
            }
            else { if((*(therobot->__serial_port_controle_A)) != NULL){(**(therobot->__serial_port_controle_A)).Write(therobot->robot_control.message_microcontrolerA);}}

            if(therobot->match_ping_pong(therobot->robot_control.message_microcontrolerB, reponseB))
            {
                therobot->robot_control.isTransmitB = true;
            }
            else { if((*(therobot->__serial_port_sensor_B)) != NULL){(**(therobot->__serial_port_sensor_B)).Write(therobot->robot_control.message_microcontrolerB);}}
        }
    }

    std::cout << "[CONTROL-C EVENT - ALL ACTIONNEUR SHUT DOWN.] \n Good night robot...";

    exit(i);
}

// THREAD ANALYSER DEBUG.
float Robot_system::round(float var)
{
    // 37.66666 * 100 =3766.66
    // 3766.66 + .5 =3767.16    for rounding off value
    // then type cast to int so value is 3767
    // then divided by 100 so the value converted into 37.67
    float value = (int)(var * 100 + .5);
    return (float)value / 100;
}

void Robot_system::add_texte(cv::Mat image)
{
    /*
        DESCRIPTION: add title on the image of interface.
    */
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
            "A - COMMAND SYSTEM", //text
            cv::Point(10, 135), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "B - SENSOR SYSTEM", //text
            cv::Point(10, 185), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);
    cv::putText(image, //target image
            "CAM & SLAMCORE", //text
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
            "NVIDIA HEAT/LOAD", //text
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
            "TH3 - MICRO A LISTEN", //text
            cv::Point(10, 635), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH4 - MICRO A SPEAKER", //text
            cv::Point(10, 685), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);  
    cv::putText(image, //target image
            "TH5 - MICRO B LISTEN", //text
            cv::Point(10, 735), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH6 - MICRO B SPEAKER", //text
            cv::Point(10, 785), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1); 
    cv::putText(image, //target image
            "TH7 - SERVER LISTEN", //text
            cv::Point(10, 835), //top-left position
            2, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            1);   
    cv::putText(image, //target image
            "TH8 - SERVER SPEAKER", //text
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
}

void Robot_system::add_intern_sensors(cv::Mat image)
{   
    // CPU HEAT.
    cv::Scalar color_police;

    if(cpu_heat < 45){color_police = cv::Scalar(0, 255, 0);}
    if(cpu_heat >= 45 && cpu_heat < 65){color_police = cv::Scalar(0, 255, 255);}
    if(cpu_heat >= 65){color_police = cv::Scalar(0, 0, 255);}
    if(cpu_heat == -1){color_police = cv::Scalar(200, 200, 200);}

    cv::putText(image, //target image
        cv::format("%.1f C", cpu_heat), //text
        cv::Point(460, 385), //top-left position
        0, //font
        1.0,
        color_police, //font color
        2); 
    
    // CPU LOAD.
    if(cpu_load < 30){color_police = cv::Scalar(0, 255, 0);}
    if(cpu_load >= 30 && cpu_load < 70){color_police = cv::Scalar(0, 255, 255);}
    if(cpu_load >= 70){color_police = cv::Scalar(0, 0, 255);}
    if(cpu_load == -1){color_police = cv::Scalar(200, 200, 200);}

    cv::putText(image, //target image
        cv::format("%.1f (%)", cpu_load), //text
        cv::Point(460+120, 385), //top-left position
        0, //font
        1.0,
        color_police, //font color
        2); 

    std::string state = "";
    if(state_sensor_cpu == 1){color_police = cv::Scalar(0, 255, 0); state = "Connect";}
    if(state_sensor_cpu == 2){color_police = cv::Scalar(0, 0, 255); state = "Disconnect";}
    rectangle(image, cv::Point(750, 350), cv::Point(1000, 400),
        color_police,
    -1, cv::LINE_8);
    cv::putText(image, //target image
        state, //text
        cv::Point(760, 385), //top-left position
        0, //font
        1.0,
        CV_RGB(255, 255, 255), //font color
        2); 
    state = "";
    
    // FAN POWER.
    if(fan_power < 30){color_police = cv::Scalar(0, 255, 0);}
    if(fan_power >= 30 && fan_power < 70){color_police = cv::Scalar(0, 255, 255);}
    if(fan_power >= 70){color_police = cv::Scalar(0, 0, 255);}
    if(fan_power == -1){color_police = cv::Scalar(200, 200, 200);}

    cv::putText(image, //target image
        cv::format("%.1f (%)", fan_power), //text
        cv::Point(460, 435), //top-left position
        0, //font
        1.0,
        color_police, //font color
        2); 

    if(state_sensor_fan == 1){color_police = cv::Scalar(0, 255, 0); state = "Connect";}
    if(state_sensor_fan == 2){color_police = cv::Scalar(0, 0, 255); state = "Disconnect";}
    rectangle(image, cv::Point(750, 400), cv::Point(1000, 450),
        color_police,
    -1, cv::LINE_8);
    cv::putText(image, //target image
        state, //text
        cv::Point(760, 435), //top-left position
        0, //font
        1.0,
        CV_RGB(255, 255, 255), //font color
        2); 
}

void Robot_system::add_state(cv::Mat image, int A, std::string th_state, double hz, cv::Scalar fond)
{  
    rectangle(image, cv::Point(750, A), cv::Point(1000, A+50),
            fond,
        -1, cv::LINE_8);

    cv::putText(image, //target image
        cv::format("%2.2f Hz", hz), //text
        cv::Point(460, A+35), //top-left position
        0, //font
        1.0,
        CV_RGB(0, 0, 0), //font color
        2); 
    cv::putText(image, //target image
        th_state, //text
        cv::Point(760, A+35), //top-left position
        0, //font
        1.0,
        CV_RGB(255, 255, 255), //font color
        2);  
}

void Robot_system::add_state_slamcore(cv::Mat image)
{   
    std::string string_tracking_status = "";
    cv::Scalar fond_tracking_status( 255, 255, 255);

    if(slam_process_state){
        string_tracking_status = "Working";
        fond_tracking_status = cv::Scalar(0,255,0);
    }
    if(!slam_process_state){
        string_tracking_status = "Not working";
        fond_tracking_status = cv::Scalar(0,0,255);
    }

    rectangle(image, cv::Point(750, 200), cv::Point(1000, 200+50),
            fond_tracking_status,
        -1, cv::LINE_8);

    cv::putText(image, //target image
        string_tracking_status, //text
        cv::Point(760, 200+35), //top-left position
        0, //font
        1.0,
        CV_RGB(255, 255, 255), //font color
        2);  

    cv::putText(image, //target image
        cv::format("%2.2f", robot_position.position.x), //text
        cv::Point(460, 200+35), //top-left position
        0, //font
        0.7,
        CV_RGB(0, 0, 0), //font color
        2); 
    cv::putText(image, //target image
        cv::format("%2.2f", robot_position.position.y), //text
        cv::Point(560, 200+35), //top-left position
        0, //font
        0.7,
        CV_RGB(0, 0, 0), //font color
        2); 
    cv::putText(image, //target image
        cv::format("%2.2f", robot_position.euler.y), //text
        cv::Point(660, 200+35), //top-left position
        0, //font
        0.7,
        CV_RGB(0, 0, 0), //font color
        2); 
}

void Robot_system::add_lines(cv::Mat image)
{
    cv::line(image, cv::Point(0, 50), cv::Point(1000, 50), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 100), cv::Point(1000, 100), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 150), cv::Point(1000, 150), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 200), cv::Point(1000, 200), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 250), cv::Point(1000, 250), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 300), cv::Point(1000, 300), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 350), cv::Point(1000, 350), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 400), cv::Point(1000, 400), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 450), cv::Point(1000, 450), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 500), cv::Point(1000, 500), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 550), cv::Point(1000, 550), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 600), cv::Point(1000, 600), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 650), cv::Point(1000, 650), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 700), cv::Point(1000, 700), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 750), cv::Point(1000, 750), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 800), cv::Point(1000, 800), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 850), cv::Point(1000, 850), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 900), cv::Point(1000, 900), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(0, 950), cv::Point(1000, 950), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0,1000), cv::Point(1000,1000), cv::Scalar(0, 0, 0), 2, cv::LINE_8);

    cv::line(image, cv::Point(450, 100), cv::Point( 450, 450), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(450, 500), cv::Point( 450, 950), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(450,1000), cv::Point( 450,1050), cv::Scalar(0, 0, 0), 1, cv::LINE_8);

    cv::line(image, cv::Point(750, 100), cv::Point( 750, 450), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(750, 500), cv::Point( 750, 950), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
    cv::line(image, cv::Point(750,1000), cv::Point( 750,1050), cv::Scalar(0, 0, 0), 1, cv::LINE_8);

}

void Robot_system::add_lines_sensor(cv::Mat image)
{
    /* for debug sensor. */

    cv::line(image, cv::Point(0, 150), cv::Point(600, 150), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 300), cv::Point(600, 300), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(150, 0), cv::Point(150, 150), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(300, 0), cv::Point(300, 150), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(450, 0), cv::Point(450, 150), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(200, 150), cv::Point(200, 300), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(400, 150), cv::Point(400, 300), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
}

void Robot_system::add_energy_sensor(cv::Mat image)
{
    /* Sensor debug.*/

    cv::putText(image, //target image
            cv::format("%2.2f V", robot_sensor_data.energy.voltage),
            cv::Point(100, 340), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);
    cv::putText(image, //target image
            cv::format("%2.2f A", robot_sensor_data.energy.current),
            cv::Point(100+300, 340), //top-left position
            0, //font
            1.0,
            CV_RGB(0, 0, 0), //font color
            2);
}

void Robot_system::debug_autonav(cv::Mat image)
{
    /*
        DESCRIPTION: this function will draw the autonav debug windows.
    */

    // add text.
    cv::putText(image, //target image
    "SLAM MODE", //text
    cv::Point(10, 35), //top-left position
    0, //font
    1.0,
    CV_RGB(0, 0, 0), //font color
    2);
    cv::putText(image, //target image
    "CORRIDOR MODE", //text
    cv::Point(10, 85), //top-left position
    0, //font
    1.0,
    CV_RGB(0, 0, 0), //font color
    2);
    cv::putText(image, //target image
    "LEFT WALL MODE", //text
    cv::Point(10, 135), //top-left position
    0, //font
    1.0,
    CV_RGB(0, 0, 0), //font color
    2);
    cv::putText(image, //target image
    "RIGHT WALL MODE", //text
    cv::Point(10, 185), //top-left position
    0, //font
    1.0,
    CV_RGB(0, 0, 0), //font color
    2);
    cv::putText(image, //target image
    "SECURITY STOP", //text
    cv::Point(10, 235), //top-left position
    0, //font
    1.0,
    CV_RGB(0, 0, 0), //font color
    2);

    // add data.
    auto on = CV_RGB(52, 201, 36);
    auto off = CV_RGB(255, 18, 79);
    auto dark = CV_RGB(0, 0, 0);

    if(robot_control.origin_commande == 1)
    {
        cv::putText(image, //target image
        "ON", //text
        cv::Point(310, 35), //top-left position
        0, //font
        1.0,
        on, //font color
        4);
    }
    else{
        cv::putText(image, //target image
        "OF", //text
        cv::Point(310, 35), //top-left position
        0, //font
        1.0,
        off, //font color
        4);
    }
    if(robot_control.origin_commande == 2)
    {
        cv::putText(image, //target image
        "ON", //text
        cv::Point(310, 85), //top-left position
        0, //font
        1.0,
        on, //font color
        4);
    }
    else{
        cv::putText(image, //target image
        "OFF", //text
        cv::Point(310, 85), //top-left position
        0, //font
        1.0,
        off, //font color
        4);
    }
    if(robot_control.origin_commande == 3)
    {
        cv::putText(image, //target image
        "ON", //text
        cv::Point(310, 135), //top-left position
        0, //font
        1.0,
        on, //font color
        4);
    }
    else{
        cv::putText(image, //target image
        "OFF", //text
        cv::Point(310, 135), //top-left position
        0, //font
        1.0,
        off, //font color
        4);
    }
    
    if(robot_control.origin_commande == 4)
    {
        cv::putText(image, //target image
        "ON", //text
        cv::Point(310, 185), //top-left position
        0, //font
        1.0,
        on, //font color
        4);
    }
    else{
        cv::putText(image, //target image
        "OFF", //text
        cv::Point(310, 185), //top-left position
        0, //font
        1.0,
        off, //font color
        4);
    }

    if(robot_control.origin_commande == 5)
    {
        cv::putText(image, //target image
        "ON", //text
        cv::Point(310, 235), //top-left position
        0, //font
        1.0,
        on, //font color
        4);
    }
    else{
        cv::putText(image, //target image
        "OFF", //text
        cv::Point(310, 235), //top-left position
        0, //font
        1.0,
        off, //font color
        4);
    }

    cv::putText(image, //target image
    std::to_string(robot_control.manual_commande_message), //text
    cv::Point(410, 35), //top-left position
    0, //font
    1.0,
    dark, //font color
    2);

    cv::putText(image, //target image
    cv::format("%2.2f", target_keypoint->target_angle), //text
    cv::Point(480, 35), //top-left position
    0, //font
    1.0,
    dark, //font color
    2);

    // add line.
    cv::line(image, cv::Point(0,  50), cv::Point(600,  50), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 100), cv::Point(600, 100), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 150), cv::Point(600, 150), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(0, 200), cv::Point(600, 200), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(300, 0), cv::Point(300, 250), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
    cv::line(image, cv::Point(400, 0), cv::Point(400, 250), cv::Scalar(0, 0, 0), 2, cv::LINE_8);
}

void Robot_system::thread_ANALYSER(int frequency)
{
    /*
        DESCRIPTION: this thread will analyse all system data, thread and show 
            some debug/analyst interface.
    */

    // TIME VARIABLE
    std::chrono::high_resolution_clock::time_point x = std::chrono::high_resolution_clock::now();
    auto next = std::chrono::high_resolution_clock::now();
    double time_of_loop = 1000/frequency;                   // en milliseconde.

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
    add_texte(image);

    // AUTONOMOUS MODE ULTRASON.
    cv::Mat init(250, 600, CV_8UC3, cv::Scalar(255, 255, 255));
    debug_autonomous_ultra = init;

    while(true)
    {
        cv::Mat affichage = image.clone();
        // MICRO CONTROLER SHOW.
        if(state_A_controler == 0){fond_A = (0, 0, 0); state_A = "Not initialised"; port_A_name_show = "/";}
        if(state_A_controler == 1){fond_A = cv::Scalar(0,255,0); state_A = "Connect"; port_A_name_show = port_A_name;}
        if(state_A_controler == 2){fond_A = cv::Scalar(0,0,255); state_A = "Disconnect"; port_A_name_show = "/";}
        if(state_A_controler == 3){fond_A = cv::Scalar(255,0,255); state_A = "Mute"; port_A_name_show = port_A_name;}

        if(state_B_controler == 0){fond_B = (0, 0, 0); state_B= "Not initialised"; port_B_name_show = "/";}
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

        // TIME VARIABLE.
        x                           = std::chrono::high_resolution_clock::now();
        time_span                   = x-last_loop_time;
        thread_9_hz                 = 1000/(double)time_span.count();
        thread_9_last_hz_update     = x;
        last_loop_time              = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);

        // THREAD VISUALISATION CHECKING.
        time_span = x-thread_1_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th1 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th1 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 500, th_state, thread_1_hz, fond_th1);

        time_span = x-thread_2_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th2 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th2 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 550, th_state, thread_2_hz, fond_th2);
        
        
        time_span = x-thread_3_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th3 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th3 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 600, th_state, thread_3_hz, fond_th3);
        
        time_span = x-thread_4_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th4 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th4 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 650, th_state, thread_4_hz, fond_th4);

        time_span = x-thread_5_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th5 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th5 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 700, th_state, thread_5_hz, fond_th5);

        time_span = x-thread_6_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th6 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th6 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 750, th_state, thread_6_hz, fond_th6);
        
        time_span = x-thread_7_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th7 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th7 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 800, th_state, thread_7_hz, fond_th7);
        
        time_span = x-thread_8_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th8 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th8 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        add_state(affichage, 850, th_state, thread_8_hz, fond_th8);

        time_span = x-thread_9_last_hz_update;
        if((int)time_span.count() > time_since_we_consider_thread_disconnect){
            fond_th9 = cv::Scalar(0,0,255);
            th_state = "Stop";
        } else{
            fond_th9 = cv::Scalar(0,255,0);
            th_state = "Running";
        }

        // ADD STATE SLAMCORE TRACKING.
        add_state_slamcore(affichage);

        add_intern_sensors(affichage);
        add_state(affichage, 900, th_state, thread_9_hz, fond_th9);
        add_lines(affichage);

        cv::imshow("Interface analyse vision.", affichage);
        // char c=(char)cv::waitKey(25);

        // FOR VISUAL MAP DEBUG.
        if(true)
        {
            cv::Mat copy_debug_visual_map = debug_visual_map.clone();

            debug_add_robot_pose(copy_debug_visual_map);
            debug_add_path_keypoint(copy_debug_visual_map);

            cv::resize(copy_debug_visual_map, copy_debug_visual_map, cv::Size(0,0),1.0,1.0,cv::INTER_LINEAR);//Same as resize(img, dst, Size(img.cols*1.5,img.rows*1.5),0,0,CV_INTER_LINEAR );

            cv::namedWindow("Debug visual map",cv::WINDOW_AUTOSIZE);
            cv::imshow("Debug visual map", copy_debug_visual_map);
        }    

        // FOR SENSOR DEBUG.
        if(true)
        {   
            cv::Mat copy_debug_sensor = debug_sensor.clone();
            add_energy_sensor(copy_debug_sensor);
            add_lines_sensor(copy_debug_sensor);
            cv::namedWindow("Debug sensor",cv::WINDOW_AUTOSIZE);
            cv::imshow("Debug sensor", copy_debug_sensor);
        }

        if(robot_general_state == Robot_state().autonomous_nav)
        {
            cv::Mat copy_debug_autonomous_ultra = debug_autonomous_ultra.clone();
            debug_autonav(copy_debug_autonomous_ultra);
            cv::namedWindow("Autonav integration",cv::WINDOW_AUTOSIZE);
            cv::imshow("Autonav integration", copy_debug_autonomous_ultra);
        }

        // LIDAR SHOW.
        if(true)
        {
            cv::Mat copy_interface_visuel = debug_lidar.clone();
            debug_lidar_interface(copy_interface_visuel);
            cv::namedWindow("lidar integration",cv::WINDOW_AUTOSIZE);
            cv::imshow("lidar integration", copy_interface_visuel);
        }

        char d=(char)cv::waitKey(25);
	    if(d==27)
	      break;
    }
}

// DEBUG FONCTION.
void Robot_system::debug_message_server()
{
    /*
        DESCRIPTION: the purpose of this function is only simulate the
            message from server in "string" format only for rapid 
            debug process.
        INFO       : this function will not be use at the end.
    */
    change_mode(Robot_state().waiting);
}

void Robot_system::debug_init_debug_map()
{
    /*
        DESCRIPTION: this function will init the visual map for debug,
            this map it's a rgb version of map_weighted and is purpose
            is to debug all navigation algorythme.
    */
    cv::cvtColor(map_weighted ,debug_visual_map, cv::COLOR_GRAY2RGB, 0);
}

void Robot_system::debug_add_robot_pose(cv::Mat copy_debug_visual_map)
{
    /*
        DESCRIPTION: this function will draw on the copy of debug_visual_map
            the position of the robot and its orientation.
    */

    // DRAW ROBOT POSE.
    cv::Point draw_pose = cv::Point(robot_position.pixel.i, robot_position.pixel.j);
    cv::circle( copy_debug_visual_map,
    draw_pose,
    2,
    cv::Scalar( 0, 0, 255 ),
    cv::FILLED,
    cv::LINE_8 );

    // TRANSFORMATION.
    cv::Point draw_pose3 = cv::Point(robot_position.pixel.ti, robot_position.pixel.tj);
    cv::circle( copy_debug_visual_map,
    draw_pose3,
    2,
    cv::Scalar( 255, 0, 0 ),
    cv::FILLED,
    cv::LINE_8 );

    // TODO: REMOVE DEBUG
    cv::Point draw_pose2 = cv::Point(96, 76);
    cv::circle( copy_debug_visual_map,
    draw_pose2,
    2,
    cv::Scalar( 255, 0, 255 ),
    cv::FILLED,
    cv::LINE_8 );

    // DRAW ROBOT ORIENTATION VECTOR.
    double distance_total_vector         = 0.50; 
    double x   = robot_position.position.x + (cos(robot_position.euler.y + M_PI/2) * distance_total_vector);
    double y   = robot_position.position.y + (sin(robot_position.euler.y + M_PI/2) * distance_total_vector);
    Pair point = from_3DW_to_2DM2(x, y);
    cv::line(copy_debug_visual_map, cv::Point(robot_position.pixel.i, robot_position.pixel.j), cv::Point(point.first, point.second), cv::Scalar(0, 0, 255), 1, cv::LINE_8);

    //  draw_pose = cv::Point(robot_position.transformation.FL_sensor_pose.first, robot_position.transformation.FL_sensor_pose.second);
    // cv::circle( copy_debug_visual_map,
    // draw_pose,
    // 1,
    // cv::Scalar( 0, 255, 255 ),
    // cv::FILLED,
    // cv::LINE_8 );

    // TODO : REMOVE INFORMATION ABOUT ANGLE.
    // cv::putText(copy_debug_visual_map, //target image
    //     cv::format("%2.2f", robot_position.pixel.y_pixel), //text
    //     cv::Point((int)point.first, (int)point.second), //top-left position
    //     0, //font
    //     0.4,
    //     CV_RGB(255, 0, 0), //font color
    //     1);

    // cv::putText(copy_debug_visual_map, //target image
    //     cv::format("%2.2f", compute_vector_RKP(destination_point)), //text
    //     cv::Point((int)destination_point.first, (int)destination_point.second), //top-left position
    //     0, //font
    //     0.4,
    //     CV_RGB(255, 0, 255), //font color
    //     1);
}

void Robot_system::debug_add_path_keypoint(cv::Mat copy_debug_visual_map)
{
    /*
        DESCRIPTION: draw all navigation information.
            1 > keypoints in gray.
            2 > keypoints candidate in bordeau + line RKP.
            3 > target keypoints in red + line RPK.
            4 > keypoints isReach in green.
    */

    if(keypoints_path.size() > 0 && robot_general_state == Robot_state().autonomous_nav)
    {
        // 1
        for(int i = 0; i < keypoints_path.size(); i++)
        {
            if(!keypoints_path[i].isReach)
            {
                cv::circle(copy_debug_visual_map, cv::Point(keypoints_path[i].coordinate.first, keypoints_path[i].coordinate.second),1, cv::Scalar(30,30,30), cv::FILLED, 1, 0);
            }
        }

        // 2 
        // TODO: remove
        for(int i = 0; i < possible_candidate_target_keypoint.size(); i++)
        {
            cv::circle(copy_debug_visual_map, cv::Point(possible_candidate_target_keypoint[i]->coordinate.first, possible_candidate_target_keypoint[i]->coordinate.second),1, cv::Scalar(19,0,76), cv::FILLED, 1,0);
            // cv::line(copy_debug_visual_map, cv::Point(possible_candidate_target_keypoint[i]->coordinate.first, possible_candidate_target_keypoint[i]->coordinate.second), cv::Point(robot_position.pixel.i, robot_position.pixel.j), cv::Scalar(19,0,76), 1, cv::LINE_8);
            // cv::putText(copy_debug_visual_map, //target image
            //     cv::format("%2.2f", possible_candidate_target_keypoint[i]->validation_angle), //text
            //     cv::Point((int)possible_candidate_target_keypoint[i]->coordinate.first, (int)possible_candidate_target_keypoint[i]->coordinate.second), //top-left position
            //     0, //font
            //     0.2,
            //     CV_RGB(255, 0, 0), //font color
            //     1);
        }

        // 4
        for(int i = 0; i < keypoints_path.size(); i++)
        {
            if(keypoints_path[i].isReach)
            {
                cv::circle(copy_debug_visual_map, cv::Point(keypoints_path[i].coordinate.first, keypoints_path[i].coordinate.second),1, cv::Scalar(0,255,0), cv::FILLED, 1, 0);
            }
        }

        // 3 : TODO : REMOVE TEXTE
        cv::circle(copy_debug_visual_map, cv::Point(target_keypoint->coordinate.first, target_keypoint->coordinate.second),2, cv::Scalar(255,0,0), cv::FILLED, 1,0);
        cv::line(copy_debug_visual_map, cv::Point(target_keypoint->coordinate.first, target_keypoint->coordinate.second), cv::Point(robot_position.pixel.ti, robot_position.pixel.tj), cv::Scalar(0, 0, 0), 1, cv::LINE_8);
        
        // cv::putText(copy_debug_visual_map, //target image
        // cv::format("%2.2f", target_keypoint->validation_angle), //text
        // cv::Point((int)target_keypoint->coordinate.first, (int)target_keypoint->coordinate.second), //top-left position
        // 0, //font
        // 0.4,
        // CV_RGB(255, 0, 0), //font color
        // 1);
    }
    else
    {
        // std::cout << "[WARNING] keypoints_path is empty or we are not in autonomous_nav mode." << std::endl;
    }
}

void Robot_system::debug_init_sensor()
{
    /*
        DESCRIPTION: this function will init the debug fonction for 
            all sensor of robot.
    */

    cv::Mat debug_sensor_init(350, 600, CV_8UC3, cv::Scalar(255, 255, 255));
    debug_sensor = debug_sensor_init;
}

void Robot_system::debug_init_lidar()
{
    /* DESCRIPTION:
        we init the debug init.
    */
    cv::Mat interface_visuel(450, 800, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat init_local_grid(25, 50, CV_8UC3, cv::Scalar(255, 255, 255));
    cv::Mat init_local_grey_grid(25, 50, CV_8UC1, cv::Scalar(255));
    cv::circle(interface_visuel, cv::Point(400,400),4, cv::Scalar(0,0,0), cv::FILLED, 1, 0);
    // cv::circle(init_local_grid, cv::Point(24,24),0, cv::Scalar(0,0,255), cv::FILLED, 0, 0);

    // Top Left Corner
    cv::Point p1(int(400-(robot_sensor_data.lidar.largeur/200)*400/5), int(400-(((robot_sensor_data.lidar.longeur+robot_sensor_data.lidar.roue)/100)*400/5)));
    robot_sensor_data.lidar.p1 = p1;
  
    // Bottom Right Corner
    cv::Point p2(int(400+(robot_sensor_data.lidar.largeur/200)*400/5), int(400-(((robot_sensor_data.lidar.roue)/100)*400/5)));
    robot_sensor_data.lidar.p2 = p2;
  
    int thickness = 1;
  
    // Drawing the Rectangle
    cv::rectangle(interface_visuel, p1, p2,
              cv::Scalar(0, 0, 255),
              thickness, cv::LINE_8);

    debug_lidar = interface_visuel;
    local_grid  = init_local_grid;
    local_grey_grid = init_local_grey_grid;
    copy_local_grey_grid = init_local_grey_grid;
}

void Robot_system::debug_lidar_interface(cv::Mat interface_visuel)
{ 
  /*  DESCRIPTION:
      this function will take all lidar data and show them in beautiful 
      opencv interface.
  */

//   convert brut data to 2D points.
//   Point_2D last_save_for_WD;      
//   last_save_for_WD.i = 0;
//   last_save_for_WD.j = 0;
  for(auto point : list_points)
  {
    // int pixel_x = sin(point.angle)*point.value*400/5+400;
    // int pixel_y = cos(point.angle)*point.value*400/5+400;
    // if(pixel_x != 400 && pixel_y != 400 && point.value > 0.3)
    // {
    //   Point_2D new_p;
    //   new_p.i = pixel_x;
    //   new_p.j = pixel_y;
    //   // std::cout << "point(" << pixel_x << "," << pixel_y << ") ";
    //   if(pow((pow(new_p.i-last_save_for_WD.i,2)+pow(new_p.j-last_save_for_WD.j,2)),0.5)*5/400 > 0.1)
    //   {list_points.push_back(new_p);
    //     last_save_for_WD.i = pixel_x;
    //     last_save_for_WD.j = pixel_y;
    //   }
    // }
    // std::cout << point.angle << "," << point.value << " en pixel : " << pixel_x << "," << pixel_y << "\n";
    cv::circle(interface_visuel, cv::Point(point.i,point.j),1, cv::Scalar(0,0,0), cv::FILLED, 0, 0);
  } 
 
    int thickness = 1;
  if(robot_sensor_data.lidar.dangerous_obstacle)
  {
      thickness = -1;
  }
  cv::rectangle(interface_visuel, robot_sensor_data.lidar.p1, robot_sensor_data.lidar.p2,
              cv::Scalar(0, 0, 255),
              thickness, cv::LINE_8);
//   wall_detector(list_points, list_line_cluster);
//   int color_compteur = 0;
//   std::cout << "[LINE DETECTION]" << "\n";
//   for(auto line : *list_line_cluster)
//   {
//     //compute angle.
//     double t = pow(pow(line.total_i,2)+pow(line.total_j,2),0.5);
//     double angle = acos(line.total_i/t);
    

//     std::cout << "cluster: " << line.id << " ,size:" << line.nombre << " ,angle: " << angle << " totali:" << line.total_i << "," << line.total_j << "\n";
//     for(auto point : *list_points)
//     {
//       if(point.cluster == line.id)
//       {
//         cv::circle(interface_visuel, cv::Point(point.i,point.j),2, cv::Scalar(color_liste[color_compteur].B,color_liste[color_compteur].G,color_liste[color_compteur].R), cv::FILLED, 1, 0);
//       }
//     }
//     color_compteur += 1;
//     if(color_compteur >= color_liste.size()) {color_compteur = 0;}
//   }

//   cv::namedWindow("interface_visuel",cv::WINDOW_AUTOSIZE);
//   cv::imshow("interface_visuel", interface_visuel);

}
