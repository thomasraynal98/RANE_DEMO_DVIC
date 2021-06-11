#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>

#include "../include/robot_system.h"


// CONSTRUCTOR.
Robot_system::Robot_system(std::string val_id)
{   
    // initialisation process.
    robot_id              = val_id;
    thread_1_localisation = std::thread(&thread_LOCALISATION);
    thread_2_commande     = std::thread(&thread_COMMANDE);

    thread_1_localisation.join();
    thread_2_commande.join();
}

// FONCTION.
std::string Robot_system::get_id()
{
    return robot_id;
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
        std::cout << " [THREAD] localisation thread. \n";
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
        std::cout << "[THREAD] commande thread. \n";
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}