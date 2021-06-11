#include <stdio.h>
#include <iostream>
#include <string.h>

#ifndef ROBOT_SYSTEM_H
#define ROBOT_SYSTEM_H
 
class Robot_system
{
    private:
        // VARIABLE.
        std::string robot_id;
        std::thread thread_1_localisation;
        std::thread thread_2_commande;

    public:
        // CONSTRUCTEUR.
        Robot_system(std::string val_id);

        // FONCTION.
        std::string get_id();

        // THREAD.
        void thread_LOCALISATION();
        void thread_COMMANDE();
};
 
#endif