#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>
#include <sstream>
#include "../include/fonction.h"
#include "../include/robot_system.h"
#include <libserial/SerialPort.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <thread>
#include <libserial/SerialStream.h>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <unistd.h>
#include <iostream>
#include <cstdlib>
#include <signal.h>
#include <csignal>

#include <iostream>
#include <csignal>

int main () {
    Robot_system robot1 = Robot_system();
    // std::cout;
    // auto oui = sio::client();
    // oui.connect('https://api-devo-docker.herokuapp.com/')

    // int debug_mode = 1;
    // bool is_openable = true;

    // LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
    // std::string name_port = "/dev/ttyUSB0";

    // try{ serial_port->Open(name_port);}
    // catch (LibSerial::OpenFailed ex)
    // {
    //     if(debug_mode==1) {std::cout << "Failed to open SerialPort : " << name_port << std::endl;}
    //     is_openable = false;
    // }
    // catch (LibSerial::AlreadyOpen ex)
    // {
    //     if(debug_mode==1) {std::cout << "SerialPort already open : " << name_port << std::endl;}
    //     is_openable = false;
    // }

    // if(is_openable)
    // {
    //     std::cout << "Success to open SerialPort : " << name_port << std::endl;
    //     serial_port->Write("0/");
    //     // usleep(500000);
    //     std::string reponse;
    //     serial_port->ReadLine(reponse);
    //     if(debug_mode==1) {std::cout << "reponse:" << reponse;}
    // }

    return 0;
}
