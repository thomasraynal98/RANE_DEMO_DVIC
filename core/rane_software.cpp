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
#include "../include/robot_system.h"

#include <iostream>
#include <ctime>
#include <ratio>
#include <chrono>
using namespace std::chrono;

int main(){
    Robot_system rane("MK2R2");
    std::cout << rane.get_id();
}