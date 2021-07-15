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

// inline int modulo(int a, int b) { return a < 0 ? b - (-a % b): a % b; }

int main()
{
    Robot_system robot1 = Robot_system("mk2r2");
}