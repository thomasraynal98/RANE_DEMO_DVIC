#include <stdio.h>
#include <iostream>
#include <string.h>
#include <chrono>
#include <thread>
#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>
#include <unistd.h>

using namespace std;

int main(){
    std::string val;
    ifstream ifile("/sys/class/thermal/thermal_zone1/temp");
    // ifile >> val;    // lire jusqu'a "/".
    // val.rdbuf();     // lire tout.
    ifile.close();
    std::cout << val;
}