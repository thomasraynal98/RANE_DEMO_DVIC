#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "../include/fonction.h"

constexpr const char* const SERIAL_PORT_1 = "/dev/tty9" ;
constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB1" ;

void get_available_port(std::vector<LibSerial::SerialPort*> &available_ports)
{
    /*
        DESCRIPTOR      : this function will check all available serial ports 
            with this type of name, ttyACMX, ttyUSBX.
        INPUT/OUTPUT    :
        * available_ports   > vector of available serial ports
    */
    // LibSerial::SerialPort serial_port();
    // available_ports.push_back(serial_port);
    
    // Check all ttyACMX.
    for (int i=0; i<4; ++i)
    {
        LibSerial::SerialPort* serial_port = new LibSerial::SerialPort;
        std::string name_port = "/dev/ttyACM" + std::__cxx11::to_string(i);
        bool is_available = true;

        try{ serial_port->Open(name_port);}
        catch (LibSerial::OpenFailed ex)
        {
            std::cout << "Failed to open SerialPort : " << name_port << std::endl;
            is_available = false;
        }

        if(is_available)
        {
            std::cout << serial_port->GetFileDescriptor() << std::endl;
            available_ports.push_back(serial_port);
        }
    }

    // Check all ttyUSBX.
}

int main() {

    std::vector<LibSerial::SerialPort*> available_ports;

    get_available_port(available_ports);

    for(auto port : available_ports)
    {
        std::cout << (int)port->GetBaudRate() << "\n";
    }

    delete available_ports;

	return 0;
}