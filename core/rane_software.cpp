#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string.h>

#include <libserial/SerialPort.h>
#include <libserial/SerialStream.h>

#include "../include/fonction.h"

constexpr const char* const SERIAL_PORT_1 = "/dev/ttyACM1" ;
constexpr const char* const SERIAL_PORT_2 = "/dev/ttyUSB1" ;

int main() {

	using LibSerial::SerialPort ;
    using LibSerial::SerialStream ;

    // Instantiate a Serial Port and a Serial Stream object.
    SerialPort serial_port ; 
    SerialStream serial_stream ;

    // Open the hardware serial ports.
	std::cout << "IsOpen:" << serial_port.IsOpen();
    try
    {
        serial_port.Open(SERIAL_PORT_1);
    }
    catch (LibSerial::OpenFailed ex)
    {
        std::cout << "Failed to open SerialPort ";
    }
    // serial_stream.Open( SERIAL_PORT_2 ) ;

    // // Set the baud rates.
    // using LibSerial::BaudRate ;
    // serial_port.SetBaudRate( BaudRate::BAUD_115200 ) ;
    // serial_stream.SetBaudRate( BaudRate::BAUD_115200 ) ;

    // char write_byte_1 = 'a' ;
    // char write_byte_2 = 'b' ;

    // char read_byte_1 = 'A' ;
    // char read_byte_2 = 'B' ;

    // // Write a character.
    // serial_port.WriteByte(write_byte_1) ;
    // serial_stream << write_byte_2 ;

    // size_t timeout_milliseconds = 5 ;

    // using LibSerial::ReadTimeout ;
    // try
    // {
    //     // Read a character.
    //     serial_port.ReadByte(read_byte_1, timeout_milliseconds) ;
    //     serial_stream >> read_byte_2 ;
    // }
    // catch (const ReadTimeout&)
    // {
    //     std::cerr << "The Read() call has timed out." << std::endl ;
    // }

    // std::cout << "serial_port read:   " << read_byte_1 << std::endl ;
    // std::cout << "serial_stream read: " << read_byte_2 << std::endl ;

    // // Close the Serial Port and Serial Stream.
    // serial_port.Close() ;
    // serial_stream.Close() ;

	return 0;
}