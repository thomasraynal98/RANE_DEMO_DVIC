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
#include <thread>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include "../include/robot_system.h"

// CONSTRUCTOR.
Robot_system::Robot_system(std::string val_id)
{   
    // initialisation process.
    robot_general_state       = Robot_state().initialisation;
    robot_id                  = val_id;
    robot_speed               = -1;
    cpu_heat                  = -1;
    cpu_load                  = -1;
    fan_power                 = -1;

    // initialisation SLAM.
    init_slam_sdk();

    // initialisation map and points (TODO: in the future, no points will be in computer
    // only send from server).
    map_weighted = cv::imread(path_to_weighted_map, cv::IMREAD_GRAYSCALE);
    if(map_weighted.empty())
    {
        std::cout << "[ERROR] Could not read the image: " << path_to_weighted_map << std::endl;
    }

    // initialisation of debug map.
    debug_init_debug_map();

    // initialisation microcontroler.
    LibSerial::SerialPort* _serial_port_controle_A;
    LibSerial::SerialPort* _serial_port_sensor_B;
    __serial_port_controle_A  = &_serial_port_controle_A;
    __serial_port_sensor_B    = &_serial_port_sensor_B;
    *__serial_port_controle_A = get_available_port(0, controler_A_pong, true);
    *__serial_port_sensor_B   = get_available_port(0, controler_B_pong, true);

    // initialisation all thread.
    thread_1_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_2_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_3_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_4_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_5_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_6_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_7_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_8_last_hz_update   = std::chrono::high_resolution_clock::now();
    thread_9_last_hz_update   = std::chrono::high_resolution_clock::now();

    thread_1_localisation     = std::thread(&Robot_system::thread_LOCALISATION  , this, 50);
    thread_2_commande         = std::thread(&Robot_system::thread_COMMANDE      , this, 20);
    // thread_3_listener_MICROA  = std::thread(&Robot_system::thread_LISTENER      , this, 10, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, "A"); 
    // thread_4_speaker_MICROA   = std::thread(&Robot_system::thread_SPEAKER       , this,  2, __serial_port_controle_A, std::ref(state_A_controler), controler_A_pong, "A"); 
    // thread_5_listener_MICROB  = std::thread(&Robot_system::thread_LISTENER      , this, 10,  __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, "B"); 
    // thread_6_speaker_MICROB   = std::thread(&Robot_system::thread_SPEAKER       , this,  2,  __serial_port_sensor_B, std::ref(state_B_controler), controler_B_pong, "B");
    thread_7_listener_SERVER  = std::thread(&Robot_system::thread_SERVER_LISTEN , this, 20);
    thread_8_speaker_SERVER   = std::thread(&Robot_system::thread_SERVER_SPEAKER, this, 10); 
    thread_9_thread_ANALYSER  = std::thread(&Robot_system::thread_ANALYSER      , this, 10); 

    thread_1_localisation.join();
    thread_2_commande.join();
    // thread_3_listener_MICROA.join();
    // thread_4_speaker_MICROA.join();
    // thread_5_listener_MICROB.join();
    // thread_6_speaker_MICROB.join();
    thread_7_listener_SERVER.join();
    thread_8_speaker_SERVER.join();
    thread_9_thread_ANALYSER.join();

    // end of initialisation.
    robot_general_state       = Robot_state().waiting;
}

Robot_state::Robot_state()
{}

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

void Robot_system::init_slam_sdk()
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
        // return -1;
    }

    std::cout << "Starting SLAM..." << std::endl;
    // ******************************************************************
    // Open the device
    // ******************************************************************
    slam->openWithSession(path_to_current_session.c_str());
    // ******************************************************************
    // Enable all the streams
    // ******************************************************************
    slam->setStreamEnabled(slamcore::Stream::Pose, true);
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

    slam->registerCallback<slamcore::Stream::Pose>(
    [this](const slamcore::PoseInterface<slamcore::camera_clock>::CPtr& poseObj) {
      robot_position.position.x    = poseObj->getTranslation().x();
      robot_position.position.y    = poseObj->getTranslation().y();
      robot_position.position.z    = poseObj->getTranslation().z();
      robot_position.orientation.x = poseObj->getRotation().x();
      robot_position.orientation.y = poseObj->getRotation().y();
      robot_position.orientation.z = poseObj->getRotation().z();
      robot_position.orientation.w = poseObj->getRotation().w();
      from_quaternion_to_euler(robot_position);
    });

    // INIT ATTRIBU OBJECT.
    slamcore = std::move(slam);
}
catch (const slamcore::slam_exception& ex)
{
  std::cerr << "system_error exception! " << ex.what() << " / " << ex.code().message()
            << " / " << ex.code().value() << std::endl;
  slamcore::slamcoreDeinit();
//   return -1;
}
catch (const std::exception& ex)
{
  std::cerr << "Uncaught std::exception! " << ex.what() << std::endl;
  slamcore::slamcoreDeinit();
//   return -1;
}
catch (...)
{
  std::cerr << "Uncaught unknown exception!" << std::endl;
  slamcore::slamcoreDeinit();
//   return -1;
}

// FONCTION NAVIGATION.
void Robot_system::aStarSearch(cv::Mat grid, const Pair& src, const Pair& dest)
{
	// If the source is out of range
	if (!isValid(grid, src)) {
		printf("Source is invalid\n");
		return;
	}

	// If the destination is out of range
	if (!isValid(grid, dest)) {
		printf("Destination is invalid\n");
		return;
	}

	// Either the source or the destination is blocked
	if (!isUnBlocked(grid, src)
		|| !isUnBlocked(grid, dest)) {
		printf("Source or the destination is blocked\n");
		return;
	}

	// If the destination cell is the same as source cell
	if (isDestination(src, dest)) {
		printf("We are already at the destination\n");
		return;
	}

	// Create a closed list and initialise it to false which
	// means that no cell has been included yet This closed
	// list is implemented as a boolean 2D array
	bool closedList[grid.rows][grid.cols];
	memset(closedList, false, sizeof(closedList));

	// Declare a 2D array of structure to hold the details
	// of that cell
    // constexpr auto p = static_cast<int>(grid.cols);
    // const int pp = grid.rows;
	// array<array<cell, COL>, ROW> cellDetails;
    int cols = grid.cols;
    int rows = grid.rows;
    
    std::vector<std::vector<cell>> cellDetails(rows, std::vector<cell>(cols));

	int i, j;
	// Initialising the parameters of the starting node
	i = src.first, j = src.second;
	cellDetails[i][j].f = 0.0;
	cellDetails[i][j].g = 0.0;
	cellDetails[i][j].h = 0.0;
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

				Cell-->Popped Cell (i, j)
				N --> North	 (i-1, j)
				S --> South	 (i+1, j)
				E --> East	 (i, j+1)
				W --> West		 (i, j-1)
				N.E--> North-East (i-1, j+1)
				N.W--> North-West (i-1, j-1)
				S.E--> South-East (i+1, j+1)
				S.W--> South-West (i+1, j-1)
		*/
		for (int add_x = -1; add_x <= 1; add_x++) {
			for (int add_y = -1; add_y <= 1; add_y++) {
				Pair neighbour(i + add_x, j + add_y);
				// Only process this cell if this is a valid
				// one
				if (isValid(grid, neighbour)) {
					// If the destination cell is the same
					// as the current successor
					if (isDestination(neighbour, dest)) 
                    { // Set the Parent of
									// the destination cell
						cellDetails[neighbour.first][neighbour.second].parent = { i, j };
						printf("The destination cell is found\n");
						// tracePath(cellDetails, dest);
                        // high_resolution_clock::time_point t2 = high_resolution_clock::now();
 
                        // duration<double, std::milli> time_span = t2 - t1;

                        // Show. //////////////////////////////////////////////////////////////////
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
                        
                        Path.emplace(row, col);
                        while (!Path.empty()) {
                            Pair p = Path.top();
                            Path.pop();
                            printf("-> (%d,%d) ", p.first, p.second);
                        }
                        ///////////////////////////////////////////////////////////////////////////
                        
                        // std::cout << "It took me " << time_span.count() << " milliseconds.";
                        // std::cout << std::endl;

						return;
					}
					// If the successor is already on the
					// closed list or if it is blocked, then
					// ignore it. Else do the following
					else if (!closedList[neighbour.first][neighbour.second] && isUnBlocked(grid, neighbour)) 
                    {
						double gNew, hNew, fNew;
						gNew = cellDetails[i][j].g + 1.0;
						hNew = calculateHValue(neighbour, dest);
						fNew = gNew + hNew;

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
						if (cellDetails[neighbour.first][neighbour.second].f == -1 || cellDetails[neighbour.first][neighbour.second].f > fNew) 
                        {
							openList.emplace(fNew, neighbour.first,neighbour.second);

							// Update the details of this
							// cell
							cellDetails[neighbour.first][neighbour.second].g = gNew;
							cellDetails[neighbour.first][neighbour.second].h = hNew;
							cellDetails[neighbour.first][neighbour.second].f = fNew;
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
}

double Robot_system::calculateHValue(const Pair& src, const Pair& dest)
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
	return isValid(grid, point) && ((grid.at<uchar>(point.first,point.second) == 255) || (grid.at<uchar>(point.first,point.second) == 200));
}

bool Robot_system::isValid(cv::Mat grid, const Pair& point)
{ 
    // Returns true if row number and column number is in range.
    return (point.first >= 0) && (point.first < grid.rows) && (point.second >= 0) && (point.second < grid.cols);
}

void Robot_system::from_3DW_to_2DM()
{
    /*
        DESCRIPTION: this function will convert the 3D world pose into
            the 2D map pixel coordinate.
    */
}
// THREAD.
void Robot_system::thread_LOCALISATION(int frequency)
{
    /*
        DESCRIPTION: this thread will compute the SLAM algorythme
            and get the position of robot on the current map.
    */
    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();
    
    // START SLAM.
    slamcore->start();

    while(slamcore->spinOnce())
    {   
        // TIMING VARIABLE.
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_1_hz                = 1000/(double)time_span.count();
        thread_1_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)(time_of_loop));
        std::this_thread::sleep_until(next);
        // END TIMING VARIABLE.
        std::cout << "[THREAD-1]\n";

        std::cout << robot_position.position.x << ", " <<
                     robot_position.position.y << ", " <<
                     robot_position.position.z << "\n";
    }
}

void Robot_system::thread_COMMANDE(int frequency)
{
    /*
        DESCRIPTION: this thread will get all input data and user
            information to command all composant.
    */

    double time_of_loop = 1000/frequency;                  // en milliseconde.
    std::chrono::high_resolution_clock::time_point last_loop_time = std::chrono::high_resolution_clock::now();
    std::chrono::high_resolution_clock::time_point x              = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

    while(true)
    {   
        // TIMING VARIABLE.
        x                          = std::chrono::high_resolution_clock::now();         
        time_span                  = x-last_loop_time;
        thread_2_hz                = 1000/(double)time_span.count();
        thread_2_last_hz_update    = x;
        last_loop_time             = x;
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);
        // END TIMING VARIABLE.
        // std::cout << "[THREAD-2]\n"; ////////////////////////////////////////////////////

        if(robot_general_state == Robot_state::follow)
        {   
            // INFO : We are in follow mode.
            //     1. Check if we have a global path.
            if(true)
            {

            }
            else
            {
                // We need to compute a global path.
            }

        }
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

    // TIME VARIABLE
    int time_of_loop     = 1000/frequency;                                       // time wait until ping.
    int time_since_lost  = 500;                                                  // time to declare the port lost and close.
    bool is_lost         = false;

    std::chrono::high_resolution_clock::time_point timer_start, current_timer;
    std::chrono::duration<double, std::milli> time_span;
    auto next = std::chrono::high_resolution_clock::now();

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
        next                       += std::chrono::milliseconds((int)time_of_loop);
        std::this_thread::sleep_until(next);

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
        get_interne_data();

        // std::cout << "[THREAD-8]\n";
    }
}

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

        add_intern_sensors(affichage);
        add_state(affichage, 900, th_state, thread_9_hz, fond_th9);
        add_lines(affichage);

        cv::imshow("Interface analyse vision.", affichage);
        char c=(char)cv::waitKey(25);
        cv::namedWindow("Car",cv::WINDOW_AUTOSIZE);
        cv::imshow("Car", debug_visual_map);
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
    std::cout << "debug";
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