/*
    DESCRIPTION:
        This code run the slamcore API and read a previous session map.
        You can localise in the previous map.
        And we use a function to convert quaternion to euler.
*/

#include "../include/fonction.h"
#include "../include/robot_system.h"

#include <slamcore/slamcore.hpp>

#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <system_error>
#include <thread>

int main(int argc, char* argv[])
try
{
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
    return -1;
  }

  std::cout << "Starting SLAM..." << std::endl;

  // ******************************************************************
  // Open the device
  // ******************************************************************
  //   slam->open();
    const std::string ses_path = "../data/session/dvic1.session";
    slam->openWithSession(ses_path.c_str());


  // ******************************************************************
  // Print versions
  // ******************************************************************
  const std::string slam_version =
    slam->getProperty<std::string>(slamcore::Property::FirmwareVersion);
  const std::string slam_build_ver =
    slam->getProperty<std::string>(slamcore::Property::FirmwareBuildVersion);
  const std::string slam_build_type =
    slam->getProperty<std::string>(slamcore::Property::FirmwareBuildType);

  std::cout << "Client Version: " << slamcore::getVersion() << "/"
            << slamcore::getBuildVersion() << "/" << slamcore::getBuildType() << std::endl;
  std::cout << "SLAM Version: " << slam_version << "/" << slam_build_ver << "/"
            << slam_build_type << std::endl;

  // ******************************************************************
  // Enable all the streams
  // ******************************************************************
  slam->setStreamEnabled(slamcore::Stream::Pose, true);

  // *****************************************************************
  // Register callbacks!
  // *****************************************************************

  Pose pose;

  slam->registerCallback<slamcore::Stream::ErrorCode>(
    [](const slamcore::ErrorCodeInterface::CPtr& errorObj) {
      const auto rc = errorObj->getValue();
      std::cout << "Received: ErrorCode" << std::endl;
      std::cout << "\t" << rc.message() << " / " << rc.value() << " / "
                << rc.category().name() << std::endl;
    });

  slam->registerCallback<slamcore::Stream::Pose>(
    [&pose](const slamcore::PoseInterface<slamcore::camera_clock>::CPtr& poseObj) {
      // std::cout << "Received: Pose" << std::endl;
      // std::cout << "\t" << poseObj->getTranslation().x() << ","
      //           << poseObj->getTranslation().y() << "," << poseObj->getTranslation().z()
      //           << std::endl
      //           << "\t" << poseObj->getRotation().x() << ","
      //           << poseObj->getRotation().y() << "," << poseObj->getRotation().z()
      //           << "," << poseObj->getRotation().w()
      //           << std::endl;

      pose.position.x    = poseObj->getTranslation().x();
      pose.position.y    = poseObj->getTranslation().y();
      pose.position.z    = poseObj->getTranslation().z();
      pose.orientation.x = poseObj->getRotation().x();
      pose.orientation.y = poseObj->getRotation().y();
      pose.orientation.z = poseObj->getRotation().z();
      pose.orientation.w = poseObj->getRotation().w();
    });

  // ******************************************************************
  // Start streaming
  // ******************************************************************
  slam->start();
  
  // ******************************************************************
  // Main receiving loop
  // ******************************************************************
  while (slam->spinOnce())
  {
    from_quaternion_to_euler(pose);
    std::cout << "Euler:" << "\t"
              << pose.euler.r << "," << pose.euler.p << "," << pose.euler.y
              << std::endl;
  }

  // ******************************************************************
  // Stop SLAM
  // ******************************************************************
  slam->stop();

  // ******************************************************************
  // Disconnect/Close SLAM
  // ******************************************************************
  slam->close();

  slam.reset();

  // ******************************************************************
  // De-initialise SLAMcore API
  // ******************************************************************
  slamcore::slamcoreDeinit();

  std::cout << "We're Done Here." << std::endl;

  return 0;
}
catch (const slamcore::slam_exception& ex)
{
  std::cerr << "system_error exception! " << ex.what() << " / " << ex.code().message()
            << " / " << ex.code().value() << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}
catch (const std::exception& ex)
{
  std::cerr << "Uncaught std::exception! " << ex.what() << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}
catch (...)
{
  std::cerr << "Uncaught unknown exception!" << std::endl;
  slamcore::slamcoreDeinit();
  return -1;
}

