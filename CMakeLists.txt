CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include Socket IO.
include(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/socket.io-client-cpp/CMakeLists.txt)

# include directories.
INCLUDE_DIRECTORIES(include)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread -o CPUUsage")

# find package.
FIND_PACKAGE(OpenCV 4 REQUIRED)
FIND_PACKAGE(slamcore_sdk REQUIRED)
find_package(ydlidar_sdk REQUIRED)

include_directories(
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})

# add the executable.
ADD_EXECUTABLE(rane_software 
    core/rane_software.cpp
    src/fonction.cpp
    src/robot_system.cpp
    src/connection_listener.cpp
    include/fonction.h
    include/robot_system.h
    include/connection_listener.h)

TARGET_LINK_LIBRARIES(rane_software
    ${OpenCV_LIBS}
    /usr/local/lib/libserial.so
    slamcore::slamcore
    sioclient
    pthread
    ${YDLIDAR_SDK_LIBRARIES})

# add weighted map executable maker.
ADD_EXECUTABLE(make_weighted_map
    core/make_weighted_map.cpp)

TARGET_LINK_LIBRARIES(make_weighted_map
    ${OpenCV_LIBS})
