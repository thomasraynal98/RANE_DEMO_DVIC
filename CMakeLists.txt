CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# set good gcc version.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -pthread")

# find package.
FIND_PACKAGE(OpenCV 4)

# add the executable.
ADD_EXECUTABLE(rane_software 
    core/rane_software.cpp
    src/fonction.cpp
    src/robot.cpp
    include/fonction.h
    include/robot.h)

TARGET_LINK_LIBRARIES(rane_software
    ${OpenCV_LIBS}
    /usr/local/lib/libserial.so)