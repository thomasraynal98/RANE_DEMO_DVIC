CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

# include directories.
INCLUDE_DIRECTORIES(include)

# find package.
FIND_PACKAGE(OpenCV 4)


# add the executable.
ADD_EXECUTABLE(rane_software 
    core/rane_software.cpp
    src/fonction.cpp
    include/fonction.h)

TARGET_LINK_LIBRARIES(rane_software
    ${OpenCV_LIBS}
    /usr/local/lib/libserial.so)