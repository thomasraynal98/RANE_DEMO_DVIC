CMAKE_MINIMUM_REQUIRED(VERSION 3.10)

# set the project name.
PROJECT(rane_software VERSION 1.0)

INCLUDE_DIRECTORIES(include)

# add the executable.
ADD_EXECUTABLE(rane_software 
    core/rane_software.cpp
    src/fonction.cpp
    include/fonction.h)
