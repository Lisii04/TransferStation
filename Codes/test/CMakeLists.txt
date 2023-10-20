# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(uart)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Declare the executable target built from your sources
add_executable(uart uart.cpp)
