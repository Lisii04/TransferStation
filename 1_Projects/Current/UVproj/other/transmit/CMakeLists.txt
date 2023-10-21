# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(uart)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

add_executable(uart uart_t.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})
# Declare the executable target built from your sources
