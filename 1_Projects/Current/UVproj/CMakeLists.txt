# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(UVProj)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Declare the executable target built from your sources
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
add_executable(UVProj main.cpp func_rel.cpp)
target_include_directories(UVProj PRIVATE ${Python3_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES})

# Link your application with OpenCV libraries
target_link_libraries(UVProj ${OpenCV_LIBS})
