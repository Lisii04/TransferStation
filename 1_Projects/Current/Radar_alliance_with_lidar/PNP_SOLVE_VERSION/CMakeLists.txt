# cmake needs this line
cmake_minimum_required(VERSION 3.1)

# Define project name
project(Opencv-camera-pnpsolve)

# Find OpenCV, you may need to set OpenCV_DIR variable
# to the absolute path to the directory containing OpenCVConfig.cmake file
# via the command line or GUI
find_package(OpenCV REQUIRED)
find_package(PCL 1.3 REQUIRED)

# Enable C++11
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Declare the executable target built from your sources
include_directories(${PCL_INCLUDE_DIRS})                #4
link_directories(${PCL_LIBRARY_DIRS})                   #5
add_definitions(${PCL_DEFINITIONS})                     #6
# add_executable(Opencv-camera-pnpsolve camera_calibration.cpp)
# add_executable(Opencv-camera-pnpsolve chesscoard_capture.cpp)
# add_executable(Opencv-camera-pnpsolve pnp_solve.cpp)
add_executable(Opencv-camera-pnpsolve pcl_test.cpp)

# Link your application with OpenCV libraries
target_link_libraries(Opencv-camera-pnpsolve PRIVATE ${OpenCV_LIBS} ${PCL_LIBRARIES})