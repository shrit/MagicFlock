cmake_minimum_required(VERSION 3.1.0)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
## Specify your project's name
project(controller)

# Enable strict handling of warnings

#add_definitions(" -fPIC -O0 -g -std=c++11")

# Set moc compilling for qt 
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(gazebo REQUIRED)

# Specfiy a list of source files .

set ( SOURCES  
  main.cc
  px4_device.cc
  settings.cc
  gazebo.cc
  algo/q_learning.cc
  )

set (BOOST_INCLUDE_DIRS "/meta/local/include/")


# Specify include directories
include_directories(

 ${BOOST_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
  /usr/local/include/dronecode_sdk
  /usr/include/
  /usr/include/eigen3
  /meta/DeviceController/algo/

  # ... any other header files
  )


# Find required pacakge by qt to link against

#find_library(ros_lib HINT "/opt/ros/melodic/lib")


# Specify variable 'dronecode_sdk_lib' containing location of Dronecode_Sdk library.
set(dronecode_sdk_lib "/usr/local/lib/libdronecode_sdk.so")
set(dronecode_sdk_offboard_lib "/usr/local/lib/libdronecode_sdk_offboard.so")
set(dronecode_sdk_telemetry_lib "/usr/local/lib/libdronecode_sdk_telemetry.so")
set(dronecode_sdk_action_lib "/usr/local/lib/libdronecode_sdk_action.so")
set(dronecode_sdk_calibrattion_lib "/usr/local/lib/libdronecode_sdk_calibration.so")
set(ncurses_lib "/usr/lib/libncurses.so")
set(pthread_lib "/usr/lib/libpthread.so")
set(atomic_lib "/usr/lib/libatomic.so")		
set(boost_lib_system "/meta/local/lib/libboost_system.so")
set(boost_lib_thread "/meta/local/lib/libboost_thread.so")
set(boost_lib_program_options "/meta/local/lib/libboost_program_options.so")


link_directories(${GAZEBO_LIBRARY_DIRS})

list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(controller  
  
 ${SOURCES}  
  # ... any other source files
  )


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")



#link_directories("/opt/ros/melodic/lib/")

# Specify your app's executable name and a list of linked libraries
target_link_libraries(controller
  
  # Add  'dronecode_sdk_lib' variable defining
  #where the Dronecode_Sdk library can be found
  ${pthread_lib}
  ${atomic_lib}
  ${GAZEBO_LIBRARIES}
  ${dronecode_sdk_lib}
  ${dronecode_sdk_offboard_lib}
  ${dronecode_sdk_action_lib}
  ${dronecode_sdk_telemetry_lib}
  ${dronecode_sdk_calibrattion_lib}
  ${ncurses_lib}
  
  ${boost_lib_system}
  ${boost_lib_thread}
  ${boost_lib_program_options}
  
  )



