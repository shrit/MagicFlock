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
  controller.cc
  settings.cc
  gazebo.cc
  )

set (BOOST_INCLUDE_DIRS "/meta/local/include/")
#set (BOOST_66_INCLUDE_DIRS "/meta/local_boost_66/include")

# Specify include directories
include_directories(

 ${BOOST_INCLUDE_DIRS}
 #${BOOST_66_INCLUDE_DIRS}
 ${GAZEBO_INCLUDE_DIRS}
  /usr/local/include/dronecode_sdk
  /usr/include/
#  /opt/ros/melodic/include
  # ... any other header files
  )


# Find required pacakge by qt to link agaisnt
#find_library(ros_lib HINT "/opt/ros/melodic/lib")

#find_package(catkin REQUIRED COMPONENTS roscpp rospy geometric_msgs genmsg)


# Specify variable 'dronecode_sdk_lib' containing location of Dronecode_Sdk library.
set(dronecode_sdk_lib "/usr/local/lib/libdronecode_sdk.so")
set(dronecode_sdk_offboard_lib "/usr/local/lib/libdronecode_sdk_offboard.so")
set(dronecode_sdk_telemetry_lib "/usr/local/lib/libdronecode_sdk_telemetry.so")
set(dronecode_sdk_action_lib "/usr/local/lib/libdronecode_sdk_action.so")
set(ncurses_lib "/usr/lib/libncurses.so")
set(pthread_lib "/usr/lib/libpthread.so")

set(boost_lib_system "/meta/local/lib/libboost_system.so")
set(boost_lib_thread "/meta/local/lib/libboost_thread.so")
set(boost_lib_program_options "/meta/local/lib/libboost_program_options.so")
# set(ros_lib "/opt/ros/melodic/lib/libroscpp.so")
# set(ros_console_lib "/opt/ros/melodic/lib/librosconsole.so")
# set(ros_serialization_lib "/opt/ros/melodic/lib/libroscpp_serialization.so")
# set(ros_lib_lib "/opt/ros/melodic/lib/libroslib.so")

#old boost
# set(boost_lib_system_66 "/meta/local_boost_66/lib/libboost_system.so")
# set(boost_lib_filesystem_66 "/meta/local_boost_66/lib/libboost_filesystem.so")
# set(boost_lib_thread_66 "/meta/local_boost_66/lib/libboost_thread.so")
# set(boost_lib_chrono_66 "/meta/local_boost_66/lib/libboost_chrono.so")
# set(boost_lib_regexp_66 "/meta/local_boost_66/lib/libboost_regex.so")
# set(boost_lib_program_options "/meta/local_boost_66/lib/libboost_program_options.so")



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
  # ${ros_lib}
  # ${ros_console_lib}
  # ${ros_serialization_lib}
  # ${ros_lib_lib}
  ${GAZEBO_LIBRARIES}
  ${dronecode_sdk_lib}
  ${dronecode_sdk_offboard_lib}
  ${dronecode_sdk_action_lib}
  ${dronecode_sdk_telemetry_lib}
  ${ncurses_lib}
  
  ${boost_lib_system}
  ${boost_lib_thread}
  ${boost_lib_program_options}
  
  # ${boost_lib_system_66}
  # ${boost_lib_filesystem_66}
  # ${boost_lib_thread_66}
  # ${boost_lib_chrono_66}
  # ${boost_lib_regexp_66}
  # ${boost_lib_program_options}
  )



