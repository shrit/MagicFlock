cmake_minimum_required(VERSION 3.1.0)

## Specify your project's name
project(controller)

# Enable strict handling of warnings

add_definitions(" -fPIC  -std=c++11 -g" )

# Set moc compilling for qt 
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify your app's executable name, and list of source files used to create it.

set ( SOURCES  
  main.cc
  controller.cc
  
  )

set (BOOST_INCLUDE_DIRS "/meta/local/include/")

# Specify include directories
include_directories(

  ${BOOST_INCLUDE_DIRS}
  
  /usr/local/include/dronecore
  /usr/include/
  #   # ... any other header files
  )


# Find required pacakge by qt to link agaisnt

# Specify variable 'dronecore_lib' containing location of DroneCore library.
set(dronecore_lib "/usr/local/lib/libdronecore.so")
set(dronecore_offboard_lib "/usr/local/lib/libdronecore_offboard.so")
set(dronecore_telemetry_lib "/usr/local/lib/libdronecore_telemetry.so")
set(dronecore_action_lib "/usr/local/lib/libdronecore_action.so")
set(ncurses_lib "/usr/lib/libncurses.so")
set(boost_lib_system "/meta/local/lib/libboost_system.so")
set(boost_lib_thread "/meta/local/lib/libboost_thread.so")
set(boost_lib_program_options "/meta/local/lib/libboost_program_options.so")


add_executable(controller  
  
 ${SOURCES}  
  # ... any other source files
  )



# Specify your app's executable name and a list of linked libraries
target_link_libraries(controller
  # Add  'dronecore_lib' variable defining where the DroneCore library can be found.
  
  ${dronecore_lib}
  ${dronecore_offboard_lib}
  ${dronecore_action_lib}
  ${dronecore_telemetry_lib}
  ${ncurses_lib}
  ${boost_lib_system}
  ${boost_lib_thread}
  ${boost_lib_program_options}

  )



