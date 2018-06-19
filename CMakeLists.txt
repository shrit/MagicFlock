cmake_minimum_required(VERSION 2.8.12)

## Specify your project's name
project(controller)

# Enable strict handling of warnings
if(NOT MSVC)
    add_definitions("-std=c++11 -g" )
else()
    add_definitions("-std=c++11 -g")
endif()

# Specify your app's executable name, and list of source files used to create it.
add_executable(controller
  main.cc
    # ... any other source files
)
#target_link_libraries(your_executable_name
#    dronecore  #All apps link against dronecore library
    # ... any other linked libraries
#)
# Specify include directories
include_directories(
  /usr/local/include/dronecore
  /usr/include/
)

# Specify variable 'dronecore_lib' containing location of DroneCore library.

set(dronecore_lib "/usr/local/lib/libdronecore.so")
set(dronecore_offboard_lib "/usr/local/lib/libdronecore_offboard.so")
set(dronecore_telemetry_lib "/usr/local/lib/libdronecore_telemetry.so")
set(dronecore_action_lib "/usr/local/lib/libdronecore_action.so")
set(ncurses_lib "/usr/lib/libncurses.so")


# Specify your app's executable name and a list of linked libraries
target_link_libraries(controller
  # Add  'dronecore_lib' variable defining where the DroneCore library can be found.
  ${ncurses_lib}
  ${dronecore_lib}
  ${dronecore_offboard_lib}
  ${dronecore_action_lib}
  ${dronecore_telemetry_lib}
  # dronecore # Link against library named dronecore in standard install location
  # ... any other linked libraries
  )



