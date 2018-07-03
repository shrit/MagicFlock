cmake_minimum_required(VERSION 3.1.0)

## Specify your project's name
project(controller)

# Enable strict handling of warnings

add_definitions("-std=c++11 -g" )

# Set moc compilling for qt 
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# Specify your app's executable name, and list of source files used to create it.
add_executable(controller
  main.cc
  controller.hxx
  
    # ... any other source files
    )


# Specify include directories
include_directories(
  /usr/local/include/dronecore
  /usr/include/

  # ... any other header files
)


# Find required pacakge by qt to link agaisnt
find_package( Qt5 REQUIRED
  Widgets
  Core
  GamePad
  )

# Specify variable 'dronecore_lib' containing location of DroneCore library.
set(dronecore_lib "/usr/local/lib/libdronecore.so")
set(dronecore_offboard_lib "/usr/local/lib/libdronecore_offboard.so")
set(dronecore_telemetry_lib "/usr/local/lib/libdronecore_telemetry.so")
set(dronecore_action_lib "/usr/local/lib/libdronecore_action.so")


# Specify your app's executable name and a list of linked libraries
target_link_libraries(controller
  # Add  'dronecore_lib' variable defining where the DroneCore library can be found.

  ${dronecore_lib}
  ${dronecore_offboard_lib}
  ${dronecore_action_lib}
  ${dronecore_telemetry_lib}

  # qt libraries
  Qt5:Widgets
  Qt5:Core
  Qt5:gamepad  
  )



