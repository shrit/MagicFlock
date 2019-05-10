cmake_minimum_required(VERSION 3.9.0)

project(QCopter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp")

find_package(gazebo REQUIRED)

include_directories(

  ${GAZEBO_INCLUDE_DIRS}
 /usr/local/include/dronecode_sdk 
 /usr/include/
  )


#find_package(boost)
set(dronecode_sdk_lib "/usr/local/lib/libdronecode_sdk.so")
set(dronecode_sdk_offboard_lib "/usr/local/lib/libdronecode_sdk_offboard.so")
set(dronecode_sdk_telemetry_lib "/usr/local/lib/libdronecode_sdk_telemetry.so")
set(dronecode_sdk_action_lib "/usr/local/lib/libdronecode_sdk_action.so")
set(dronecode_sdk_calibrattion_lib "/usr/local/lib/libdronecode_sdk_calibration.so")
set(file_system "/usr/lib/libstdc++fs.a")
set(ncurses_lib "/usr/lib/libncurses.so")
set(pthread_lib "/usr/lib/libpthread.so")
set(arma "/usr/lib/libarmadillo.so")
set(mlpack "/usr/local/lib64/libmlpack.so.3.0")
set(gomp "/usr/lib/libgomp.so")

set(boost_lib_serialization "/usr/lib/libboost_serialization.so")
set(boost_lib_program_options "/usr/lib/libboost_program_options.so")
set(boost_lib_time "/usr/lib/libboost_date_time.so")

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(QCopter    
  src/config_ini.cc
  src/data_set.cc
  src/gazebo.cc
  src/keyboard.cc
  third_party/joystick/joystick.cc
  src/log.cc  
  src/main.cc
  src/px4_device.cc
  src/settings.cc    
  )

target_link_libraries(QCopter   
  ${pthread_lib}
  ${arma}
  ${GAZEBO_LIBRARIES}
  ${dronecode_sdk_lib}
  ${dronecode_sdk_offboard_lib}
  ${dronecode_sdk_action_lib}
  ${dronecode_sdk_telemetry_lib}
  ${dronecode_sdk_calibrattion_lib}

  ${gomp}
  ${mlpack}
  ${file_system}
  # ${ncurses_lib}
  ${boost_lib_serialization}
  ${boost_lib_program_options}  
  ${boost_lib_time}  
  )


