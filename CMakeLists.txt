cmake_minimum_required(VERSION 3.9.0)

project(lemon)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp")

find_package(gazebo REQUIRED)

include_directories(

  ${GAZEBO_INCLUDE_DIRS}
 /usr/local/include/mavsdk 
 /usr/include/
  )

set(mavsdk_lib "/usr/local/lib64/libmavsdk.so")
set(mavsdk_action_lib "/usr/local/lib64/libmavsdk_action.so")
set(mavsdk_calibrattion_lib "/usr/local/lib64/libmavsdk_calibration.so")
set(mavsdk_offboard_lib "/usr/local/lib64/libmavsdk_offboard.so")
set(mavsdk_telemetry_lib "/usr/local/lib64/libmavsdk_telemetry.so")
set(mavsdk_shell_lib "/usr/local/lib64/libmavsdk_shell.so")

set(file_system "/usr/lib/libstdc++fs.a")
set(pthread_lib "/usr/lib/libpthread.so")
set(arma "/usr/lib/libarmadillo.so")
set(mlpack "/meta/mlpack/build/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")

set(boost_lib_serialization "/usr/lib/libboost_serialization.so")
set(boost_lib_program_options "/usr/lib/libboost_program_options.so")
set(boost_lib_time "/usr/lib/libboost_date_time.so")

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(lemon
  src/algo/training.cc
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

target_link_libraries(lemon   
  ${pthread_lib}
  ${arma}
  ${GAZEBO_LIBRARIES}
  ${mavsdk_lib}
  ${mavsdk_action_lib}
  ${mavsdk_calibrattion_lib}
  ${mavsdk_offboard_lib}
  ${mavsdk_shell_lib}
  ${mavsdk_telemetry_lib}

  ${gomp}
  ${mlpack}
  ${file_system}
  ${boost_lib_serialization}
  ${boost_lib_program_options}  
  ${boost_lib_time}  
  )


