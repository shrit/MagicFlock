cmake_minimum_required(VERSION 3.9.0)
project(sac_learning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp")

find_package(gazebo REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  /usr/local/include/ilmr
  /usr/local/include/mavsdk
  )

set(il4mrc "/usr/local/lib/libIL4MRC.so")
set(mlpack "/usr/local/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")
set(boost_seria "/usr/lib/libboost_serialization.a")
set(ResetPlugin "/usr/local/lib/libResetPlugin.so")
set(reset_model_msgs "/usr/local/lib/libreset_model_msgs.so")

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(sac_learning 
  main.cc 
  )

target_link_libraries(sac_learning
  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_calibration
  MAVSDK::mavsdk_offboard
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_shell
  MAVSDK::mavsdk
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${boost_seria}
  ${il4mrc}
  ${mlpack}
  ${gomp}
  ${ResetPlugin}
  ${reset_model_msgs}  
)
