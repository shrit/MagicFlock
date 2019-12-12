cmake_minimum_required(VERSION 3.9.0)
project(ILMR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp -lmavsdk")

find_package(gazebo REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost COMPONENTS serialization program_options date_time REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}  
  /usr/local/include/mavsdk
  )

set(file_system "/usr/lib/libstdc++fs.a")
set(pthread_lib "/usr/lib/libpthread.so")
set(mlpack "/meta/mlpack/build/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")

file(GLOB HEADER_LIST CONFIGURE_DEPENDS "src/include/*.hh")
file(GLOB HEADER_LIST_XX CONFIGURE_DEPENDS "src/include/*.hxx")

add_library(ILMR SHARED
  src/action.cc
  src/reward.cc  
  src/config_ini.cc
  src/data_set.cc
  src/gazebo.cc
  src/keyboard.cc
  src/joystick.cc
  src/log.cc
  src/logger.cc
  src/px4_device.cc
  src/settings.cc
  src/timer.cc
  src/time_steps.cc
  ${HEADER_LIST}
  ${HEADER_LIST_XX}
  )

target_include_directories(ILMR PUBLIC
  src/include
  )

target_compile_features(ILMR PUBLIC cxx_std_17)

target_link_libraries(ILMR
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${file_system}
  ${pthread_lib}
  ${mlpack}
  ${gomp}

  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_calibration  
  MAVSDK::mavsdk_offboard
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_shell  
  MAVSDK::mavsdk  
  )

install(FILES
  ${HEADER_LIST}
  ${HEADER_LIST_XX}
  ${THIRD_PARTY_LIST}
  
  DESTINATION include/ILMR
  )

install(TARGETS ILMR
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )
