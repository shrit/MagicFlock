cmake_minimum_required(VERSION 3.9.0)
project(ILMR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp")

find_package(gazebo REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost COMPONENTS serialization program_options date_time REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${MAVSDK_INCLUDE_DIRS}
  /usr/local/include/mavsdk 
  /usr/include/
  )

set(file_system "/usr/lib/libstdc++fs.a")
set(pthread_lib "/usr/lib/libpthread.so")
set(mlpack "/meta/mlpack/build/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${ARMADILLO_LIBRARY_DIR}
  ${BOOST_LIBRARY_DIRS}
  ${MAVSDK_LIBRARY_DIRS}
  )

add_library(ILMR SHARED
  src/algo/action.cc
  src/algo/reward.cc  
  src/config_ini.cc
  src/data_set.cc
  src/gazebo.cc
  src/keyboard.cc
  third_party/joystick/joystick.cc
  src/log.cc
  src/px4_device.cc
  src/settings.cc    
  )

target_link_libraries(ILMR
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${MAVSDK_LIBRARIES}
  ${file_system}
  ${pthread_lib}
  ${mlpack}
  ${gomp}
  )

# install(FILES "${CMAKE_CURRENT_BINARY_DIR}/lib/"
#   DESTINATION "${CMAKE_INSTALL_LIBDIR}")
