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
  /usr/local/include/mavsdk
  include
  )

set(mlpack "/usr/local/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")

file(GLOB HEADER_LIST CONFIGURE_DEPENDS "src/include/*.hh")
file(GLOB HEADER_LIST_XX CONFIGURE_DEPENDS "src/include/*.hxx")
file(GLOB HEADER_LIST_PP CONFIGURE_DEPENDS "src/include/*.hpp")

add_subdirectory( src/gazebo )

add_library(ILMR SHARED
  src/continuous_actions.cc
  src/dataset.cc
  src/discret_actions.cc
  src/evaluate_model.cc
  src/flocking.cc
  src/keyboard.cc
  src/joystick.cc
  src/logger.cc
  src/px4_device.cc
  src/real_time_samples.cc
  src/time.cc
  src/time_steps.cc
  ${HEADER_LIST}
  ${HEADER_LIST_XX}
  ${HEADER_LIST_PP}
  )

target_include_directories(ILMR PUBLIC
  src/include
  )

target_compile_features(ILMR PUBLIC cxx_std_17)

target_link_libraries(ILMR
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${BOOST_LIBRARIES}
  ${mlpack}
  ${gomp}
  stdc++fs
  pthread

  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_calibration  
  MAVSDK::mavsdk_offboard
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_shell  
  MAVSDK::mavsdk
  )
target_include_directories(ILMR PRIVATE 
  ${CMAKE_CURRENT_BINARY_DIR}/src/gazebo
)
file(GLOB PROTO_HEADER_LIST CONFIGURE_DEPENDS 
"${CMAKE_CURRENT_BINARY_DIR}/src/gazebo/*.h")

install(FILES
  ${HEADER_LIST}
  ${HEADER_LIST_XX}
  ${PROTO_HEADER_LIST}
  ${THIRD_PARTY_LIST}
  
  DESTINATION include/ILMR
  )

install(TARGETS ILMR
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )
