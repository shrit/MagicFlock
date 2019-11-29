cmake_minimum_required(VERSION 3.9.0)
project(generate_data_set)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp ")

find_package(gazebo REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  /usr/local/include/ilmr
  /usr/local/include/mavsdk
  )

set(ilmr "/usr/local/lib/libILMR.so")
set(mavsdk "/usr/local/lib64/libmavsdk.so")
link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(generate_data_set 
  main.cc 
  )

target_link_libraries(generate_data_set       
  ${GAZEBO_LIBRARIES}
  ${mavsdk}
  ${ilmr}
  )
