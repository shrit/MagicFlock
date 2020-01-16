cmake_minimum_required(VERSION 3.9.0)
project(training)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O0 -Wno-parentheses -march=native -g3 
-fopenmp -lgomp ")

find_package(Armadillo REQUIRED)
find_package(gazebo REQUIRED)

include_directories(
  /usr/local/include/ilmr
  ${BOOST_INCLUDE_DIRS}
	${GAZEBO_INCLUDE_DIRS}
  )

set(ilmr "/usr/local/lib/libILMR.so")
set(mlpack "/meta/mlpack/build/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")
set(boost_seria "/usr/lib/libboost_serialization.a")

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(training
  main.cc
  )

target_link_libraries(training
  ${ARMADILLO_LIBRARIES}
	${GAZEBO_LIBRARIES}
  ${boost_seria}
  ${ilmr}
  ${mlpack}
  ${gomp}
  )
