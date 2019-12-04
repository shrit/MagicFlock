cmake_minimum_required(VERSION 3.9.0)
project(iterative_learning)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp ")

find_package(gazebo REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost COMPONENTS serialization REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${BOOST_INCLUDE_DIRS}
  /usr/local/include/ilmr
  /usr/local/include/mavsdk
  )

set(ilmr "/usr/local/lib/libILMR.so")
set(mavsdk "/usr/local/lib64/libmavsdk.so")
set(mlpack "/meta/mlpack/build/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")
set(boost_seria "/usr/lib/libboost_serialization.a")

link_directories(${GAZEBO_LIBRARY_DIRS})

add_executable(iterative_learning 
  main.cc 
  )

target_link_libraries(iterative_learning
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${boost_seria}
  ${mavsdk}
  ${ilmr}
  ${mlpack}
  ${gomp}
  )
