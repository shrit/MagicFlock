cmake_minimum_required(VERSION 3.9.0)
project(ILMR)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(CMake/cotire.cmake)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp")

find_package(gazebo REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  /usr/local/include/mavsdk
  include
  )

set(mlpack "/usr/local/lib/libmlpack.so")
set(gomp "/usr/lib/libgomp.so")

# Need to be moved into
#add_subdirectory(src/gazebo)

add_subdirectory(src/IL4MRC)

# target_include_directories(ILMR PUBLIC
#   src/include
#   )

# target_compile_features(ILMR PUBLIC cxx_std_17)
