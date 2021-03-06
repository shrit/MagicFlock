cmake_minimum_required(VERSION 3.9.0)
project(MagicFlock)

include(CMake/cotire.cmake)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(ARMADILLO_VERSION "8.400.0")
# Include modules in the CMake directory.
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake")

if (UNIX)
  include(GNUInstallDirs)
endif()

if(CMAKE_COMPILER_IS_GNUCC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -ftemplate-depth=1000 -Wno-parentheses -march=native -g3 -fopenmp -lgomp -Wno-unused-function -ffunction-sections -fdata-sections -pthread")
endif()

if(DEBUG)
    add_definitions(-DDEBUG)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -O0 -ftemplate-backtrace-limit=0")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c99 -g -O0")
endif()

find_package(Armadillo REQUIRED)
find_package(gazebo REQUIRED)
find_package(MAVSDK REQUIRED)
find_package(Boost COMPONENTS date_time REQUIRED)
find_package(Protobuf REQUIRED)

set(MagicFlock_INCLUDE_DIRS ${MagicFlock_INCLUDE_DIRS} ${MLPACK_INCLUDE_DIRS} 
  ${ARMADILLO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${MAVSDK_INCLUDE_DIRS})
set(MagicFlock_LIBRARIES ${MagicFlock_LIBRARIES} ${MLPACK_LIBRARIES} ${ARMADILLO_LIBRARIES} ${MAVSDK_LIBRARIES} ${BOOST_INCLUDE_DIRS})

include_directories(
  /usr/local/include/mavsdk
  /usr/include/ignition/math6
  include
)

# set(mlpack "/usr/local/lib/libmlpack.so")
# set(gomp "/usr/lib/libgomp.so")
include_directories(BEFORE ${MLPACK_INCLUDE_DIRS})
include_directories(BEFORE ${CMAKE_SOURCE_DIR}/src/)

add_subdirectory(src/MagicFlock)

# target_include_directories(ILMR PUBLIC
#   src/include
#   )

# target_compile_features(ILMR PUBLIC cxx_std_17)
