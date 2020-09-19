cmake_minimum_required(VERSION 3.9.0)
project(IL4MRC)

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

set(IL4MRC_INCLUDE_DIRS ${IL4MRC_INCLUDE_DIRS} ${MLPACK_INCLUDE_DIRS} 
  ${ARMADILLO_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${MAVSDK_INCLUDE_DIRS})
set(IL4MRC_LIBRARIES ${IL4MRC_LIBRARIES} ${MLPACK_LIBRARIES} ${ARMADILLO_LIBRARIES} ${MAVSDK_LIBRARIES} ${BOOST_INCLUDE_DIRS})

include_directories(
  /usr/local/include/mavsdk
  include
)

# set(mlpack "/usr/local/lib/libmlpack.so")
# set(gomp "/usr/lib/libgomp.so")

add_subdirectory(src/IL4MRC)

# target_include_directories(ILMR PUBLIC
#   src/include
#   )

# target_compile_features(ILMR PUBLIC cxx_std_17)
