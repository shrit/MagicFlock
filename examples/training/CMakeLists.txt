cmake_minimum_required(VERSION 3.9.0)
project(training)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions("-Wall -Werror -Wextra -O -Wno-parentheses -march=native -g3 
-fopenmp -lgomp ")

include_directories(
  /usr/local/include/ilmr
  )

set(ilmr "/usr/local/lib/libILMR.so")

add_executable(training
  main.cc 
  )

target_link_libraries(training       
  ${ilmr}
  )
