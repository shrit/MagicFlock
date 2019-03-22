cmake_minimum_required(VERSION 3.9.0)

project(QCopter)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_definitions(" -fPIC -O0 -g")

find_package(gazebo REQUIRED)

add_subdirectory(src)

