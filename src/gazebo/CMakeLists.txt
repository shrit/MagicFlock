cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(reset_model_msgs msgs/ResetModel.proto)
PROTOBUF_GENERATE_CPP(RES_PROTO_SRCS RES_PROTO_HDRS ${reset_model_msgs})

add_library(reset_model_msgs SHARED ${RES_PROTO_SRCS})
link_libraries(reset_model_msgs)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ResetPlugin SHARED gazebo_model_reset_plugin.cc)
target_link_libraries(ResetPlugin ${GAZEBO_LIBRARIES} reset_model_msgs)
