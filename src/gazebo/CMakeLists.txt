cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(SwarmMsgs
  msgs/spawn.proto
  ${PROTOBUF_IMPORT_DIRS}/vector3d.proto
  ${PROTOBUF_IMPORT_DIRS}/header.proto
  ${PROTOBUF_IMPORT_DIRS}/time.proto
)

PROTOBUF_GENERATE_CPP(MSG_PROTO_SRCS MSG_PROTO_HDRS ${SwarmMsgs})

add_library(swarm_plugin_msgs SHARED ${MSG_PROTO_SRCS})
link_libraries(swarm_plugin_msgs)

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
target_include_directories(ResetPlugin PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

install(TARGETS reset_model_msgs
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )

install(TARGETS ResetPlugin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  )
