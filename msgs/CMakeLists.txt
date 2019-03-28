cmake_minimum_required(VERSION 3.9.0)

find_package(Protobuf REQUIRED)

set(PROTOBUF_IMPORT_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()



PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS 
  Groundtruth.proto
  header.proto
  time.proto
  )

add_library(NS3_ROS SHARED ${PROTO_SRCS} ${PROTO_HDRS})
target_link_libraries(NS3_ROS ${PROTOBUF_LIBRARY})
