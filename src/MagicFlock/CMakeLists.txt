set(DIRS
  actions
  algorithms
  check
  controller
  data
  dists
  swarm_model
  input
  metrics
  neighbors
  simulator
  state
  third_party
  util
)

foreach(dir ${DIRS})
  add_subdirectory(${dir})
endforeach()

add_library(IL4MRC SHARED ${IL4MRC_SRCS})

target_link_libraries(IL4MRC
  ${IL4MRC}
  ${mlpack}
  ${gomp}
  stdc++fs
  pthread

  MAVSDK::mavsdk_action
  MAVSDK::mavsdk_calibration
  MAVSDK::mavsdk_offboard
  MAVSDK::mavsdk_telemetry
  MAVSDK::mavsdk_shell
  MAVSDK::mavsdk
)

cotire(IL4MRC)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/src/IL4MRC/simulator/gazebo
)

file(GLOB_RECURSE IL4MRC_HEADER_LIST_H RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)
file(GLOB_RECURSE IL4MRC_HEADER_LIST_HPP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)
file(GLOB PROTO_HEADER_LIST CONFIGURE_DEPENDS 
 "${CMAKE_CURRENT_BINARY_DIR}/src/IL4MRC/simulator/gazebo/*.h")

set(INCLUDE_FILES ${IL4MRC_HEADER_LIST_H} ${IL4MRC_HEADER_LIST_HPP} ${PROTO_HEADER_LIST})

message ("print cmake binary dir" "${CMAKE_BINARY_DIR}")
message ("print cmake binary dir" "${CMAKE_CURRENT_BINARY_DIR}")
add_custom_target(il4mrc_headers)
add_custom_command(TARGET il4mrc_headers POST_BUILD
  COMMENT "Moving header files to include/IL4MRC/"
  COMMAND ${CMAKE_COMMAND} ARGS -E
    make_directory ${CMAKE_BINARY_DIR}/include/
)

foreach(incl_file ${INCLUDE_FILES})
  add_custom_command(TARGET il4mrc_headers POST_BUILD
    COMMAND ${CMAKE_COMMAND} ARGS -E
      copy ${CMAKE_CURRENT_SOURCE_DIR}/${incl_file}
           ${CMAKE_BINARY_DIR}/include/IL4MRC/${incl_file}
      BYPRODUCTS ${CMAKE_BINARY_DIR}/include/IL4MRC/${incl_file})
endforeach()

add_custom_command(TARGET il4mrc_headers POST_BUILD
  COMMENT "Moving header files to include/IL4MRC/"
  COMMAND ${CMAKE_COMMAND} ARGS -E
    copy ${CMAKE_BINARY_DIR}/src/IL4MRC/simulator/gazebo/ResetModel.pb.h
         ${CMAKE_BINARY_DIR}/include/IL4MRC/simulator/gazebo/ResetModel.pb.h
)

install(DIRECTORY "${CMAKE_BINARY_DIR}/include/IL4MRC" DESTINATION
    "${CMAKE_INSTALL_INCLUDEDIR}")

set(INSTALL_TARGETS IL4MRC)

install(TARGETS ${INSTALL_TARGETS}
  LIBRARY DESTINATION ${IL4MRC_LIBS_DIR}
  ARCHIVE DESTINATION ${IL4MRC_LIBS_DIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_dependencies(IL4MRC il4mrc_headers)
