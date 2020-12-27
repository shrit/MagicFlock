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

add_library(MagicFlock SHARED ${MagicFlock_SRCS})

target_link_libraries(MagicFlock
  ${MagicFlock}
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

cotire(MagicFlock)

include_directories(
  ${CMAKE_CURRENT_BINARY_DIR}/src/MagicFlock/simulator/gazebo
)

file(GLOB_RECURSE MagicFlock_HEADER_LIST_H RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)
file(GLOB_RECURSE MagicFlock_HEADER_LIST_HPP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)
file(GLOB PROTO_HEADER_LIST CONFIGURE_DEPENDS 
 "${CMAKE_CURRENT_BINARY_DIR}/src/MagicFlock/simulator/gazebo/*.h")

set(INCLUDE_FILES ${MagicFlock_HEADER_LIST_H} ${MagicFlock_HEADER_LIST_HPP} ${PROTO_HEADER_LIST})

message ("print cmake binary dir" "${CMAKE_BINARY_DIR}")
message ("print cmake binary dir" "${CMAKE_CURRENT_BINARY_DIR}")
add_custom_target(il4mrc_headers)
add_custom_command(TARGET il4mrc_headers POST_BUILD
  COMMENT "Moving header files to include/MagicFlock/"
  COMMAND ${CMAKE_COMMAND} ARGS -E
    make_directory ${CMAKE_BINARY_DIR}/include/
)

foreach(incl_file ${INCLUDE_FILES})
  add_custom_command(TARGET il4mrc_headers POST_BUILD
    COMMAND ${CMAKE_COMMAND} ARGS -E
      copy ${CMAKE_CURRENT_SOURCE_DIR}/${incl_file}
           ${CMAKE_BINARY_DIR}/include/MagicFlock/${incl_file}
      BYPRODUCTS ${CMAKE_BINARY_DIR}/include/MagicFlock/${incl_file})
endforeach()

add_custom_command(TARGET il4mrc_headers POST_BUILD
  COMMENT "Moving header files to include/MagicFlock/"
  COMMAND ${CMAKE_COMMAND} ARGS -E
    copy ${CMAKE_BINARY_DIR}/src/MagicFlock/simulator/gazebo/ResetModel.pb.h
         ${CMAKE_BINARY_DIR}/include/MagicFlock/simulator/gazebo/ResetModel.pb.h
)

install(DIRECTORY "${CMAKE_BINARY_DIR}/include/MagicFlock" DESTINATION
    "${CMAKE_INSTALL_INCLUDEDIR}")

set(INSTALL_TARGETS MagicFlock)

install(TARGETS ${INSTALL_TARGETS}
  LIBRARY DESTINATION ${MagicFlock_LIBS_DIR}
  ARCHIVE DESTINATION ${MagicFlock_LIBS_DIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

add_dependencies(MagicFlock il4mrc_headers)
