set(DIRS
  actions
  algorithms
  check
  controller
  data
  dists
  flocking
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

add_library(IL4MRC ${IL4MRC_SRCS})

file(GLOB_RECURSE IL4MRC_HEADER_LIST_H RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.h)
file(GLOB_RECURSE IL4MRC_HEADER_LIST_HPP RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)
# file(GLOB PROTO_HEADER_LIST CONFIGURE_DEPENDS 
# "${CMAKE_CURRENT_BINARY_DIR}/src/gazebo/*.h")
set(INCLUDE_FILES ${IL4MRC_HEADER_LIST_HPP})

add_custom_target(il4mrc_headers)
add_custom_command(TARGET il4mrc_headers POST_BUILD
  COMMENT "Moving header files to include/IL4MRC/"
  COMMAND ${CMAKE_COMMAND} ARGS -E
    make_directory ${CMAKE_BINARY_DIR}/include/IL4MRC/
)

# Then copy each of the header files over to that directory.
foreach(incl_file ${INCLUDE_FILES})
  add_custom_command(TARGET il4mrc_headers POST_BUILD
    COMMAND ${CMAKE_COMMAND} ARGS -E
      copy ${CMAKE_CURRENT_SOURCE_DIR}/${incl_file}
           ${CMAKE_BINARY_DIR}/include/IL4MRC/${incl_file}
      BYPRODUCTS ${CMAKE_BINARY_DIR}/include/IL4MRC/${incl_file})
endforeach()

target_link_libraries(IL4MRC
  ${GAZEBO_LIBRARIES}
  ${ARMADILLO_LIBRARIES}
  ${BOOST_LIBRARIES}
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

# target_include_directories(ILM4RC PRIVATE 
#   ${CMAKE_CURRENT_BINARY_DIR}/src/gazebo
# )

install(DIRECTORY "${CMAKE_BINARY_DIR}/include/IL4MRC" DESTINATION
    "${CMAKE_INSTALL_INCLUDEDIR}")

set(INSTALL_TARGETS IL4MRC)

install(TARGETS ${INSTALL_TARGETS}
  LIBRARY DESTINATION ${IL4MRC_LIBS_DIR}
  ARCHIVE DESTINATION ${IL4MRC_LIBS_DIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
