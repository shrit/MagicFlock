set(SOURCES
  check_destination.hpp
  check_destination_impl.hpp
  check_swarm_shape.hpp
  check_swarm_shape_impl.hpp
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(IL4MRC_SRCS ${IL4MRC_SRCS} ${DIR_SRCS} PARENT_SCOPE)

