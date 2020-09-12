set(SOURCES
  px4_device.hpp
  px4_device.cc
  quadrotor.hpp
  quadrotor_impl.hpp
  swarm_device.hpp
  swarm_device_impl.hpp
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(XX_SRCS ${XX_SRCS} ${DIR_SRCS} PARENT_SCOPE)

