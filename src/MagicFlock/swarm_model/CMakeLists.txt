set(DIRS
  collision_detector
  flocking
  random
)

foreach(dir ${DIRS})
  add_subdirectory(${dir})
endforeach()

set(MagicFlock_SRCS ${MagicFlock_SRCS} ${DIR_SRCS} PARENT_SCOPE)
