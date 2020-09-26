set(DIRS
  flocking
  random
)

foreach(dir ${DIRS})
  add_subdirectory(${dir})
endforeach()

set(IL4MRC_SRCS ${IL4MRC_SRCS} ${DIR_SRCS} PARENT_SCOPE)
