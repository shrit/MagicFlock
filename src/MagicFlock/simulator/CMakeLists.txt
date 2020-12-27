set(SOURCES
  gazebo.hpp
  gazebo_impl.hpp
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

add_subdirectory(gazebo)

set(MagicFlock_SRCS ${MagicFlock_SRCS} ${DIR_SRCS} PARENT_SCOPE)

