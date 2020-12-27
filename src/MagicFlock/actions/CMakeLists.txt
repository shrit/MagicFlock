set(SOURCES
  action_generator.hpp
  action_generator_impl.hpp
  continuous_actions.hpp
  continuous_actions.cc
  discret_actions.hpp
  discret_actions.cc
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MagicFlock_SRCS ${MagicFlock_SRCS} ${DIR_SRCS} PARENT_SCOPE)

