set(SOURCES
  dataset.hpp
  dataset_impl.hpp
)


set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MagicFlock_SRCS ${MagicFlock_SRCS} ${DIR_SRCS} PARENT_SCOPE)

