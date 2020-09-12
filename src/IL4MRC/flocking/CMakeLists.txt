set(SOURCES
  flocking.hpp
  flocking.cc
  olfati_saber.hpp
  olfati_saber.cc 
)


set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(XX_SRCS ${XX_SRCS} ${DIR_SRCS} PARENT_SCOPE)

