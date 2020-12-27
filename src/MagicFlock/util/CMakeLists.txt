set(SOURCES
  arma_helper.hpp
  arma_helper_impl.hpp
  logger.hpp
  logger.cc
  position_gps.hpp
  propagation_model.hpp
  propagation_model_impl.hpp
  real_time_samples.hpp
  real_time_samples.cc
  scheduler.hpp
  time.hpp
  time.cc
  time_steps.hpp
  time_steps.cc
  Vector.hpp
  Vector_impl.hpp
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(MagicFlock_SRCS ${MagicFlock_SRCS} ${DIR_SRCS} PARENT_SCOPE)

