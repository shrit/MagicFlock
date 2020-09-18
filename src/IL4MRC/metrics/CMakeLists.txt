set(SOURCES
  evaluate_model.hpp
  evaluate_model.cc
  exponential_moving_average.hpp
  exponential_moving_average_impl.hpp
  histogram.hpp
  histogram_impl.hpp
  max_distance.hpp
  max_distance_impl.hpp
  min_distance.hpp
  min_distance_impl.hpp 
  one_hot_encoding.hpp
  one_hot_encoding_impl.hpp
)


set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(IL4MRC_SRCS ${IL4MRC_SRCS} ${DIR_SRCS} PARENT_SCOPE)

