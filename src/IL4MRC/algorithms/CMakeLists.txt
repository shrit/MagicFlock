set(SOURCES
  ann_action_predictor.hpp
  ann_action_predictor_impl.hpp
  ann_enhanced_predictor.hpp
  ann_enhanced_predictor_impl.hpp
  ann_error_predictor.hpp
  ann_error_predictor_impl.hpp
  ann_predictor.hpp
  ann_predictor_impl.hpp
  ann_state_predictor.hpp
  ann_state_predictor_impl.hpp
  argmin.hpp
  argmin_impl.hpp
  knn_predictor.hpp
  knn_predictor_impl.hpp
)

set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(XX_SRCS ${XX_SRCS} ${DIR_SRCS} PARENT_SCOPE)

