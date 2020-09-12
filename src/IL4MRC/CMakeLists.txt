set(SOURCES
  action_generator.hpp
  action_generator_impl.hpp
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
  arma_helper.hpp
  arma_helper_impl.hpp
  check_destination.hpp
  check_destination_impl.hpp
  check_swarm_shape.hpp
  check_swarm_shape_impl.hpp
  CLI11.hpp
  continuous_actions.hpp
  dataset.hpp
  dataset_impl.hpp
  discret_actions.hpp
  evaluate_model.hpp
  exponential_moving_average.hpp
  exponential_moving_average_impl.hpp
  flocking.hpp
  gaussian_noise.hpp
  gaussian_noise_impl.hpp
  gazebo.hpp
  gazebo_impl.hpp
  histogram.hpp
  histogram_impl.hpp
  joystick.hpp
  keyboard.hpp
  knn_predictor.hpp
  knn_predictor_impl.hpp
  logger.hpp
  nearest_neighbors.hpp
  nearest_neighbors_impl.hpp
  one_hot_encoding.hpp
  one_hot_encoding_impl.hpp
  position_gps.hpp
  propagation_model.hpp
  propagation_model_impl.hpp
  px4_device.hpp
  quadrotor.hpp
  quadrotor_impl.hpp
  real_time_samples.hpp
  state.hpp
  state_impl.hpp
  swarm_device.hpp
  swarm_device_impl.hpp
  time.hpp
  time_steps.hpp
  Vector.hpp
  Vector_impl.hpp
)


set(DIR_SRCS)
foreach(file ${SOURCES})
  set(DIR_SRCS ${DIR_SRCS} ${CMAKE_CURRENT_SOURCE_DIR}/${file})
endforeach()

set(XX_SRCS ${XX_SRCS} ${DIR_SRCS} PARENT_SCOPE)

