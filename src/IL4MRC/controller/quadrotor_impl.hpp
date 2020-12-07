#pragma once

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  Quadrotor()
{
  // Nothing to do here
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  init(unsigned int id,
       std::string name,
       std::string label,
       int number_of_quad,
       int num_of_antenna_src,
       std::vector<ignition::math::Vector3d> fix_antenna_positions,
       std::vector<Quadrotor<flight_controller_t,
                             FilterType,
                             NoiseType,
                             StateType,
                             ActionType>>& quad)
{
  id_ = id;
  name_ = name;
  label_ = label;
  num_neighbors_ = number_of_quad - 1;
  num_of_antenna_src_ = num_of_antenna_src;
  dataset_.init_dataset_directory();
  port_number_ = std::to_string(1454) + std::to_string(id);
  rssi_from_neighbors().resize(number_of_quad +
                               num_of_antenna_src_); // Max number of quad
  // Max number -1, dont count my position
  neighbor_positions().resize(num_neighbors_);
  neighbor_antenna_positions().resize(num_neighbors_);
  _fix_antenna_positions = fix_antenna_positions;
  neigh_antenna_dists_container().resize(num_neighbors_ + num_of_antenna_src_);
  neigh_angle_antenna_dists_container().resize(num_neighbors_);
  int num_of_antenna_transmitter = (num_neighbors_ + num_of_antenna_src_) * 2;
  arma::colvec initial_value(num_of_antenna_transmitter);
  initial_value.fill(-55);
  filter_.initial_value() = initial_value;
  filter_.reset();
  std::vector<Quadrotor<flight_controller_t,
                        FilterType,
                        NoiseType,
                        StateType,
                        ActionType>>& quad_ = quad;
  start_position_sampler(quad_);
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
unsigned int
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  id() const
{
  return id_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  name() const
{
  return name_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
double
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  height()
{
  return position().Z();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  port_number() const
{
  return port_number_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  port_number()
{
  return port_number_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  start_controller()
{
  controller_ = std::make_unique<flight_controller_t>("udp", port_number());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  start_flocking_model(const ignition::math::Vector4d& gains,
                       const ignition::math::Vector3d& destination,
                       const ignition::math::Vector3d& max_speed,
                       const bool& leader)
{
  std::lock_guard<std::mutex> lock(_flocking_mutex);

  flocking_sampler_.start(
    50, [&]() { flocking_model(gains, destination, max_speed, leader); });
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  flocking_model(const ignition::math::Vector4d& gains,
                 const ignition::math::Vector3d& destination,
                 const ignition::math::Vector3d& max_speed,
                 const bool& leader)
{
  Flocking flock(
    gains, position(), neighbor_positions(), destination, max_speed, leader);
  current_action().action() = flock.Velocity();
  current_action().one_hot_action() = flock.OneHotEncodingVelocity();
  all_actions().push_back(current_action());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  start_collision_detector(int duration)
{
  std::lock_guard<std::mutex> lock(_collision_mutex);

  collision_detector_sampler_.start(duration, [&]() {
    CollisionDetector cd(position(), neighbor_positions(), last_action());
    if (min_dist_ < 0.5)
      current_action_.action() = cd.Velocity();
  });
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  random_model(const ignition::math::Vector4d& axis_speed,
               const double& passed_time)
{
  std::lock_guard<std::mutex> lock(_random_mutex);
  current_action().action() = random.Velocity(axis_speed, passed_time);
  all_actions().push_back(current_action());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  stop_flocking_model()
{
  flocking_sampler_.stop();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  stop_collision_detector()
{
  collision_detector_sampler_.stop();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  start_position_sampler(std::vector<Quadrotor<flight_controller_t,
                                               FilterType,
                                               NoiseType,
                                               StateType,
                                               ActionType>>& quads)
{
  std::lock_guard<std::mutex> lock(_position_sampler_mutex);
  position_sampler_.start(50, [&]() {
    std::size_t j = 0;
    for (std::size_t i = 0; i < quads.size(); ++i) {
      if (id_ != quads.at(i).id()) {
        neighbor_positions().at(j) = quads.at(i).position();
        neighbor_antenna_positions().at(j) = quads.at(i).wt_antenna_position();
        // Add two antennas here positions
        // We need to sample the distance not only positions
        ++j;
      }
    }
  });
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  stop_position_sampler()
{
  position_sampler_.stop();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  start_sampling_rt_state(int interval)
{
  state_sampler_.start(interval, [this]() { sample_state(); });
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  stop_sampling_rt_state()
{
  state_sampler_.stop();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  sample_state()
{
  std::lock_guard<std::mutex> lock(_sample_state_mutex);

  if constexpr (std::is_same<StateType, ReceivedSignal>::value) {
    State<FilterType, NoiseType, StateType, std::vector<StateType>> state(
      id_, num_neighbors_, num_of_antenna_src_, rssi_from_neighbors(), filter_);
    current_state_ = state;
  } else if constexpr (std::is_same<StateType, AntennaDists>::value) {
    calculate_distances_to_neighbors_antenna();
    State<FilterType, NoiseType, StateType, std::vector<AntennaDists>> state(
      id_,
      num_neighbors_,
      num_of_antenna_src_,
      neigh_antenna_dists_container(),
      filter_);
    current_state_ = state;
  } else if constexpr (std::is_same<StateType, FullWiFi>::value) {
    calculate_angle_distances_to_neighbors_antenna();
    State<FilterType, NoiseType, StateType, std::vector<FullWiFi>> state(
      id_,
      num_neighbors_,
      num_of_antenna_src_,
      neigh_angle_antenna_dists_container(),
      filter_);
    current_state_ = state;
  }
  all_states_.push_back(current_state_);
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  sample_action(std::function<void()> execute_action)
{
  execute_action();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  sample_state_action_state(std::function<void(void)> action_to_execute,
                            std::function<void(void)> trajectory)
{
  sample_state();                   // s t-1
  sample_action(action_to_execute); // a t-1
  trajectory();                     // Execute the a t-1
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  sample_state();                   // s t
  sample_action(action_to_execute); // a t
  trajectory();                     // Execute the a t
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  // logger::logger_->info("Registering States, last state, Current state {}
  // {}",
  //                       last_state().Data(),
  //                       current_state().Data());
  // arma::colvec check_double = current_state().Data() - last_state().Data();
  // if (!check_double.is_zero()) {
  //   save_dataset_sasas();
  // }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_state() const
{
  std::lock_guard<std::mutex> lock(_current_state_mutex);
  return current_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_predicted_state()
{
  return current_predicted_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_predicted_state() const
{
  return current_predicted_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_predicted_enhanced_state()
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_predicted_enhanced_state() const
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<State<FilterType, NoiseType, StateType, std::vector<StateType>>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  all_states() const
{
  std::lock_guard<std::mutex> lock(_all_state_mutex);
  return all_states_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  last_state()
{
  if (all_states_.size() > 1) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 1);
    last_state_ = (*it_state);
  }
  return last_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  before_last_state()
{
  if (all_states_.size() > 2) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 2);
    before_last_state_ = (*it_state);
  }
  return before_last_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  before_2_last_state()
{
  if (all_states_.size() > 3) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 3);
    before_2_last_state_ = (*it_state);
  }
  return before_2_last_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
State<FilterType, NoiseType, StateType, std::vector<StateType>>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  before_3_last_state()
{
  if (all_states_.size() > 4) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 4);
    before_3_last_state_ = (*it_state);
  }
  return before_3_last_state_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  reset_all_states()
{
  all_states_.clear();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_loss(arma::vec loss_vector)
{
  loss_vector_ = loss_vector;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
arma::vec
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_loss() const
{
  return loss_vector_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  sample_action()
{
  ActionType action;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ActionType&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_action()
{
  std::lock_guard<std::mutex> lock(_current_action_mutex);
  return current_action_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  current_action() const
{
  std::lock_guard<std::mutex> lock(_current_action_mutex);
  return current_action_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  last_action()
{
  if (all_actions().size() > 1) {
    auto it_action = all_actions().rbegin();
    it_action = std::next(it_action, 1);
    last_action_ = (*it_action);
  }
  return last_action_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  before_last_action()
{
  if (all_actions().size() > 2) {
    auto it_action = all_actions().rbegin();
    it_action = std::next(it_action, 2);
    before_last_action_ = (*it_action);
  }
  return before_last_action_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  before_2_last_action()
{
  if (all_actions().size() > 3) {
    auto it_action = all_actions().rbegin();
    it_action = std::next(it_action, 3);
    before_2_last_action_ = (*it_action);
  }
  return before_2_last_action_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ActionType>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  all_actions() const
{
  std::lock_guard<std::mutex> lock(_all_action_mutex);
  return all_actions_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ActionType>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  all_actions()
{
  std::lock_guard<std::mutex> lock(_all_action_mutex);
  return all_actions_;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  reset_all_actions()
{
  std::lock_guard<std::mutex> lock(_all_action_mutex);
  all_actions_.clear();
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_state()
{
  dataset_.save_state(name_, vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_position()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_position_", position().X(), position().Y(), position().Z());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
template<typename Arg, typename... Args>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_values(std::string name, Arg value, Args... values)
{
  dataset_.save_csv_dataset_2_file(name_ + "_" + name, value, values...);
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_dataset_rssi_velocity()
{
  // see if it is possible to make current action generic
  dataset_.save_csv_dataset_2_file(name_,
                                   vec_.to_std_vector(current_state().Data()),
                                   current_action().one_hot_action());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_dataset_sasas()
{
  dataset_.save_csv_dataset_2_file(
    name_,
    vec_.to_std_vector(before_3_last_state().Data()),
    vec_.to_std_vector(before_2_last_action().Data()),
    vec_.to_std_vector(before_2_last_state().Data()),
    vec_.to_std_vector(before_last_action().Data()),
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(last_action().Data()),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(current_action().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_dataset_sasasp()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_current_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(last_action().Data()),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(current_action().Data()),
    vec_.to_std_vector(current_predicted_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_dataset_with_current_enhanced_predictions()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_enhanced_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(last_action().Data()),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(current_action().Data()),
    vec_.to_std_vector(current_predicted_enhanced_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_dataset_with_loss()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_loss",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(last_action().Data()),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(current_action().Data()),
    vec_.to_std_vector(current_loss()));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  save_histogram(int count)
{
  /*  Save a version of the time steps to create a histogram */
  /*  Increase one since count start from 0 */
  /*  Need to find a solution for this with a timer */
  count++;
  dataset_.save_controller_count(count);
  histo_.histogram(count);
  dataset_.save_histogram(histo_.get_histogram<int>());
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  position() const
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  position()
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neighbor_positions() const
{
  std::lock_guard<std::mutex> lock(_neighbor_positions_mutex);
  return _neighbor_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neighbor_positions()
{
  std::lock_guard<std::mutex> lock(_neighbor_positions_mutex);
  return _neighbor_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neighbor_antenna_positions() const
{
  std::lock_guard<std::mutex> lock(_neighbor_antenna_positions_mutex);
  return _neighbor_antenna_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neighbor_antenna_positions()
{
  std::lock_guard<std::mutex> lock(_neighbor_antenna_positions_mutex);
  return _neighbor_antenna_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  fix_antenna_positions() const
{
  std::lock_guard<std::mutex> lock(_fix_antenna_positions_mutex);
  return _fix_antenna_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<ignition::math::Vector3d>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  fix_antenna_positions()
{
  std::lock_guard<std::mutex> lock(_fix_antenna_positions_mutex);
  return _fix_antenna_positions;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_1_antenna_position() const
{
  std::lock_guard<std::mutex> lock(_wr_1_position_mutex);
  return _wr_1_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_1_antenna_position()
{
  std::lock_guard<std::mutex> lock(_wr_1_position_mutex);
  return _wr_1_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_2_antenna_position() const
{
  std::lock_guard<std::mutex> lock(_wr_2_position_mutex);
  return _wr_2_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_2_antenna_position()
{
  std::lock_guard<std::mutex> lock(_wr_2_position_mutex);
  return _wr_2_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wt_antenna_position() const
{
  std::lock_guard<std::mutex> lock(_wt_position_mutex);
  return _wt_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wt_antenna_position()
{
  std::lock_guard<std::mutex> lock(_wt_position_mutex);
  return _wt_position;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Quaternion<double>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  orientation() const
{
  std::lock_guard<std::mutex> lock(_orientation_mutex);
  return _orientation;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
ignition::math::Quaternion<double>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  orientation()
{
  std::lock_guard<std::mutex> lock(_orientation_mutex);
  return _orientation;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<StateType>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  rssi_from_neighbors() const
{
  std::lock_guard<std::mutex> lock(_rssi_from_neighbors_mutex);
  return _rssi_from_neighbors;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
LaserScan&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  laser_scan()
{
  std::lock_guard<std::mutex> lock(_laser_scan_mutex);
  return _laser_scan;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
LaserScan
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  laser_scan() const
{
  std::lock_guard<std::mutex> lock(_laser_scan_mutex);
  return _laser_scan;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<StateType>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  rssi_from_neighbors()
{
  std::lock_guard<std::mutex> lock(_rssi_from_neighbors_mutex);
  return _rssi_from_neighbors;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<AntennaDists>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neigh_antenna_dists_container() const
{
  std::lock_guard<std::mutex> lock(_neigh_antenna_dists_container_mutex);
  return _neigh_antenna_dists_container;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<AntennaDists>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neigh_antenna_dists_container()
{
  std::lock_guard<std::mutex> lock(_neigh_antenna_dists_container_mutex);
  return _neigh_antenna_dists_container;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<FullWiFi>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neigh_angle_antenna_dists_container() const
{
  std::lock_guard<std::mutex> lock(_neigh_angle_antenna_dists_container_mutex);
  return _neigh_angle_antenna_dists_container;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<FullWiFi>&
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  neigh_angle_antenna_dists_container()
{
  std::lock_guard<std::mutex> lock(_neigh_angle_antenna_dists_container_mutex);
  return _neigh_angle_antenna_dists_container;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::vector<double>
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  distances_to_neighbors()
{
  std::vector<double> distances(neighbor_positions().size());
  for (std::size_t i = 0; i < neighbor_positions().size(); ++i) {
    distances.at(i) = position().Distance(neighbor_positions().at(i));
  }
  return distances;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  calculate_distances_to_neighbors_antenna()
{
  for (std::size_t i = 0; i < neighbor_antenna_positions().size(); ++i) {
    neigh_antenna_dists_container().at(i).dist_antenna_1 =
      wr_1_antenna_position().Distance(neighbor_antenna_positions().at(i));
    neigh_antenna_dists_container().at(i).dist_antenna_2 =
      wr_2_antenna_position().Distance(neighbor_antenna_positions().at(i));
  }
  int j = neighbor_antenna_positions().size();
  for (std::size_t i = 0; i < fix_antenna_positions().size(); ++i) {
    neigh_antenna_dists_container().at(j).dist_antenna_1 =
      wr_1_antenna_position().Distance(fix_antenna_positions().at(i));
    neigh_antenna_dists_container().at(j).dist_antenna_2 =
      wr_2_antenna_position().Distance(fix_antenna_positions().at(i));
    j++;
  }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  calculate_angle_distances_to_neighbors_antenna()
{
  for (std::size_t i = 0; i < neighbor_positions().size(); ++i) {
    neigh_angle_antenna_dists_container().at(i).antenna =
      position().Distance(neighbor_positions().at(i));
    neigh_angle_antenna_dists_container().at(i).azimuth =
      aoa_azimuth(neighbor_positions().at(i));
    neigh_angle_antenna_dists_container().at(i).elevation =
      aoa_elevation(neighbor_positions().at(i));
  }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
double
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  distance_to(int id)
{
  double distance = distances_to_neighbors().at(id);
  return distance;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wireless_receiver_1_topic_name()
{
  std::string topic_name =
    "/gazebo/default/" + name_ + "/WR_1/Wireless Receiver/transceiver";
  return topic_name;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wireless_receiver_2_topic_name()
{
  std::string topic_name =
    "/gazebo/default/" + name_ + "/WR_2/Wireless Receiver/transceiver";
  return topic_name;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  laser_scaner_topic_name()
{
  std::string topic_name = "/gazebo/default/" + name() + "/LaserScan/scan";
  return topic_name;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wireless_transmitter_topic_name()
{
  return "/gazebo/default/" + name() + "/WT/Wireless Transmitter/transceiver";
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wt_name()
{
  return name() + "::WT";
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_1_name()
{
  return name() + "::WR_1";
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  wr_2_name()
{
  return name() + "::WR_2";
}

/*  Parsing the ReceivedSignal send by Gazebo */
template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  RxMsgN1(const ConstWirelessNodesPtr& _msg)
{
  if constexpr (std::is_same<StateType, ReceivedSignal>::value) {
    std::lock_guard<std::mutex> lock(_rx_1_mutex);
    int numRxNodes = _msg->node_size();
    for (int i = 0; i < numRxNodes; ++i) {
      gazebo::msgs::WirelessNode RxNode = _msg->node(i);
      const char essid = RxNode.essid().back();
      int id = (int(essid) - 48);
      rssi_from_neighbors().at(i).id = id;
      rssi_from_neighbors().at(i).name = RxNode.essid();
      rssi_from_neighbors().at(i).antenna_1 = RxNode.signal_level();
    }
  }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  RxMsgN2(const ConstWirelessNodesPtr& _msg)
{
  if constexpr (std::is_same<StateType, ReceivedSignal>::value) {
    std::lock_guard<std::mutex> lock(_rx_2_mutex);
    int numRxNodes = _msg->node_size();
    for (int i = 0; i < numRxNodes; ++i) {
      gazebo::msgs::WirelessNode RxNode = _msg->node(i);
      const char essid = RxNode.essid().back();
      int id = (int(essid) - 48);
      rssi_from_neighbors().at(i).id = id;
      rssi_from_neighbors().at(i).name = RxNode.essid();
      rssi_from_neighbors().at(i).antenna_2 = RxNode.signal_level();
    }
  }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  subRxTopic(Quadrotor<flight_controller_t,
                       FilterType,
                       NoiseType,
                       StateType,
                       ActionType>::NodePtr& node)
{
  std::string topic_WR_1 = wireless_receiver_1_topic_name();
  subs_.push_back(node->Subscribe(topic_WR_1,
                                  &Quadrotor<flight_controller_t,
                                             FilterType,
                                             NoiseType,
                                             StateType,
                                             ActionType>::RxMsgN1,
                                  this));

  std::string topic_WR_2 = wireless_receiver_2_topic_name();
  subs_.push_back(node->Subscribe(topic_WR_2,
                                  &Quadrotor<flight_controller_t,
                                             FilterType,
                                             NoiseType,
                                             StateType,
                                             ActionType>::RxMsgN2,
                                  this));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  LaserScanMsg(const ConstLaserScanStampedPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_laser_mutex);
  gazebo::msgs::LaserScan msg = _msg->scan();
  laser_scan().ranges.resize(msg.ranges_size());
  laser_scan().intensities.resize(msg.intensities_size());
  logger::logger_->info("Range size {}", msg.ranges_size());
  logger::logger_->info("Intensities size {}", msg.intensities_size());
  for (int i = 0; i < msg.ranges_size(); ++i) {
    laser_scan().ranges.at(i) = msg.ranges(i);
  }
  for (int i = 0; i < msg.intensities_size(); ++i) {
    laser_scan().intensities.at(i) = msg.intensities(i);
  }
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
void
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  LaserScanTopic(Quadrotor<flight_controller_t,
                           FilterType,
                           NoiseType,
                           StateType,
                           ActionType>::NodePtr& node)
{
  std::string topic_laser = laser_scaner_topic_name();
  subs_.push_back(node->Subscribe(topic_laser,
                                  &Quadrotor<flight_controller_t,
                                             FilterType,
                                             NoiseType,
                                             StateType,
                                             ActionType>::LaserScanMsg,
                                  this));
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
double
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  aoa_azimuth(ignition::math::Vector3d neighbor_position)
{
  double y = neighbor_position.Y() - position().Y();
  double x = neighbor_position.X() - position().X();

  double azimuth = std::atan2(y, x) * 180 / PI;
  return azimuth;
}

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
double
Quadrotor<flight_controller_t, FilterType, NoiseType, StateType, ActionType>::
  aoa_elevation(ignition::math::Vector3d neighbor_position)
{
  double z = neighbor_position.Z() - height();
  double distance = position().Distance(neighbor_position);
  double elevation = std::asin(z/distance) * 180 / PI;
  return elevation;
}
