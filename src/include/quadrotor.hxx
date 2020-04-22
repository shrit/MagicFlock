#pragma once

template<class flight_controller_t, class simulator_t, class NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::Quadrotor(
  flight_controller_t,
  unsigned int id,
  std::string name,
  std::shared_ptr<simulator_t> sim_interface)
  : id_(id)
  , name_(name)
  , sim_interface_(sim_interface)
{
  data_set_.init_dataset_directory();
  rt_samples_ = std::make_shared<RTSamples>();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
unsigned int
Quadrotor<flight_controller_t, simulator_t, NoiseType>::id() const
{
  return id_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::string
Quadrotor<flight_controller_t, simulator_t, NoiseType>::name() const
{
  return name_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::add_nearest_neighbor_id(
  unsigned int id)
{
  nearest_neighbors_.push_back(id);
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::vector<unsigned int>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::nearest_neighbors()
  const
{
  return nearest_neighbors_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::start_sampling_rt_state(
  int interval)
{
  rt_samples_->start(interval, [this]() {
    State<simulator_t, NoiseType> state(
      sim_interface_, id_, nearest_neighbors_);
    current_state_ = state;
    all_states_.push_back(state);
  });
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::stop_sampling_rt_state()
{
  rt_samples_->stop();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::sample_state()
{
  State<simulator_t, NoiseType> state(sim_interface_, id_, nearest_neighbors_);
  current_state_ = state;
  all_states_.push_back(state);

  data_set_.plot_distance_to_neighbor(0,
                                      "Distance to first leader",
                                      "Time Step",
                                      "Distance",
                                      name_ + "_distance_to_0",
                                      "lines",
                                      all_states_);

  data_set_.plot_distance_to_neighbor(2,
                                      "Distance to second leader",
                                      "Time Step",
                                      "Distance",
                                      name_ + "_distance_to_2",
                                      "lines",
                                      all_states_);
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::current_state() const
{
  return current_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>&
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  current_predicted_state()
{
  return current_predicted_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  current_predicted_state() const
{
  return current_predicted_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>&
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  current_predicted_enhanced_state()
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  current_predicted_enhanced_state() const
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::vector<State<simulator_t, NoiseType>>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::all_states() const
{
  return all_states_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::last_state()
{
  if (all_states_.size() > 1) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 1);
    last_state_ = (*it_state);
  }
  return last_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::before_last_state()
{
  if (all_states_.size() > 2) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 2);
    before_last_state_ = (*it_state);
  }
  return before_last_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
State<simulator_t, NoiseType>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::before_2_last_state()
{
  if (all_states_.size() > 3) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 3);
    before_2_last_state_ = (*it_state);
  }
  return before_2_last_state_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::reset_all_states()
{
  all_states_.clear();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::current_loss(
  arma::vec loss_vector)
{
  loss_vector_ = loss_vector;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
arma::vec
Quadrotor<flight_controller_t, simulator_t, NoiseType>::current_loss() const
{
  return loss_vector_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::current_action(
  Actions::Action action)
{
  all_actions_.push_back(action);
  current_action_ = action;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
Actions::Action
Quadrotor<flight_controller_t, simulator_t, NoiseType>::current_action() const
{
  return current_action_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
Actions::Action
Quadrotor<flight_controller_t, simulator_t, NoiseType>::last_action()
{
  if (all_actions_.size() > 1) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 1);
    last_action_ = (*it_action);
  }
  return last_action_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
Actions::Action
Quadrotor<flight_controller_t, simulator_t, NoiseType>::before_last_action()
{
  if (all_actions_.size() > 2) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 2);
    before_last_action_ = (*it_action);
  }
  return before_last_action_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
Actions::Action
Quadrotor<flight_controller_t, simulator_t, NoiseType>::before_2_last_action()
{
  if (all_actions_.size() > 3) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 3);
    before_2_last_action_ = (*it_action);
  }
  return before_2_last_action_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::vector<Actions::Action>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::all_actions() const
{
  return all_actions_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::reset_all_actions()
{
  all_actions_.clear();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::register_state()
{
  data_set_.save_state(name_, vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::register_data_set()
{
  data_set_.save_csv_data_set_2_file(
    name_,
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  register_data_set_with_current_predictions()
{
  data_set_.save_csv_data_set_2_file(
    name_ + "_current_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_predicted_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  register_data_set_with_current_enhanced_predictions()
{
  data_set_.save_csv_data_set_2_file(
    name_ + "_enhanced_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_predicted_enhanced_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  register_data_set_with_loss()
{
  data_set_.save_csv_data_set_2_file(
    name_ + "_loss",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_loss()));
}

template<class flight_controller_t, class simulator_t, class NoiseType>
template<class flight_controller_t, typename Arg, typename... Args>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::register_loss(
  Arg arg,
  Args... args)
{
  data_set_.save_error_file(name_, arg, args...);
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::register_histogram(
  int count)
{
  /*  Save a version of the time steps to create a histogram */
  /*  Increase one since count start from 0 */
  /*  Need to find a solution for this with a timer */
  count++;
  data_set_.save_controller_count(count);
  histo_.histogram(count);
  data_set_.save_histogram(histo_.get_histogram<int>());
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  register_actions_evaluation(Actions::Action first_action,
                              Actions::Action second_action)
{
  if (first_action == second_action) {
    data_set_.save_actions(name_, 1);
  } else {
    data_set_.save_actions(name_, -1);
  }
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::register_episodes(
  int n_episodes)
{
  data_set_.save_episodes(n_episodes);
}

template<class flight_controller_t, class simulator_t, class NoiseType>
bool
Quadrotor<flight_controller_t, simulator_t, NoiseType>::examin_geometric_shape()
{
  bool shape =
    shape_.is_good_shape(id_, nearest_neighbors_, sim_interface_->positions());
  return shape;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
double
Quadrotor<flight_controller_t, simulator_t, NoiseType>::height()
{
  return sim_interface_->positions().at(id_).Z();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, simulator_t, NoiseType>::position()
{
  return sim_interface_->positions().at(id);
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::vector<ignition::math::Vector3d>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::position_of_neighbors()
{
  return sim_interface_->positions();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
double&
Quadrotor<flight_controller_t, simulator_t, NoiseType>::speed()
{
  return speed_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
double
Quadrotor<flight_controller_t, simulator_t, NoiseType>::speed() const
{
  return speed_;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::increase_speed()
{
  speed_++;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::decrease_speed()
{
  speed_--;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::increase_speed_by_value(
  double speed)
{
  speed_ = speed_ + speed;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::decrease_speed_by_value(
  double speed)
{
  speed_ = speed_ - speed;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
void
Quadrotor<flight_controller_t, simulator_t, NoiseType>::reset_models()
{
  sim_interface_->reset_models();
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::vector<double>
Quadrotor<flight_controller_t, simulator_t, NoiseType>::distances_to_neighbors()
{
  std::vector<double> distances;
  std::map<unsigned int, double> neigh_distances;
  neigh_distances = dist_.distances_to_neighbors(
    id_, nearest_neighbors_, sim_interface_->positions());
  distances = vec_.map_to_vector(neigh_distances);
  return distances;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
double
Quadrotor<flight_controller_t, simulator_t, NoiseType>::distance_to(int id)
{
  double distance = distances_to_neighbors().at(id);
  return distance;
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::string
Quadrotor<flight_controller_t, simulator_t, NoiseType>::reset_topic_name()
{
  return "/gazebo/default/iris_1/model_reset";
}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::string
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  wireless_receiver_topic_name()
{}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::string
Quadrotor<flight_controller_t, simulator_t, NoiseType>::
  wireless_transmitter_topic_name()
{}

template<class flight_controller_t, class simulator_t, class NoiseType>
std::string
Quadrotor<flight_controller_t, simulator_t, NoiseType>::port_number()
{
  // 1454 + i number of quadroto
}

// positions = /gazebo/default/pose/info
