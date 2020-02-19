#pragma once

template<class simulator_t>
Quadrotor<simulator_t>::Quadrotor(unsigned int id,
                                  std::string name,
                                  std::shared_ptr<simulator_t> sim_interface)
  : id_(id)
  , name_(name)
  , sim_interface_(sim_interface)
{
  data_set_.init_dataset_directory();
  rt_samples_ = std::make_shared<RTSamples>();
}

template<class simulator_t>
unsigned int
Quadrotor<simulator_t>::id() const
{
  return id_;
}

template<class simulator_t>
std::string
Quadrotor<simulator_t>::name() const
{
  return name_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::add_nearest_neighbor_id(unsigned int id)
{
  nearest_neighbors_.push_back(id);
}

template<class simulator_t>
std::vector<unsigned int>
Quadrotor<simulator_t>::nearest_neighbors() const
{
  return nearest_neighbors_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::start_sampling_rt_state(int interval)
{
  rt_samples_->start(interval, [this]() {
    State<simulator_t> state(sim_interface_, id_, nearest_neighbors_);
    current_state_ = state;
    all_states_.push_back(state);
  });
}

template<class simulator_t>
void
Quadrotor<simulator_t>::stop_sampling_rt_state()
{
  rt_samples_->stop();
}

template<class simulator_t>
void
Quadrotor<simulator_t>::sample_state()
{
  State<simulator_t> state(sim_interface_, id_, nearest_neighbors_);
  current_state_ = state;
  all_states_.push_back(state);
}

template<class simulator_t>
State<simulator_t>
Quadrotor<simulator_t>::current_state() const
{
  return current_state_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::current_predicted_state(
  arma::vec current_predicted_state)
{
  current_predicted_state_ = current_predicted_state;
}

template<class simulator_t>
arma::vec
Quadrotor<simulator_t>::current_predicted_state() const
{
  return current_predicted_state_;
}

template<class simulator_t>
std::vector<State<simulator_t>>
Quadrotor<simulator_t>::all_states() const
{
  return all_states_;
}

template<class simulator_t>
State<simulator_t>
Quadrotor<simulator_t>::last_state()
{
  if (all_states_.size() > 1) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 1);
    last_state_ = (*it_state);
  }
  return last_state_;
}

template<class simulator_t>
State<simulator_t>
Quadrotor<simulator_t>::before_last_state()
{
  if (all_states_.size() > 2) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 2);
    before_last_state_ = (*it_state);
  }
  return before_last_state_;
}

template<class simulator_t>
State<simulator_t>
Quadrotor<simulator_t>::before_2_last_state()
{
  if (all_states_.size() > 3) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 3);
    before_2_last_state_ = (*it_state);
  }
  return before_2_last_state_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::reset_all_states()
{
  all_states_.clear();
}

template<class simulator_t>
void
Quadrotor<simulator_t>::current_loss(arma::vec loss_vector)
{
  loss_vector_ = loss_vector;
}

template<class simulator_t>
arma::vec
Quadrotor<simulator_t>::current_loss() const
{
   return loss_vector_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::current_action(Actions::Action action)
{
  all_actions_.push_back(action);
  current_action_ = action;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::current_action() const
{
  return current_action_;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::last_action()
{
  if (all_actions_.size() > 1) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 1);
    last_action_ = (*it_action);
  }
  return last_action_;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::before_last_action()
{
  if (all_actions_.size() > 2) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 2);
    before_last_action_ = (*it_action);
  }
  return before_last_action_;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::before_2_last_action()
{
  if (all_actions_.size() > 3) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 3);
    before_2_last_action_ = (*it_action);
  }
  return before_2_last_action_;
}

template<class simulator_t>
std::vector<Actions::Action>
Quadrotor<simulator_t>::all_actions() const
{
  return all_actions_;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::most_used_action()
{
  std::vector<double> most_used_action(7);
  most_used_action.at(0) = std::count(
    all_actions_.begin(), all_actions_.end(), Actions::Action::forward);
  most_used_action.at(1) = std::count(
    all_actions_.begin(), all_actions_.end(), Actions::Action::backward);
  most_used_action.at(2) =
    std::count(all_actions_.begin(), all_actions_.end(), Actions::Action::left);
  most_used_action.at(3) = std::count(
    all_actions_.begin(), all_actions_.end(), Actions::Action::right);
  most_used_action.at(4) =
    std::count(all_actions_.begin(), all_actions_.end(), Actions::Action::up);
  most_used_action.at(5) =
    std::count(all_actions_.begin(), all_actions_.end(), Actions::Action::down);
  most_used_action.at(6) = std::count(
    all_actions_.begin(), all_actions_.end(), Actions::Action::NoMove);
  Actions action;
  logger::logger_->info("most used action {}", most_used_action);
  Actions::Action most_used =
    action.int_to_action(mtools_.index_of_max_value(most_used_action));
  return most_used;
}

template<class simulator_t>
Actions::Action
Quadrotor<simulator_t>::most_frequent_action_in_container()
{
  std::vector<double> most_used_action(7);
  most_used_action.at(0) = std::count(action_container_.begin(),
                                      action_container_.end(),
                                      Actions::Action::forward);
  most_used_action.at(1) = std::count(action_container_.begin(),
                                      action_container_.end(),
                                      Actions::Action::backward);
  most_used_action.at(2) = std::count(
    action_container_.begin(), action_container_.end(), Actions::Action::left);
  most_used_action.at(3) = std::count(
    action_container_.begin(), action_container_.end(), Actions::Action::right);
  most_used_action.at(4) = std::count(
    action_container_.begin(), action_container_.end(), Actions::Action::up);
  most_used_action.at(5) = std::count(
    action_container_.begin(), action_container_.end(), Actions::Action::down);
  most_used_action.at(6) = std::count(action_container_.begin(),
                                      action_container_.end(),
                                      Actions::Action::NoMove);
  Actions action;
  logger::logger_->info("most used action {}", most_used_action);
  Actions::Action most_used =
    action.int_to_action(mtools_.index_of_max_value(most_used_action));
  return most_used;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::save_action_in_container(Actions::Action action)
{
  action_container_.push_back(action);
}

template<class simulator_t>
void
Quadrotor<simulator_t>::reset_all_actions()
{
  all_actions_.clear();
}

template<class simulator_t>
void
Quadrotor<simulator_t>::register_data_set()
{
  data_set_.save_csv_data_set_2_file(
    name_,
    before_last_state(),
    mtools_.to_one_hot_encoding(last_action(), 7),
    last_state(),
    mtools_.to_one_hot_encoding(current_action(), 7),
    current_state());
}

template<class simulator_t>
void
Quadrotor<simulator_t>::register_data_set_with_current_predictions()
{
  data_set_.save_csv_data_set_2_file(
    name_ + "_current_predictions",
    before_last_state(),
    mtools_.to_one_hot_encoding(last_action(), 7),
    last_state(),
    mtools_.to_one_hot_encoding(current_action(), 7),
    mtools_.to_std_vector(current_predicted_state()),
    current_state());
}

template<class simulator_t>
void
Quadrotor<simulator_t>::register_data_set_with_loss()
{
  data_set_.save_csv_data_set_2_file(
    name_ + "_loss",
    before_last_state(),
    mtools_.to_one_hot_encoding(last_action(), 7),
    last_state(),
    mtools_.to_one_hot_encoding(current_action(), 7),
    mtools_.to_std_vector(current_loss()));
}

template<class simulator_t>
void
Quadrotor<simulator_t>::register_histogram(int count)
{
  /*  Save a version of the time steps to create a histogram */
  /*  Increase one since count start from 0 */
  /*  Need to find a solution for this with a timer */
  count++;
  data_set_.save_controller_count(count);
  mtools_.histogram(count);
  data_set_.save_histogram(mtools_.get_histogram<int>());
}

template<class simulator_t>
bool
Quadrotor<simulator_t>::examin_geometric_shape()
{
  bool shape =
    mtools_.is_good_shape(id_, nearest_neighbors_, sim_interface_->positions());
  return shape;
}

template<class simulator_t>
double
Quadrotor<simulator_t>::height()
{
  return sim_interface_->positions().at(id_).z;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::set_speed(double speed)
{
  speed_ = speed;
}

template<class simulator_t>
double
Quadrotor<simulator_t>::speed() const
{
  return speed_;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::increase_speed()
{
  speed_++;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::decrease_speed()
{
  speed_--;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::increase_speed_by_value(double speed)
{
  speed_ = speed_ + speed;
}

template<class simulator_t>
void
Quadrotor<simulator_t>::decrease_speed_by_value(double speed)
{
  speed_ = speed_ - speed;
}

template<class simulator_t>
std::vector<double>
Quadrotor<simulator_t>::distances_to_neighbors()
{
  std::vector<double> distances;
  std::map<unsigned int, double> neigh_distances;
  neigh_distances = mtools_.distances_to_neighbors(
    id_, nearest_neighbors_, sim_interface_->positions());
  distances = mtools_.map_to_vector(neigh_distances);
  return distances;
}
