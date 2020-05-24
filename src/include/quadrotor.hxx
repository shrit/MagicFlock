#pragma once

template<class flight_controller_t, class FilterType, class ActionType>
Quadrotor<flight_controller_t, FilterType, ActionType>::Quadrotor()
{}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::init(unsigned int id,
                                                             std::string name,
                                                             std::string label)
{
  id_ = id;
  name_ = name;
  label_ = label;
  NodePtr node(new gazebo::transport::Node());
  node_ = node;
  dataset_.init_dataset_directory();
  node_->Init();
  state_sampler_ = std::make_shared<RTSamples>();
  neighbor_sampler_ = std::make_shared<RTSamples>();
  flocking_sampler_ = std::make_shared<RTSamples>();
  port_number_ = std::to_string(1454) + std::to_string(id);
}

/* Check this function it is cases segfault*/
template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::make_reference_2_swarm(
  std::vector<Quadrotor<flight_controller_t, FilterType, ActionType>> quads)
{
  for (int i = 0; i < quads.size(); ++i) {
    if (quads.at(i)->id() == this->id()) {
      quads.erase(quads.begin() + i); // needs verifications
    }
  }
  quads_ = quads;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::start_controller()
{
  controller_ = std::make_unique<flight_controller_t>("udp", port_number());
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, ActionType>::start_flocking(
  double sepGain,
  double cohGain,
  double migGain,
  double cutoffDist)
{
  flocking_sampler_->start(50, [&]() {
    Flocking flock(sepGain, cohGain, migGain, cutoffDist, position());
    flock.Velocity();
  });
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::stop_flocking()
{
  flocking_sampler_->stop();
}

template<class flight_controller_t, class FilterType, class ActionType>
unsigned int
Quadrotor<flight_controller_t, FilterType, ActionType>::id() const
{
  return id_;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::name() const
{
  return name_;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::
  start_nearest_neighbor_detector()
{
  neighbor_sampler_->start(50, [this]() {
    NearestNeighbors<RSSI> nn(_rssi_from_neighbors);
    _nearest_neighbors = nn.search();
  });
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::
  stop_nearest_neighbor_detector()
{
  neighbor_sampler_->stop();
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<unsigned int>
Quadrotor<flight_controller_t, FilterType, ActionType>::nearest_neighbors()
  const
{
  return _nearest_neighbors;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::start_sampling_rt_state(
  int interval)
{
  state_sampler_->start(interval, [this]() {
    State<FilterType, std::vector<RSSI>> state(id_, nearest_neighbors());
    current_state_ = state;
    all_states_.push_back(state);
  });
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::stop_sampling_rt_state()
{
  state_sampler_->stop();
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::sample_state()
{
  State<FilterType, std::vector<RSSI>> state(id_, nearest_neighbors());
  current_state_ = state;
  all_states_.push_back(state);
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::current_state() const
{
  return current_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>&
Quadrotor<flight_controller_t, FilterType, ActionType>::
  current_predicted_state()
{
  return current_predicted_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::
  current_predicted_state() const
{
  return current_predicted_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>&
Quadrotor<flight_controller_t, FilterType, ActionType>::
  current_predicted_enhanced_state()
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::
  current_predicted_enhanced_state() const
{
  return current_predicted_enhanced_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<State<FilterType, std::vector<RSSI>>>
Quadrotor<flight_controller_t, FilterType, ActionType>::all_states() const
{
  return all_states_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::last_state()
{
  if (all_states_.size() > 1) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 1);
    last_state_ = (*it_state);
  }
  return last_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::before_last_state()
{
  if (all_states_.size() > 2) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 2);
    before_last_state_ = (*it_state);
  }
  return before_last_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
State<FilterType, std::vector<RSSI>>
Quadrotor<flight_controller_t, FilterType, ActionType>::before_2_last_state()
{
  if (all_states_.size() > 3) {
    auto it_state = all_states_.rbegin();
    it_state = std::next(it_state, 3);
    before_2_last_state_ = (*it_state);
  }
  return before_2_last_state_;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::reset_all_states()
{
  all_states_.clear();
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::current_loss(
  arma::vec loss_vector)
{
  loss_vector_ = loss_vector;
}

template<class flight_controller_t, class FilterType, class ActionType>
arma::vec
Quadrotor<flight_controller_t, FilterType, ActionType>::current_loss() const
{
  return loss_vector_;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::sample_action()
{
  all_actions_.push_back(action.Data());
  current_action_ = action;
}

template<class flight_controller_t, class FilterType, class ActionType>
ActionType&
Quadrotor<flight_controller_t, FilterType, ActionType>::current_action()
{
  return current_action_;
}

template<class flight_controller_t, class FilterType, class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, ActionType>::current_action() const
{
  return current_action_;
}

template<class flight_controller_t, class FilterType, class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, ActionType>::last_action()
{
  if (all_actions_.size() > 1) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 1);
    last_action_ = (*it_action);
  }
  return last_action_;
}

template<class flight_controller_t, class FilterType, class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, ActionType>::before_last_action()
{
  if (all_actions_.size() > 2) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 2);
    before_last_action_ = (*it_action);
  }
  return before_last_action_;
}

template<class flight_controller_t, class FilterType, class ActionType>
ActionType
Quadrotor<flight_controller_t, FilterType, ActionType>::before_2_last_action()
{
  if (all_actions_.size() > 3) {
    auto it_action = all_actions_.rbegin();
    it_action = std::next(it_action, 3);
    before_2_last_action_ = (*it_action);
  }
  return before_2_last_action_;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<ActionType>
Quadrotor<flight_controller_t, FilterType, ActionType>::all_actions() const
{
  return all_actions_;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::reset_all_actions()
{
  all_actions_.clear();
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_state()
{
  dataset_.save_state(name_, vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::
  save_dataset_rssi_velocity()
{
  // see if it is possible to make current action generic
  dataset_.save_csv_dataset_2_file(
    name_, vec_.to_std_vector(current_state().Data(), current_action().Data()));
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_dataset_sasas()
{
  dataset_.save_csv_dataset_2_file(
    name_,
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_dataset_sasasp()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_current_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_predicted_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::
  save_dataset_with_current_enhanced_predictions()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_enhanced_predictions",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_predicted_enhanced_state().Data()),
    vec_.to_std_vector(current_state().Data()));
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_dataset_with_loss()
{
  dataset_.save_csv_dataset_2_file(
    name_ + "_loss",
    vec_.to_std_vector(before_last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(last_action(), 7)),
    vec_.to_std_vector(last_state().Data()),
    vec_.to_std_vector(encode_.to_one_hot_encoding(current_action(), 7)),
    vec_.to_std_vector(current_loss()));
}

template<class flight_controller_t, class FilterType, class ActionType>
template<typename Arg, typename... Args>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_loss(Arg arg,
                                                                  Args... args)
{
  dataset_.save_error_file(name_, arg, args...);
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_histogram(
  int count)
{
  /*  Save a version of the time steps to create a histogram */
  /*  Increase one since count start from 0 */
  /*  Need to find a solution for this with a timer */
  count++;
  dataset_.save_controller_count(count);
  histo_.histogram(count);
  dataset_.save_histogram(histo_.get_histogram<int>());
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_actions_evaluation(
  DiscretActions::Action first_action,
  DiscretActions::Action second_action)
{
  if (first_action == second_action) {
    dataset_.save_actions(name_, 1);
  } else {
    dataset_.save_actions(name_, -1);
  }
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::save_episodes(
  int n_episodes)
{
  dataset_.save_episodes(n_episodes);
}

template<class flight_controller_t, class FilterType, class ActionType>
double
Quadrotor<flight_controller_t, FilterType, ActionType>::height()
{
  return position().Z();
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, ActionType>::position() const
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, ActionType>::position()
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_1_antenna_position()
  const
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_1_antenna_position()
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_2_antenna_position()
  const
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_2_antenna_position()
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d
Quadrotor<flight_controller_t, FilterType, ActionType>::wt_antenna_position()
  const
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Vector3d&
Quadrotor<flight_controller_t, FilterType, ActionType>::wt_antenna_position()
{
  std::lock_guard<std::mutex> lock(_position_mutex);
  return _position;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Quaternion<double>
Quadrotor<flight_controller_t, FilterType, ActionType>::orientation() const
{
  std::lock_guard<std::mutex> lock(_orientation_mutex);
  return _orientation;
}

template<class flight_controller_t, class FilterType, class ActionType>
ignition::math::Quaternion<double>&
Quadrotor<flight_controller_t, FilterType, ActionType>::orientation()
{
  std::lock_guard<std::mutex> lock(_orientation_mutex);
  return _orientation;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<RSSI>
Quadrotor<flight_controller_t, FilterType, ActionType>::rssi_from_neighbors()
  const
{
  std::lock_guard<std::mutex> lock(_rssi_from_neighbors_mutex);
  return _rssi_from_neighbors;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<RSSI>&
Quadrotor<flight_controller_t, FilterType, ActionType>::rssi_from_neighbors()
{
  std::lock_guard<std::mutex> lock(_rssi_from_neighbors_mutex);
  return _rssi_from_neighbors;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::vector<double>
Quadrotor<flight_controller_t, FilterType, ActionType>::distances_to_neighbors()
{
  std::vector<double> distances;
  // std::map<unsigned int, double> neigh_distances;
  // neigh_distances =
  //   dist_.distances_to_neighbors(id_, nearest_neighbors_, position());
  // distances = vec_.map_to_vector(neigh_distances);
  return distances;
}

template<class flight_controller_t, class FilterType, class ActionType>
double
Quadrotor<flight_controller_t, FilterType, ActionType>::distance_to(int id)
{
  double distance = distances_to_neighbors().at(id);
  return distance;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::
  wireless_receiver_1_topic_name()
{
  return "/gazebo/default/" + name() + "/WR_1/Wireless Reveiver/transceiver";
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::
  wireless_receiver_2_topic_name()
{
  return "/gazebo/default/" + name() + "/WR_2/Wireless Reveiver/transceiver";
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::
  wireless_transmitter_topic_name()
{
  return "/gazebo/default/" + name() + "/WT/Wireless Transmitter/transceiver";
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::port_number() const
{
  return port_number_;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string&
Quadrotor<flight_controller_t, FilterType, ActionType>::port_number()
{
  return port_number_;
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::wt_name()
{
  return name() + "::wireless_transmitter::link";
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_1_name()
{
  return name() + "::wireless_receiver_1::link";
}

template<class flight_controller_t, class FilterType, class ActionType>
std::string
Quadrotor<flight_controller_t, FilterType, ActionType>::wr_2_name()
{
  return name() + "::wireless_receiver_2::link";
}

/*  Parsing the RSSI send by Gazebo */
template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::RxMsgN1(
  const ConstWirelessNodesPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_rx_1_mutex);
  int numRxNodes = _msg->node_size();
  rssi_from_neighbors().resize(numRxNodes);
  for (int i = 0; i < numRxNodes; ++i) {
    gazebo::msgs::WirelessNode RxNode = _msg->node(i);
    rssi_from_neighbors().at(i).name = RxNode.essid();
    rssi_from_neighbors().at(i).antenna_1 = RxNode.signal_level();
  }
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::RxMsgN2(
  const ConstWirelessNodesPtr& _msg)
{
  std::lock_guard<std::mutex> lock(_rx_2_mutex);
  int numRxNodes = _msg->node_size();
  rssi_from_neighbors().resize(numRxNodes);
  for (int i = 0; i < numRxNodes; ++i) {
    gazebo::msgs::WirelessNode RxNode = _msg->node(i);
    rssi_from_neighbors().at(i).name = RxNode.essid();
    rssi_from_neighbors().at(i).antenna_2 = RxNode.signal_level();
  }
}

template<class flight_controller_t, class FilterType, class ActionType>
gazebo::transport::NodePtr
Quadrotor<flight_controller_t, FilterType, ActionType>::node()
{
  return node_;
}

template<class flight_controller_t, class FilterType, class ActionType>
void
Quadrotor<flight_controller_t, FilterType, ActionType>::subRxTopic()
{
  std::string topic_WR_1 = wireless_receiver_1_topic_name();
  subs_.push_back(node_->Subscribe(
    topic_WR_1,
    &Quadrotor<flight_controller_t, FilterType, ActionType>::RxMsgN1,
    this));

  std::string topic_WR_2 = wireless_receiver_2_topic_name();
  subs_.push_back(node_->Subscribe(
    topic_WR_2,
    &Quadrotor<flight_controller_t, FilterType, ActionType>::RxMsgN2,
    this));
}
