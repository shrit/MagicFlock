/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 */
#pragma once

#include <queue>
#include <stack>
#include <utility>
#include <vector>

#include "Vector.hh"
#include "action.hh"
#include "check_swarm_shape.hh"
#include "compute_distance.hh"
#include "data_set.hh"
#include "histogram.hh"
#include "math_tools.hh"
#include "one_hot_encoding.hh"
#include "real_time_samples.hh"
#include "state.hh"

/* Ignition related headers*/
#include <ignition/math6/ignition/math/Quaternion.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>

#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

struct RSSI
{
  double antenna_1; /* Value measured on antenna 1*/
  double antenna_2; /* Value measured on antenna 2*/
  std::string name; /* Name of the transmitter quadrotor */
};

template<class flight_controller_t, class NoiseType>
class Quadrotor
{

public:
  Quadrotor(unsigned int id, std::string name, std::string label);

  using NodePtr = gazebo::transport::NodePtr;
  using SubPtr = gazebo::transport::SubscriberPtr;

  unsigned int id() const;
  std::string name() const;
  std::string label() const;

  /* Neighbors related fuctions*/
  std::vector<unsigned int> nearest_neighbors() const;
  void add_nearest_neighbor_id(unsigned int id);
  double distance_to(int id);
  std::vector<double> distances_to_neighbors();

  /* Position related functions */
  ignition::math::Vector3d position() const;
  ignition::math::Vector3d& position();
  ignition::math::Vector3d wr_1_antenna_position() const;
  ignition::math::Vector3d& wr_1_antenna_position();
  ignition::math::Vector3d wr_2_antenna_position() const;
  ignition::math::Vector3d& wr_2_antenna_position();
  ignition::math::Vector3d wt_antenna_position() const;
  ignition::math::Vector3d& wt_antenna_position();
  ignition::math::Quaternion<double> orientation() const;
  ignition::math::Quaternion<double>& orientation();
  std::vector<ignition::math::Vector3d> position_of_neighbors() const;

  /*  State related functions */
  void sample_state();
  State<NoiseType> current_state() const;
  State<NoiseType>& current_predicted_state();
  State<NoiseType> current_predicted_state() const;
  State<NoiseType>& current_predicted_enhanced_state();
  State<NoiseType> current_predicted_enhanced_state() const;
  State<NoiseType> last_state();
  State<NoiseType> before_last_state();
  State<NoiseType> before_2_last_state();
  std::vector<State<NoiseType>> all_states() const;
  void reset_all_states();
  void start_sampling_rt_state(int interval);
  void stop_sampling_rt_state();

  /* Loss related functions */
  void current_loss(arma::vec current_loss);
  arma::vec current_loss() const;

  /*Action related functions */
  Actions::Action current_action() const;
  void current_action(Actions::Action action);
  Actions::Action last_action();
  Actions::Action before_last_action();
  Actions::Action before_2_last_action();
  std::vector<Actions::Action> all_actions() const;
  void reset_all_actions();
  double height();
  void subRxTopic();

  /*  Data set related functions */
  void register_data_set();
  void register_data_set_with_current_predictions();
  void register_data_set_with_current_enhanced_predictions();
  void register_data_set_with_loss();
  void register_episodes(int n_episode);
  void register_histogram(int count);
  template<typename Arg, typename... Args>
  void register_loss(Arg arg, Args... args);
  void register_state();
  void register_actions_evaluation(Actions::Action first_action,
                                   Actions::Action second_action);
  bool examin_geometric_shape();

  void reset_models();

  std::vector<RSSI> rssi_from_neighbors() const;
  std::vector<RSSI>& rssi_from_neighbors();

  std::string reset_topic_name();
  std::string wireless_receiver_1_topic_name();
  std::string wireless_receiver_2_topic_name();
  std::string wireless_transmitter_topic_name();
  std::string wr_1_name(); /* Wireless antenna name */
  std::string wr_2_name();
  std::string wt_name();

  std::string port_number();
  std::shared_ptr<flight_controller_t> controller();
  NodePtr node();

  void start_controller();

  void RxMsgN1(const ConstWirelessNodesPtr& _msg);
  void RxMsgN2(const ConstWirelessNodesPtr& _msg);

  Quadrotor(const Quadrotor&);

private:
  Actions::Action current_action_{ Actions::Action::Unknown };
  Actions::Action last_action_{ Actions::Action::Unknown };
  Actions::Action before_last_action_{ Actions::Action::Unknown };
  Actions::Action before_2_last_action_{ Actions::Action::Unknown };
  std::vector<Actions::Action> all_actions_;
  std::vector<Actions::Action> action_container_;
  std::vector<SubPtr> subs_;
  ComputeDistance dist_;
  VectorHelper vec_;
  OneHotEncoding encode_;
  CheckShape shape_;
  Histogram histo_;
  DataSet data_set_;
  arma::vec loss_vector_;
  State<NoiseType> current_predicted_state_;
  State<NoiseType> current_predicted_enhanced_state_;
  State<NoiseType> current_state_;
  State<NoiseType> last_state_;
  State<NoiseType> before_last_state_;
  State<NoiseType> before_2_last_state_;
  std::vector<State<NoiseType>> all_states_;

  mutable std::mutex _rssi_from_neighbors_mutex{};
  std::vector<RSSI> _rssi_from_neighbors;

  std::shared_ptr<RTSamples> rt_samples_;
  unsigned int id_;  /* Quadrotor id  (Parsed from gazebo)*/
  std::string name_; /* Quadrotor name  (Parsed from gazebo)*/
  /* Quadrotor label (Given by the user, leader, follower, etc)*/
  std::string label_;
  std::vector<unsigned int> nearest_neighbors_;
  std::shared_ptr<flight_controller_t> controller_;

  mutable std::mutex _position_mutex{};
  ignition::math::Vector3d _position;

  mutable std::mutex _wr_1_position_mutex{};
  ignition::math::Vector3d _wr_1_position;

  mutable std::mutex _wr_2_position_mutex{};
  ignition::math::Vector3d _wr_2_position;

  mutable std::mutex _wt_position_mutex{};
  ignition::math::Vector3d _wt_position;

  mutable std::mutex _orientation_mutex{};
  ignition::math::Quaternion<double> _orientation;
  mutable std::mutex _rx_1_mutex{};
  mutable std::mutex _rx_2_mutex{};  
  NodePtr node_;
};

#include "quadrotor.hxx"
