/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 */
#pragma once
/* C++ Standard library includes*/
#include <queue>
#include <stack>
#include <utility>
#include <vector>

/* IL4MRC library includes */
#include "Vector.hh"
#include "compute_distance.hh"
#include "continuous_actions.hh"
#include "dataset.hh"
#include "discret_actions.hh"
#include "flocking.hh"
#include "histogram.hh"
#include "math_tools.hh"
#include "nearest_neighbors.hh"
#include "one_hot_encoding.hh"
#include "real_time_samples.hh"
#include "state.hh"

/* Ignition related headers */
#include <ignition/math6/ignition/math/Quaternion.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>

/* Gazebo simulator includes */
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

struct RSSI
{
  double antenna_1; /* Value measured on antenna 1*/
  double antenna_2; /* Value measured on antenna 2*/
  std::string name; /* Name of the transmitter quadrotor */
};

template<class flight_controller_t, class FilterType, class ActionType>
class Quadrotor
  : public std::enable_shared_from_this<
      Quadrotor<flight_controller_t, FilterType>>
{

public:
  Quadrotor();

  using NodePtr = gazebo::transport::NodePtr;
  using SubPtr = gazebo::transport::SubscriberPtr;
  using inner_flight_controller = flight_controller_t;

  void init(unsigned int id, std::string name, std::string label);
  /* Quadrotor related functions*/
  unsigned int id() const;
  std::string name() const;
  std::string label() const;
  double height();
  std::string port_number() const;
  std::string& port_number();
  NodePtr node();
  void start_controller();
  ignition::math::Vector3d start_flocking(double sepGain,
                                          double cohGain,
                                          double migGain,
                                          double cutoffDist);
  void stop_flocking();

  /* Neighbors related fuctions*/
  /* This function allows you to get a shared ptr to all quadrotors in the
   * simulator*/
  void make_reference_2_swarm(
    std::vector<Quadrotor<flight_controller_t, FilterType, ActionType>> quads);
  std::vector<unsigned int> nearest_neighbors() const;
  void start_nearest_neighbor_detector();
  void stop_nearest_neighbor_detector();
  double distance_to(int id);
  std::vector<double> distances_to_neighbors();
  std::vector<RSSI> rssi_from_neighbors() const;
  std::vector<RSSI>& rssi_from_neighbors();

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
  State<FilterType> current_state() const;
  State<FilterType>& current_predicted_state();
  State<FilterType> current_predicted_state() const;
  State<FilterType>& current_predicted_enhanced_state();
  State<FilterType> current_predicted_enhanced_state() const;
  State<FilterType> last_state();
  State<FilterType> before_last_state();
  State<FilterType> before_2_last_state();
  std::vector<State<FilterType>> all_states() const;
  void reset_all_states();
  void start_sampling_rt_state(int interval);
  void stop_sampling_rt_state();

  /* Loss related functions */
  void current_loss(arma::vec current_loss);
  arma::vec current_loss() const;

  /* Discret Actions related functions */
  DiscretActions::Action current_action() const;
  void current_action(DiscretActions::Action action);
  DiscretActions::Action last_action();
  DiscretActions::Action before_last_action();
  DiscretActions::Action before_2_last_action();
  std::vector<DiscretActions::Action> all_actions() const;
  void reset_all_actions();

  /*  Data set related functions */

  /* Save dataset as rssi, velocity (r, v)*/
  void save_dataset_rssi_velocity();
  /* Save dataset as state action (s_t-1,a_t-1,s_t,a_t,s_t+1) */
  void save_dataset_sasas();
  /* Save dataset with predicted state as state action
   * (s_t-1,a_t-1,s_t,a_t,s_t+1, s^_t+1) */
  void save_dataset_sasasp();
  /* Save dataset as state action (s_t-1,a_t-1,s_t,a_t,s_t+1) */
  void save_dataset_with_current_enhanced_predictions();
  /* Save dataset as state action (s_t-1,a_t-1,s_t,a_t,s_t+1) */
  void save_dataset_with_loss();
  /* Save episode number */
  void save_episodes(int n_episode);
  void save_histogram(int count);
  template<typename Arg, typename... Args>
  void save_loss(Arg arg, Args... args);
  void save_state();
  void save_actions_evaluation(DiscretActions::Action first_action,
                               DiscretActions::Action second_action);

  /* Topic names related functions */
  std::string wireless_receiver_1_topic_name();
  std::string wireless_receiver_2_topic_name();
  std::string wireless_transmitter_topic_name();
  std::string wr_1_name(); /* Wireless antenna name */
  std::string wr_2_name();
  std::string wt_name();

  /* Gazebo callbacks to recover the value of the signal strenght*/
  void RxMsgN1(const ConstWirelessNodesPtr& _msg);
  void RxMsgN2(const ConstWirelessNodesPtr& _msg);
  void subRxTopic();

  Quadrotor(const Quadrotor&){};
  Quadrotor& operator=(const Quadrotor& quad) { return *this; };

  // Quadrotor(Quadrotor&& quad) = default;
  // Quadrotor& operator=(Quadrotor&& quad) = default;
  std::unique_ptr<flight_controller_t> controller_;

private:
  DiscretActions::Action current_action_{ DiscretActions::Action::Unknown };
  DiscretActions::Action last_action_{ DiscretActions::Action::Unknown };
  DiscretActions::Action before_last_action_{ DiscretActions::Action::Unknown };
  DiscretActions::Action before_2_last_action_{
    DiscretActions::Action::Unknown
  };
  std::vector<DiscretActions::Action> all_actions_;
  std::vector<DiscretActions::Action> action_container_;
  std::vector<SubPtr> subs_;
  ComputeDistance dist_;
  VectorHelper vec_;
  OneHotEncoding encode_;
  Histogram histo_;
  DataSet dataset_;
  arma::vec loss_vector_;
  State<FilterType> current_predicted_state_;
  State<FilterType> current_predicted_enhanced_state_;
  State<FilterType> current_state_;
  State<FilterType> last_state_;
  State<FilterType> before_last_state_;
  State<FilterType> before_2_last_state_;
  std::vector<State<FilterType>> all_states_;

  mutable std::mutex _rssi_from_neighbors_mutex{};
  std::vector<RSSI> _rssi_from_neighbors;
  std::vector<unsigned int> _nearest_neighbors;

  std::shared_ptr<RTSamples> state_sampler_, neighbor_sampler_,
    flocking_sampler_;
  unsigned int id_;  /* Quadrotor id */
  std::string name_; /* Quadrotor name */
  /* Quadrotor label (Given by the user, leader, follower, etc)*/
  std::string label_;

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
  std::string port_number_;
  std::vector<Quadrotor<flight_controller_t, FilterType>>
    quads_;
};

#include "quadrotor.hxx"
