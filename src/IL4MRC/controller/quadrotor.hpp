/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 */
#pragma once
/* C++ Standard library includes */
#include <string>
#include <utility>
#include <vector>

/* IL4MRC library includes */
#include <IL4MRC/actions/continuous_actions.hpp>
#include <IL4MRC/actions/discret_actions.hpp>
#include <IL4MRC/data/dataset.hpp>
#include <IL4MRC/metrics/exponential_moving_average.hpp>
#include <IL4MRC/metrics/histogram.hpp>
#include <IL4MRC/metrics/max_distance.hpp>
#include <IL4MRC/metrics/min_distance.hpp>
#include <IL4MRC/neighbors/nearest_neighbors.hpp>
#include <IL4MRC/state/state.hpp>
#include <IL4MRC/swarm_model/collision_detector/collision_detector.hpp>
#include <IL4MRC/swarm_model/flocking/flocking.hpp>
#include <IL4MRC/swarm_model/random/random.hpp>
#include <IL4MRC/util/Vector.hpp>
#include <IL4MRC/util/real_time_samples.hpp>

/* Ignition related headers */
#include <ignition/math6/ignition/math/Quaternion.hh>
#include <ignition/math6/ignition/math/Vector2.hh>
#include <ignition/math6/ignition/math/Vector3.hh>

/* Gazebo simulator includes */
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

template<class flight_controller_t,
         class FilterType,
         class NoiseType,
         class StateType,
         class ActionType>
class Quadrotor
  : public std::enable_shared_from_this<Quadrotor<flight_controller_t,
                                                  FilterType,
                                                  NoiseType,
                                                  StateType,
                                                  ActionType>>
{

public:
  Quadrotor();

  using NodePtr = gazebo::transport::NodePtr;
  using SubPtr = gazebo::transport::SubscriberPtr;
  using inner_flight_controller = flight_controller_t;
  using Action = ActionType;

  void init(unsigned int id,
            std::string name,
            std::string label,
            int number_of_quad,
            std::vector<Quadrotor<flight_controller_t,
                                  FilterType,
                                  NoiseType,
                                  StateType,
                                  ActionType>>& quads);

  /* Quadrotor related functions*/
  unsigned int id() const;
  std::string name() const;
  std::string label() const;
  double height();
  std::string port_number() const;
  std::string& port_number();
  void start_controller();
  void start_flocking_model(ignition::math::Vector4d gains,
                            ignition::math::Vector3d destination,
                            ignition::math::Vector3d max_speed);
  void start_random_model(ignition::math::Vector4d axis_speed);
  void start_collision_detector(int duration);
  void stop_flocking_model();
  void stop_random_model();
  void stop_collision_detector();

  /* distance to neighbors related fuctions*/
  double distance_to(int id);
  std::vector<double> distances_to_neighbors();
  void calculate_distances_to_neighbors_antenna();
  std::vector<StateType> rssi_from_neighbors() const;
  std::vector<StateType>& rssi_from_neighbors();
  std::vector<AntennaDists> neigh_antenna_dists_container() const;
  std::vector<AntennaDists>& neigh_antenna_dists_container();
  LaserScan laser_scan() const;
  LaserScan& laser_scan();

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
  std::vector<ignition::math::Vector3d>& neighbor_positions();
  std::vector<ignition::math::Vector3d> neighbor_positions() const;
  std::vector<ignition::math::Vector3d>& neighbor_antenna_positions();
  std::vector<ignition::math::Vector3d> neighbor_antenna_positions() const;
  void start_position_sampler(std::vector<Quadrotor<flight_controller_t,
                                                    FilterType,
                                                    NoiseType,
                                                    StateType,
                                                    ActionType>>& quads);
  void stop_position_sampler();

  /*  State related functions */
  void sample_state();
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
  current_state() const;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>&
  current_predicted_state();
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
  current_predicted_state() const;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>&
  current_predicted_enhanced_state();
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
  current_predicted_enhanced_state() const;
  State<FilterType, NoiseType, StateType, std::vector<StateType>> last_state();
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
  before_last_state();
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
  before_2_last_state();
  std::vector<State<FilterType, NoiseType, StateType, std::vector<StateType>>>
  all_states() const;
  void reset_all_states();
  void start_sampling_rt_state(int interval);
  void stop_sampling_rt_state();

  /* Loss related functions */
  void current_loss(arma::vec current_loss);
  arma::vec current_loss() const;

  /* Discret Actions related functions */
  void sample_action();
  ActionType current_action() const;
  ActionType& current_action();
  ActionType last_action();
  ActionType before_last_action();
  ActionType before_2_last_action();
  std::vector<ActionType> all_actions() const;
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

  void save_histogram(int count);
  void save_state();

  template<typename Arg, typename... Args>
  void save_values(std::string name, Arg value, Args... values);
  void save_position(std::string iteration);

  /* Topic names related functions */
  std::string wireless_receiver_1_topic_name();
  std::string wireless_receiver_2_topic_name();
  std::string wireless_transmitter_topic_name();
  std::string laser_scaner_topic_name();
  std::string wr_1_name(); /* Wireless antenna name */
  std::string wr_2_name();
  std::string wt_name();

  /* Gazebo callbacks to recover the value of the signal strenght*/
  void RxMsgN1(const ConstWirelessNodesPtr& _msg);
  void RxMsgN2(const ConstWirelessNodesPtr& _msg);
  void LaserScanMsg(const ConstLaserScanStampedPtr& _msg);
  void subRxTopic(NodePtr& node);
  void LaserScanTopic(NodePtr& node);
  Quadrotor(const Quadrotor&){};
  Quadrotor& operator=(const Quadrotor& quad) { return *this; };

  std::unique_ptr<flight_controller_t> controller_;

private:
  // See if these actions requires initialisation.
  ActionType current_action_;
  ActionType last_action_;
  ActionType before_last_action_;
  ActionType before_2_last_action_;
  std::vector<ActionType> all_actions_;
  std::vector<ActionType> action_container_;

  std::vector<SubPtr> subs_;
  VectorHelper vec_;
  Histogram histo_;
  DataSet dataset_;
  arma::vec loss_vector_;

  /* State related data member*/
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
    current_predicted_state_;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
    current_predicted_enhanced_state_;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
    current_state_;
  State<FilterType, NoiseType, StateType, std::vector<StateType>> last_state_;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
    before_last_state_;
  State<FilterType, NoiseType, StateType, std::vector<StateType>>
    before_2_last_state_;
  std::vector<State<FilterType, NoiseType, StateType, std::vector<StateType>>>
    all_states_;

  /* Metrics related data members*/
  MaxDistance<Quadrotor<flight_controller_t,
                        FilterType,
                        NoiseType,
                        StateType,
                        ActionType>>
    max_distance_;
  MinDistance<Quadrotor<flight_controller_t,
                        FilterType,
                        NoiseType,
                        StateType,
                        ActionType>>
    min_distance_;
  double max_dist_, min_dist_;

  /* Mutex related data member*/
  mutable std::mutex _position_sampler_mutex{};
  mutable std::mutex _flocking_mutex{};
  mutable std::mutex _random_mutex{};
  mutable std::mutex _collision_mutex{};
  mutable std::mutex _neighbor_positions_mutex{};
  mutable std::mutex _neighbor_antenna_positions_mutex{};
  mutable std::mutex _neigh_antenna_dists_container_mutex{};
  mutable std::mutex _rssi_from_neighbors_mutex{};
  mutable std::mutex _sample_state_mutex{};
  mutable std::mutex _wr_1_position_mutex{};
  mutable std::mutex _wr_2_position_mutex{};
  mutable std::mutex _wt_position_mutex{};
  mutable std::mutex _orientation_mutex{};
  mutable std::mutex _position_mutex{};
  mutable std::mutex _rx_1_mutex{};
  mutable std::mutex _rx_2_mutex{};
  mutable std::mutex _laser_mutex{};
  mutable std::mutex _laser_scan_mutex{};

  /* Sampler related data member*/
  RTSamples state_sampler_, flocking_sampler_, position_sampler_,
    collision_detector_sampler_;

  unsigned int id_;   /* Quadrotor id */
  std::string name_;  /* Quadrotor name */
  int num_neighbors_; /* Number of neighbors*/
  /* Quadrotor label (Given by the user, leader, follower, etc)*/
  std::string label_;
  std::string port_number_;

  /* Position related data member */
  std::vector<ignition::math::Vector3d> _neighbor_positions;
  std::vector<ignition::math::Vector3d> _neighbor_antenna_positions;
  std::vector<AntennaDists> _neigh_antenna_dists_container;
  std::vector<StateType> _rssi_from_neighbors;
  ignition::math::Vector3d _position;
  ignition::math::Vector3d _wr_1_position;
  ignition::math::Vector3d _wr_2_position;
  ignition::math::Vector3d _wt_position;
  ignition::math::Quaternion<double> _orientation;

  LaserScan _laser_scan;

  FilterType filter_;
};

#include "quadrotor_impl.hpp"
