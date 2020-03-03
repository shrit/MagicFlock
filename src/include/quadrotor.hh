/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 *
 */
#pragma once

#include <queue>
#include <stack>
#include <utility>
#include <vector>

#include "action.hh"
#include "data_set.hh"
#include "math_tools.hh"
#include "real_time_samples.hh"
#include "state.hh"

template<class simulator_t>
class Quadrotor
{

public:
  Quadrotor(unsigned int id,
            std::string name,
            std::shared_ptr<simulator_t> sim_interface);

  unsigned int id() const;
  std::string name() const;

  /* Neighbors related fuctions*/
  std::vector<unsigned int> nearest_neighbors() const;
  void add_nearest_neighbor_id(unsigned int id);
  double distance_to(int id);
  std::vector<double> distances_to_neighbors();

  /*  State related functions */
  void sample_state();
  State<simulator_t> current_state() const;
  State<simulator_t>& current_predicted_state();
  State<simulator_t> current_predicted_state() const;
  State<simulator_t> last_state();
  State<simulator_t> before_last_state();
  State<simulator_t> before_2_last_state();
  std::vector<State<simulator_t>> all_states() const;
  void reset_all_states();

  void start_sampling_rt_state(int interval);
  void stop_sampling_rt_state();

  /* Loss related functions */
  void current_loss(arma::vec current_loss);
  arma::vec current_loss() const;

  /*  Action related functions */
  Actions::Action current_action() const;
  void current_action(Actions::Action action);
  Actions::Action last_action();
  Actions::Action before_last_action();
  Actions::Action before_2_last_action();
  std::vector<Actions::Action> all_actions() const;
  void reset_all_actions();
  double height();

  /*  Data set related functions */
  void register_data_set();
  void register_data_set_with_current_predictions();
  void register_data_set_with_loss();
  void register_episodes(int n_episode);
  void register_histogram(int count);
  template<typename Arg, typename... Args>
  void register_loss(Arg arg, Args... args);

  void register_actions_evaluation(Actions::Action first_action,
                                   Actions::Action second_action);
  /*  Speed related functions */
  double& speed();
  double speed() const;
  void increase_speed();
  void decrease_speed();
  void increase_speed_by_value(double speed);
  void decrease_speed_by_value(double speed);

  bool examin_geometric_shape();

private:
  Actions::Action current_action_{ Actions::Action::Unknown };
  Actions::Action last_action_{ Actions::Action::Unknown };
  Actions::Action before_last_action_{ Actions::Action::Unknown };
  Actions::Action before_2_last_action_{ Actions::Action::Unknown };
  std::vector<Actions::Action> all_actions_;
  std::vector<Actions::Action> action_container_;
  std::stack<Actions::Action> stack_of_future_actions_;
  std::queue<Actions::Action> queue_of_future_actions_;

  Math_tools mtools_;
  DataSet<simulator_t> data_set_;
  arma::vec loss_vector_;
  State<simulator_t> current_predicted_state_;
  State<simulator_t> current_state_;
  State<simulator_t> last_state_;
  State<simulator_t> before_last_state_;
  State<simulator_t> before_2_last_state_;
  std::vector<State<simulator_t>> all_states_;

  std::shared_ptr<RTSamples> rt_samples_;
  unsigned int id_;  /* Quadrotor id */
  std::string name_; /* Quadrotor name */
  double speed_ = 1; /*  Quadrotors speed. Default speed is equal to 1 m/s */
  std::vector<unsigned int> nearest_neighbors_;
  std::shared_ptr<simulator_t> sim_interface_;
};

#include "quadrotor.hxx"
