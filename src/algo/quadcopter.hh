/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 *
*/
#pragma once

# include <utility>

# include "action.hh"
# include "state.hh"

template <class simulator_t>  
class Quadrotor {

public:
  
  Quadrotor(unsigned int id,
	    std::string name);
  
  unsigned int id() const;
  std::string name() const;

  std::vector<unsigned int> nearest_neighbors() const;
  void add_nearest_neighbor_id(unsigned int id);
  
  State<simulator_t> current_state() const;
  State<simulator_t> last_state();
  std::vector<State<simulator_t>> all_states() const;
  void reset_all_states();
  
  Actions::Action current_action();
  Actions::Action last_action();
  std::vector<Actions::Action> all_actions() const;
  void reset_all_action();
  
private:

  Actions::Action current_action_;
  Actions::Action last_action_;
  std::vector<Actions::Action> all_actions_;
 
  State<simulator_t> current_state_;
  State<simulator_t> last_state_;
  std::vector<State<simulator_t>> all_states_;

  unsigned int id_; /* Quadrotor id */
  std::string name_; /* Quadrotor name */
  std::vector<unsigned int> nearest_neighbors_;

};

# include "quadcopter.hxx"
