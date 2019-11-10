/*
 * @author: Omar Shrit
 *
 * This file implement the states, actions of a quadrotor
 *
 *
*/
#pragma once

# include <random>
# include <utility>

# include "action.hh"
# include "../global.hh"
# include "../math_tools.hh"
# include "propagation_model.hh"
# include "state.hh"

namespace lt = local_types;

template <class simulator_t>  
class Quadrotor {

public:
  
  Quadrotor(unsigned int id,
	    std::string name);
  
  unsigned int id() const;
  std::string name() const;

  std::vector<unsigned int> nearest_neighbors(unsigned int);
  
  State<simulator_t> current_state() const;
  State<simulator_t> last_state();
  std::vector<State<simulator_t>> all_states() const;
  void reset_all_states();
  
  Actions::Action current_action();
  Actions::Action last_action() const;
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

};

# include "quadcopter.hxx"
