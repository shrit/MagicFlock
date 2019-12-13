#pragma once

/*
 * @author: Omar Shrit
 *
 * This file implement the distance between the states
 *
*/

/*  Local includes */
# include "global.hh"
# include "math_tools.hh"
# include "state.hh"

namespace lt = local_types;

template <class simulator_t>
class StateDistance {

public:
  StateDistance(State<simulator_t> quad_last_state,
		State<simulator_t> quad_current_state,
		State<simulator_t> s_t,
		State<simulator_t> s_t_1,
		int same_action_value);

  double sum_fabs(State<simulator_t> s);
  double distance() const;

private:
  double distance_;
};

# include "distance_state.hxx"
