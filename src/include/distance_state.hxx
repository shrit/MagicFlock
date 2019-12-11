#pragma once

# include "distance_state.hh"

template <class simulator_t>
StateDistance<simulator_t>::StateDistance(State<simulator_t> quad_last_state,
					  State<simulator_t> quad_current_state,
					  State<simulator_t> s_t,
					  State<simulator_t> s_t_1,
					  int same_action_value)  
{
  double d1 = std::fabs(quad_last_state - s_t);      
  double d2 = std::fabs(quad_current_state - s_t_1);      
  distance_ = d1 + d2;    
}

double StateDistance<simulator_t>::distance() const
{ return distance_; }
