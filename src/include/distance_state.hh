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
  StateDistance();
  double distance() const;

private:

  double distance_;
  

  
  
};


# include "distance_state.hxx"
