/*
 * @author: Omar Shrit 
 *
 * This file implement the state class for quadcopters
 *
 *
*/

#pragma once

# include <utility>

# include "../math_tools.hh"
# include "../global.hh"

namespace lt = local_types;

template <class simulator_t>
class Quadcopter {

public:

  class State {
        
  public:
    
    State(std::shared_ptr<simulator_t> sim_interface);
    
    double height () const;
    lt::rssi<double> signal_strength () const;
    lt::triangle<double> distances () const;
    double orientation () const;

  private:
    
    Math_tools mtools_;
    
    /*  Create a shared pointer to a simulator interface The interface
     need to have all the required data about the quadcopter*/
    std::shared_ptr<simulator_t> sim_interface_;
  };
  
    
  enum class Action
    {
     forward,
     backward,
     left,
     right,          
    };
      
  
};

# include "quadcopter.hxx"
