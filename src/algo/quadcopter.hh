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

struct state_printer {
  lt::rssi<double> rssi   ;
  double  height   ;
  lt::triangle<double>  distances  ;
  double  orientation   ;
};

inline std::ostream& operator<< (std::ostream& out, const state_printer& s)
{
  out << s.rssi <<","
      << s.height << "," 
      << s.distances << ","
      << s.orientation ;  
  return out;
}

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

    state_printer
    create_printer_struct(Quadcopter<simulator_t>::State state);
      
    
  private:
    
    Math_tools mtools_;
    lt::rssi<double> rssi_ ;
    double height_;
    double z_orinetation_  ;
    lt::triangle<double> dists_ ;
    
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
     NoMove,
    };

  enum class Reward
    {
     very_good,
     good,
     bad,
     very_bad,     
    };
  
private:  
  
};

# include "quadcopter.hxx"
