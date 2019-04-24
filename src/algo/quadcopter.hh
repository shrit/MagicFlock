/*
 * @author: Omar Shrit 
 *
 * This file implement the state class for quadcopters
 *
 *
*/

#ifndef QUADCOPTER_HH_
#define	QUADCOPTER_HH_

# include "../global.hh"

template <class simulator_t>
class Quadcopter {

public:

  class State {
    
    
  public:
    
    State();

    
    double height (std::shared_ptr<simulator_t> gzs) const;
    lt::rssi<double> signal_strength (std::shared_ptr<simulator_t> gzs) const;
    double distances () const;
    double orientation () const;

  private:

    
    
  };
  
    
  enum Action
    {
     forward,
     bacward,
     left,
     right,          
    };
  
  

  
  
};




# include "quadcopter.hxx"

#endif
