# include "quadcopter.hh"


template <class simulator_t>
Quadcopter<simulator_t>::State::State(){};


template <class simulator_t>
lt::rssi<double> Quadcopter<simulator_t>::State::
signal_strength (std::shared_ptr<simulator_t> gzs) const
{
  return  gzs->rssi();  
};
