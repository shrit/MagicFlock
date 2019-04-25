# include "quadcopter.hh"


template <class simulator_t>
Quadcopter<simulator_t>::State::State(std::shared_ptr<simulator_t> sim_interface):
  sim_interface_(std::move(sim_interface))   
{  
  rssi_ = sim_interface_->rssi();  
  height_ = sim_interface_->positions().f2.z;  
  z_orinetation_  = sim_interface_->orientations().f2.z;
  dists_ =   mtools_.triangle_side(sim_interface_->positions());  
}


template <class simulator_t>
lt::rssi<double> Quadcopter<simulator_t>::State::
signal_strength () const
{
  return rssi_ ; 
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
height () const
{/*  need to make it according to what quad?? */
  return height_;
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
orientation () const
{
  return z_orinetation_;
}

template <class simulator_t>
lt::triangle<double> Quadcopter<simulator_t>::State::
distances () const
{
  return dists_;
}

/*  To resolve later */

// template <class simulator_t>
// std::ostream& operator<< (std::ostream& out, Quadcopter<simulator_t>::State& state)
// {
//   out << state.signal_strength() <<","
//       << state.height() << "," 
//       << state.distances() << ","
//       << state.orientation() ;  
//   return out;
// }
