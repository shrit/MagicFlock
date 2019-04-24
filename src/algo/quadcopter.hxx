# include "quadcopter.hh"


template <class simulator_t>
Quadcopter<simulator_t>::State::State(std::shared_ptr<simulator_t> sim_interface):
  sim_interface_(std::move(sim_interface))   
{}


template <class simulator_t>
lt::rssi<double> Quadcopter<simulator_t>::State::
signal_strength () const
{
  return  sim_interface_->rssi();  
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
height () const
{
  return sim_interface_->position().z;  
}

template <class simulator_t>
double Quadcopter<simulator_t>::State::
orientation () const
{
  return sim_interface_->orientation().z; // verify the orientation
}

template <class simulator_t>
lt::triangle<double> Quadcopter<simulator_t>::State::
distances () const
{
  return  mtools_.triangle_side(sim_interface_->positions());
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
