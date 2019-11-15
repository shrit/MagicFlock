#pragma once

# include "state.hh"

template <class simulator_t>
State<simulator_t>::State(std::shared_ptr<simulator_t> sim_interface,
			  unsigned int id,
			  std::vector<unsigned int> nearest_neighbors):
  sim_interface_(std::move(sim_interface)),
  pmodel_(sim_interface_)
{
  rssi_ = sim_interface_->rssi();
  height_follower_2_ = sim_interface_->positions().follower_2.z;
  height_follower_1_ = sim_interface_->positions().follower_1.z;
  height_leader_ = sim_interface_->positions().leader_.z;
  z_orinetation_  = sim_interface_->orientations().follower_2.z;
  dists_2D_ = mtools_.triangle_side_2D(sim_interface_->positions());
  dists_3D_ = mtools_.triangle_side_3D(sim_interface_->positions());
  e_dists_ = pmodel_.distances_2D();
}

template <class simulator_t>
lt::rssi<double> State<simulator_t>::
signal_strength() const
{ return rssi_; }

template <class simulator_t>
double State<simulator_t>::
height_f1() const
{ return height_f1_; }

template <class simulator_t>
double State<simulator_t>::
height_f2() const
{ return height_f2_; }

template <class simulator_t>
double State<simulator_t>::
height_difference() const
{ return (height_f2_ - height_f1_); }

template <class simulator_t>
double State<simulator_t>::
orientation () const
{ return z_orinetation_; }

template <class simulator_t>
lt::triangle<double> State<simulator_t>::
distances_2D () const
{ return dists_2D_; }

template <class simulator_t>
lt::triangle<double> State<simulator_t>::
distances_3D () const
{ return dists_3D_; }

template <class simulator_t>
lt::triangle<double> State<simulator_t>::
estimated_distances () const
{ return e_dists_; }

template <class simulator_t>
inline std::ostream& operator<< (std::ostream& out, const State<simulator_t>& s)
{
  out << s.distances_3D().f1
      <<","
      << s.distances_3D().f2
      <<","
      <<s.height_difference();
  return out;
}
