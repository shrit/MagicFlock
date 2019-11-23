#pragma once

# include "state.hh"

template <class simulator_t>
State<simulator_t>::State(std::shared_ptr<simulator_t> sim_interface,
			  unsigned int id,
			  std::vector<unsigned int> nearest_neighbors)
  : sim_interface_(std::move(sim_interface)),

{
  auto leader =  sim_interface_->positions().begin();
  auto follower = sim_interface_->positions().begin() + id;
  //  estimated_dists_3D_ = pmodel_.distances_3D();
  dists_3D_ = mtools_.distances_to_neighbors(id,
					     nearest_neighbors,
					     sim_interface_->positions());
  alti_diff_ = (leader->z - follower->z);
}

template <class simulator_t>
double State<simulator_t>::
height_difference() const
{ return alti_diff_; }

template <class simulator_t>
std::vector<double> State<simulator_t>::
distances_3D() const
{ return dists_3D_; }

template <class simulator_t>
std::vector<double> State<simulator_t>::
estimated_distances() const
{ return estimated_dists_3D_; }

template <class simulator_t>
inline std::ostream& operator<< (std::ostream& out, const State<simulator_t>& s)
{
  out << s.distances_3D()
      << ","
      << s.height_difference();
  return out;
}
