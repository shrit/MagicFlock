#pragma once

#include "state.hh"

template<class simulator_t>
State<simulator_t>::State()
{}

template<class simulator_t>
State<simulator_t>::State(std::shared_ptr<simulator_t> sim_interface,
                          unsigned int id,
                          std::vector<unsigned int> nearest_neighbors)
  : sim_interface_(std::move(sim_interface))
  , id_(id)
{
  leader_ = sim_interface_->positions().begin();
  follower_ = sim_interface_->positions().begin() + id;
  dists_3D_ = mtools_.distances_to_neighbors(
    id, nearest_neighbors, sim_interface_->positions());
  alti_diff_ = (leader_->z - follower_->z);
}

template<class simulator_t>
State<simulator_t>::State(std::vector<double> distances, double altitude_diff)
{
  dists_3D_ = distances;
  alti_diff_ = altitude_diff;
}

template<class simulator_t>
std::vector<State<simulator_t>>
State<simulator_t>::StateConstructor(arma::mat values)
{
  std::vector<State<simulator_t>> states;
  for (int i = 0; i < values.n_rows; ++i) {
    arma::rowvec state_row = values.row(i);
    logger::logger_->info("state vector row vec: {}", state_row);
    std::vector<double> state = mtools_.to_std_vector(state_row);
    double alti_diff = state.back();
    state.pop_back();
    std::vector<double> dists_3D = state;
    states.emplace_back(dists_3D, alti_diff);
  }
  return states;
}

template<class simulator_t>
double
State<simulator_t>::rt_height()
{
  auto quad = sim_interface_->positions().begin() + id_;
  altitude_ = quad->z;
  return altitude_;
}

template<class simulator_t>
double
State<simulator_t>::height_difference() const
{
  return alti_diff_;
}

template<class simulator_t>
double
State<simulator_t>::rt_height_difference()
{
  alti_diff_ = (leader_->z - follower_->z);
  return alti_diff_;
}

template<class simulator_t>
std::vector<double>
State<simulator_t>::distances_3D() const
{
  return dists_3D_;
}

template<class simulator_t>
double
State<simulator_t>::distance_to(int id)
{
  double distance = dists_3D_.at(id);
  return distance;
}

template<class simulator_t>
std::vector<double>
State<simulator_t>::estimated_distances() const
{
  return estimated_dists_3D_;
}

template<class simulator_t>
inline std::ostream&
operator<<(std::ostream& out, const State<simulator_t>& s)
{
  out << s.distances_3D() << "," << s.height_difference();
  return out;
}

template<class simulator_t>
inline State<simulator_t>
operator-(const State<simulator_t>& s, const State<simulator_t>& s1)
{
  State<simulator_t> s_result;
  s_result.alti_diff = s.height_diff() - s1.height_diff();
  s_result.distance_3D() = s.distance_3D() - s1.distance_3D();
  return s_result;
}

template<class simulator_t>
inline State<simulator_t>
operator+(const State<simulator_t>& s, const State<simulator_t>& s1)
{
  State<simulator_t> s_result;
  s_result.alti_diff = s.height_diff() + s1.height_diff();
  s_result.distance_3D() = s.distance_3D() + s1.distance_3D();
  return s_result;
}

namespace ILMR {
bool
comparator(double d1, double d2)
{
  double epsilon = 0.5;
  if ((d1 - d2) < epsilon) {
    return true;
  } else
    return false;
}
}

template<class simulator_t>
inline bool
operator==(const State<simulator_t>& s, const State<simulator_t>& s1)
{
  bool result_distance = std::equal(s.distances_3D().begin(),
                                    s.distances_3D().end(),
                                    s1.distances_3D().begin(),
                                    [](const double& d, const double& d1) {
                                      bool result = ILMR::comparator(d, d1);
                                      return result;
                                    });

  bool result_height =
    comparator(s.height_difference(), s1.height_difference());

  bool result = false;
  if (result_distance and result_height) {
    result = true;
  }
  return result;
}
