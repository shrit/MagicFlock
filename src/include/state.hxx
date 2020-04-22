#pragma once

template<class NoiseType>
State<NoiseType>::State()
  : data_(dimension, arma::fill::zeros)
{
  /* Nothing to do here. */
}

template<class NoiseType>
State<NoiseType>::State(const arma::colvec& data)
  : data_(data)
{
  /* Nothing to do here. */
}

template<class NoiseType>
State<NoiseType>::State(unsigned int id,
                        std::vector<unsigned int> nearest_neighbors,
                        NoiseType noise)
  : id_(id)
  , data_(nearest_neighbors.size(), arma::fill::zeros)
{
  leader_ = sim_interface_->positions().begin();
  follower_ = sim_interface_->positions().begin() + id;

  neighbor_dists_3D_ = dist_.distances_to_neighbors(
    id, nearest_neighbors, sim_interface_->positions());

  data_ = mtools_.map_to_arma(neighbor_dists_3D_);
  data_ = noise.apply_noise(data_);
  double alti_diff = (leader_->Z() - follower_->Z());
  data_.resize(data_.n_rows + 1);
  data_.at(data_.n_rows - 1) = alti_diff;
}

template<class NoiseType>
State<NoiseType>::State(std::shared_ptr < sim_interface,
                        unsigned int id,
                        std::vector<unsigned int> nearest_neighbors)
  : id_(id)
  , data_(nearest_neighbors.size(), arma::fill::zeros)
{
  leader_ = sim_interface_->positions().begin();
  follower_ = sim_interface_->positions().begin() + id;

  neighbor_dists_3D_ = dist_.distances_to_neighbors(
    id, nearest_neighbors, sim_interface_->positions());

  data_ = mtools_.map_to_arma(neighbor_dists_3D_);
  double alti_diff = (leader_->Z() - follower_->Z());
  data_.resize(data_.n_rows + 1);
  data_.at(data_.n_rows - 1) = alti_diff;
}

template<class NoiseType>
arma::colvec
State<NoiseType>::Data() const
{
  return data_;
}

template<class NoiseType>
arma::colvec&
State<NoiseType>::Data()
{
  return data_;
}

template<class NoiseType>
arma::colvec
State<NoiseType>::Distances() const
{
  return data_;
}

template<class NoiseType>
arma::colvec&
State<NoiseType>::Distances()
{
  return data_;
}

template<class NoiseType>
std::map<unsigned int, double>
State<NoiseType>::neighbor_dists_3D() const
{
  return neighbor_dists_3D_;
}

template<class NoiseType>
const arma::colvec&
State<NoiseType>::Encode() const
{
  return data_;
}

template<class NoiseType>
inline std::ostream&
operator<<(std::ostream& out, const State<NoiseType>& s)
{
  out << s.Distances() << "," << s.AltitudeDiff();
  return out;
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

template<class NoiseType>
inline bool
operator==(const State<NoiseType>& s, const State<NoiseType>& s1)
{
  bool result_distance = std::equal(s.Distances().begin(),
                                    s.Distances().end(),
                                    s1.Distances().begin(),
                                    [](const double& d, const double& d1) {
                                      bool result = ILMR::comparator(d, d1);
                                      return result;
                                    });

  bool result_height = comparator(s.AltitudeDiff(), s1.AltitudeDiff());

  bool result = false;
  if (result_distance and result_height) {
    result = true;
  }
  return result;
}
