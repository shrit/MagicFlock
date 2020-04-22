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

  data_ = mtools_.map_to_arma(neighbor_dists_3D_);
  data_ = noise.apply_noise(data_);

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

  data_ = mtools_.map_to_arma(neighbor_dists_3D_);
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
State<NoiseType>::RSSI() const
{
  return data_;
}

template<class NoiseType>
arma::colvec&
State<NoiseType>::RSSI()
{
  return data_;
}

template<class NoiseType>
arma::colvec
State<NoiseType>::TOAs() const
{
  return data_;
}

template<class NoiseType>
arma::colvec&
State<NoiseType>::TOAs()
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
  out << s.RSSI() << "," << s.TOAs();
  return out;
}

