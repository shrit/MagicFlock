#pragma once

template<class NoiseType, class ContainerType>
State<NoiseType>::State()
  : data_(dimension, arma::fill::zeros)
{
  /* Nothing to do here. */
}

template<class NoiseType, class ContainerType>
State<NoiseType>::State(const arma::colvec& data)
  : data_(data)
{
  /* Nothing to do here. */
}

template<class NoiseType, class ContainerType>
State<NoiseType>::State(unsigned int id,
                        ContainerType container
                        NoiseType noise)
  : id_(id)
  , data_(nearest_neighbors.size(), arma::fill::zeros)
{
  data_ = mtools_.map_to_arma(neighbor_dists_3D_);
  data_ = noise.apply_noise(data_);
}

template<class NoiseType, class ContainerType>
State<NoiseType>::State(unsigned int id, ContainerType container)
  : id_(id)
  , data_(nearest_neighbors.size(), arma::fill::zeros)
{
  data_.resize(2 * container.size()); 
  std::vector<double> data;

  for (std::size_t i; i < container.size(); ++i) {
    data.push_back(container.at(i).antenna_1);
    data.push_back(container.at(i).antenna_2);
  }
  data_ = Vec.
}

template<class NoiseType, class ContainerType>
arma::colvec
State<NoiseType>::Data() const
{
  return data_;
}

template<class NoiseType, class ContainerType>
arma::colvec&
State<NoiseType>::Data()
{
  return data_;
}

template<class NoiseType, class ContainerType>
arma::colvec
State<NoiseType>::RSSI() const
{
  return data_;
}

template<class NoiseType, class ContainerType>
arma::colvec&
State<NoiseType>::RSSI()
{
  return data_;
}

template<class NoiseType, class ContainerType>
arma::colvec
State<NoiseType>::TOAs() const
{
  return data_;
}

template<class NoiseType, class ContainerType>
arma::colvec&
State<NoiseType>::TOAs()
{
  return data_;
}

template<class NoiseType, class ContainerType>
std::map<unsigned int, double>
State<NoiseType>::neighbor_dists_3D() const
{
  return neighbor_dists_3D_;
}

template<class NoiseType, class ContainerType>
const arma::colvec&
State<NoiseType>::Encode() const
{
  return data_;
}

template<class NoiseType, class ContainerType>
inline std::ostream&
operator<<(std::ostream& out, const State<NoiseType>& s)
{
  out << s.RSSI() << "," << s.TOAs();
  return out;
}

