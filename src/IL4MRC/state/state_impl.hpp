#pragma once

template<class FilterType, class NoiseType, class ContainerType>
State<FilterType, NoiseType, ContainerType>::State()
  : data_(dimension, arma::fill::zeros)
{
  /* Nothing to do here. */
}

template<class FilterType, class NoiseType, class ContainerType>
State<FilterType, NoiseType, ContainerType>::State(const arma::colvec& data)
  : data_(data)
{
  /* Nothing to do here. */
}

template<class FilterType, class NoiseType, class ContainerType>
State<FilterType, NoiseType, ContainerType>::State(
  unsigned int id,
  int num_neighbors,
  const ContainerType& container,
  FilterType filter)
  : id_(id)
  , num_neighbors_(num_neighbors)
  , data_(num_neighbors_ * 2, arma::fill::zeros)
{
  int antenna_size = num_neighbors_ * 2;
  std::vector<double> data(antenna_size);
  int i = 0;
  for (std::size_t j = 0; j < num_neighbors_; ++j) {

    if (container.at(j).id != id_) {
      data.at(i) = container.at(j).antenna_1;
      data.at(++i) = container.at(j).antenna_2;
      i = i + 1;
    }
  }

  data_ = arma.vec_to_arma(data);
  data_.replace(0, -110);
  NoiseType noise;
  noise.mean() = 0;
  noise.standard_deviation() = 6;
  ILMR::logger::logger_->debug("Data before filtering:  {}", data_);
  data_ = filter.input(data_);
  data_ = noise.apply_noise(data_);
  ILMR::logger::logger_->debug("Data after filtering:  {}", data_);
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec
State<FilterType, NoiseType, ContainerType>::Data() const
{
  Encode();
  return data_;
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec&
State<FilterType, NoiseType, ContainerType>::Data()
{
  return data_;
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec
State<FilterType, NoiseType, ContainerType>::RSSI() const
{
  return rssi_data_;
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec&
State<FilterType, NoiseType, ContainerType>::RSSI()
{
  return rssi_data_;
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec
State<FilterType, NoiseType, ContainerType>::TOAs() const
{
  return toa_data_;
}

template<class FilterType, class NoiseType, class ContainerType>
arma::colvec&
State<FilterType, NoiseType, ContainerType>::TOAs()
{
  return toa_data_;
}

template<class FilterType, class NoiseType, class ContainerType>
const arma::colvec&
State<FilterType, NoiseType, ContainerType>::Encode() const
{
  data_ = arma::join_cols(rssi_data_, toa_data_);
}

template<class FilterType, class NoiseType, class ContainerType>
inline std::ostream&
operator<<(std::ostream& out,
           const State<FilterType, NoiseType, ContainerType>& s)
{
  out << s.RSSI() << "," << s.TOAs();
  return out;
}
