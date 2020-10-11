#pragma once

template<class FilterType, class ContainerType>
State<FilterType, ContainerType>::State()
  : data_(dimension, arma::fill::zeros)
{
  /* Nothing to do here. */
}

template<class FilterType, class ContainerType>
State<FilterType, ContainerType>::State(const arma::colvec& data)
  : data_(data)
{
  /* Nothing to do here. */
}

template<class FilterType, class ContainerType>
State<FilterType, ContainerType>::State(unsigned int id,
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

  ILMR::logger::logger_->info("Data before filtering:  {}", data_);
  data_ = filter.input(data_);
  ILMR::logger::logger_->info("Data after filtering:  {}", data_);
}

template<class FilterType, class ContainerType>
arma::colvec
State<FilterType, ContainerType>::Data() const
{
  Encode();
  return data_;
}

template<class FilterType, class ContainerType>
arma::colvec&
State<FilterType, ContainerType>::Data()
{
  return data_;
}

template<class FilterType, class ContainerType>
arma::colvec
State<FilterType, ContainerType>::RSSI() const
{
  return rssi_data_;
}

template<class FilterType, class ContainerType>
arma::colvec&
State<FilterType, ContainerType>::RSSI()
{
  return rssi_data_;
}

template<class FilterType, class ContainerType>
arma::colvec
State<FilterType, ContainerType>::TOAs() const
{
  return toa_data_;
}

template<class FilterType, class ContainerType>
arma::colvec&
State<FilterType, ContainerType>::TOAs()
{
  return toa_data_;
}

template<class FilterType, class ContainerType>
const arma::colvec&
State<FilterType, ContainerType>::Encode() const
{
  data_ = arma::join_cols(rssi_data_, toa_data_);
}

template<class FilterType, class ContainerType>
inline std::ostream&
operator<<(std::ostream& out, const State<FilterType, ContainerType>& s)
{
  out << s.RSSI() << "," << s.TOAs();
  return out;
}
