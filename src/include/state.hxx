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
                                        const ContainerType& container,
                                        FilterType filter)
  : id_(id)
  , data_(container.size(), arma::fill::zeros)
{
  data_.resize(2 * container.size());
  std::vector<double> data;

  for (std::size_t i = 0; i < container.size(); ++i) {
    data.push_back(container.at(i).antenna_1);
    data.push_back(container.at(i).antenna_2);
  }
  data_ = arma.vec_to_arma(data);
  data_ = filter.input(data_);
}

template<class FilterType, class ContainerType>
State<FilterType, ContainerType>::State(unsigned int id,
                                        const ContainerType& container)
  : id_(id)
  , data_(container.size(), arma::fill::zeros)
{
  data_.resize(2 * container.size());
  std::vector<double> data;
  for (std::size_t i = 0; i < container.size(); ++i) {
    data.push_back(container.at(i).antenna_1);
    data.push_back(container.at(i).antenna_2);
  }
  data_ = arma.vec_to_arma(data);
  //std::cout << data_ << "\n";
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
