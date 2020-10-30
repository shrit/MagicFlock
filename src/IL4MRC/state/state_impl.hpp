#pragma once

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
State<FilterType, NoiseType, StateType, ContainerType>::State()
  : data_(dimension, arma::fill::zeros)
{
  /* Nothing to do here. */
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
State<FilterType, NoiseType, StateType, ContainerType>::State(
  const arma::colvec& data)
  : data_(data)
{
  /* Nothing to do here. */
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
State<FilterType, NoiseType, StateType, ContainerType>::State(
  unsigned int id,
  int num_neighbors,
  int num_of_antenna_src,
  const ContainerType& container,
  FilterType filter)
  : id_(id)
  , num_neighbors_(num_neighbors)
  , data_(num_neighbors_ * 2, arma::fill::zeros)
{
  int antenna_size = (num_neighbors_ + num_of_antenna_src) * 2;
  int num_of_transmitter = num_neighbors_ + num_of_antenna_src;
  std::vector<double> data(antenna_size);
  int i = 0;
  if constexpr (std::is_same<StateType, ReceivedSignal>()) {

    for (std::size_t j = 0; j < num_of_transmitter; ++j) {

      if (container.at(j).id != id_) {
        data.at(i) = container.at(j).antenna_1;
        data.at(++i) = container.at(j).antenna_2;
        i = i + 1;
      }
    }

    data_ = arma.vec_to_arma(data);
    // If a neighbor is out of range, put -110 instead of 0
    data_.replace(0, -110);

  } else if constexpr (std::is_same<StateType, AntennaDists>::value) {

    for (std::size_t j = 0; j < num_of_transmitter; ++j) {
      data.at(i) = container.at(j).dist_antenna_1;
      data.at(++i) = container.at(j).dist_antenna_2;
      i = i + 1;
    }
    data_ = arma.vec_to_arma(data);

  } else if constexpr (std::is_same<StateType, CrapyData>::value) {
    arma::colvec crapy_dataset = { -120, -120, -120, -120, -120, -120,
                                   -120, -120, -120, -120, -120, -120 };
    data_ = crapy_dataset;
  }

  NoiseType noise;
  noise.mean() = 0;
  noise.standard_deviation() = 6;
  ILMR::logger::logger_->debug("Data before filtering:  {}", data_);
  data_ = filter.input(data_);
  data_ = noise.apply_noise(data_);
  ILMR::logger::logger_->debug("Data after filtering:  {}", data_);
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec
State<FilterType, NoiseType, StateType, ContainerType>::Data() const
{
  Encode();
  return data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::Data()
{
  return data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec
State<FilterType, NoiseType, StateType, ContainerType>::RSSI() const
{
  return rssi_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::RSSI()
{
  return rssi_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec
State<FilterType, NoiseType, StateType, ContainerType>::TOAs() const
{
  return toa_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::TOAs()
{
  return toa_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
const arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::Encode() const
{
  data_ = arma::join_cols(rssi_data_, toa_data_);
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
inline std::ostream&
operator<<(std::ostream& out,
           const State<FilterType, NoiseType, StateType, ContainerType>& s)
{
  out << s.RSSI() << "," << s.TOAs();
  return out;
}
