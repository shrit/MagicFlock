#pragma once

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
State<FilterType, NoiseType, StateType, ContainerType>::State()
  : data_(dimension, arma::fill::zeros)
  , leader_data_(3, arma::fill::zeros)
  , followers_data_((dimension - 3), arma::fill::zeros)
  , reduced_data_(6, arma::fill::zeros)
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
  , leader_data_(3, arma::fill::zeros)
  , followers_data_((dimension - 3), arma::fill::zeros)
  , reduced_data_(6, arma::fill::zeros)
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
  , leader_data_(3, arma::fill::zeros)
  , followers_data_((dimension - 3), arma::fill::zeros)
  , reduced_data_(6, arma::fill::zeros)
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
    arma::colvec sorted_data(antenna_size, arma::fill::zeros);
    for (std::size_t j = 0; j < num_of_transmitter; ++j) {
      data.at(i) = container.at(j).dist_antenna_1;
      data.at(++i) = container.at(j).dist_antenna_2;

      i = i + 1;
    }
    sorted_data = data;
    arma::sort(sorted_data);
    reduced_data_.at(0) = sorted_data.at(0);
    reduced_data_.at(1) = sorted_data.at(1);
    reduced_data_.at(2) = sorted_data.at(2);
    reduced_data_.at(3) = sorted_data.at(3);
    reduced_data_.at(4) = sorted_data.at(12);
    reduced_data_.at(5) = sorted_data.at(13);

    data_ = arma.vec_to_arma(data);

  } else if constexpr (std::is_same<StateType, FullWiFi>::value) {
    // Extracting leader information
    for (std::size_t j = 0; j < num_neighbors_; ++j) {
      if (container.at(j).id == 0) {
        data.resize(3);
        data.at(i) = container.at(j).antenna;
        data.at(++i) = container.at(j).azimuth;
        data.at(++i) = container.at(j).elevation;
      }
      leader_data_ = arma.vec_to_arma(data);
    }
    // Extracting followers information
    i = 0;
    data.reset();
    data.resize((container.size() - 1) * 3);
    for (std::size_t j = 0; j < (num_neighbors_ - 1); ++j) {
      if (container.at(j).id != 0) {
        data.at(i) = container.at(j).antenna;
        data.at(++i) = container.at(j).azimuth;
        data.at(++i) = container.at(j).elevation;
        i = i + 1;
      }
      followers_data_ = arma.vec_to_arma(data);
    }
    data_ = arma::join_cols(leader_data_, followers_data_);

  } else if constexpr (std::is_same<StateType, CrapyData>::value) {
    arma::colvec crapy_dataset = { -120, -120, -120, -120, -120, -120,
                                   -120, -120, -120, -120, -120, -120,
                                   -120, -120, -120, -120 };
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
State<FilterType, NoiseType, StateType, ContainerType>::leader_data() const
{
  return leader_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::leader_data()
{
  return leader_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec
State<FilterType, NoiseType, StateType, ContainerType>::followers_data() const
{
  return followers_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::followers_data()
{
  return followers_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec
State<FilterType, NoiseType, StateType, ContainerType>::ReducedData() const
{
  return reduced_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
arma::colvec&
State<FilterType, NoiseType, StateType, ContainerType>::ReducedData()
{
  return reduced_data_;
}

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
inline std::ostream&
operator<<(std::ostream& out,
           const State<FilterType, NoiseType, StateType, ContainerType>& s)
{
  out << s.Data();
  return out;
}
