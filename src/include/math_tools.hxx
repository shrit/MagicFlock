#pragma once

template<typename T>
bool
Math_tools::is_good_shape(unsigned int id,
                          std::vector<unsigned int> nearest_neighbors,
                          std::vector<lt::position3D<T>> positions)
{
  bool good_shape = true;
  std::vector<double> distances =
    map_to_vector(distances_to_neighbors(id, nearest_neighbors, positions));
  logger::logger_->debug("Distances to other quadrotors: {} ", distances);
  if (std::any_of(distances.begin(), distances.end(), [&](const double& i) {
        if ((i < lower_threshold_) or (i > upper_threshold_)) {
          return true;
        } else
          return false;
      })) {
    good_shape = false;
  }
  return good_shape;
}

template<typename T>
void
Math_tools::histogram(T times)
{
  auto it = histo_.find(times);
  if (it != histo_.end()) {
    it->second = it->second + 1;
  } else {
    histo_.insert(std::pair<T, T>(times, 1));
  }
}

template<typename T>
std::map<T, T>
Math_tools::get_histogram()
{
  return histo_;
}

template<typename KeyType, typename ValueType>
std::pair<KeyType, ValueType>
Math_tools::get_max_histogram(const std::map<KeyType, ValueType>& x)
{
  return *std::max_element(x.begin(),
                           x.end(),
                           [](const std::pair<KeyType, ValueType>& p1,
                              const std::pair<KeyType, ValueType>& p2) {
                             return p1.second < p2.second;
                           });
}

/*  Simple implementation, need more logical one */
template<typename Arg, typename Arg2>
arma::colvec
Math_tools::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
{
  arma::colvec one_hot(number_of_class, arma::fill::zeros);

  if (number_of_class > static_cast<int>(arg)) {
    one_hot.at(static_cast<int>(arg)) = 1;
  } else {
    logger::logger_->error(
      "Can not convert to one hot, please add more classes...");
  }
  return one_hot;
}

template<typename Arg>
arma::uword
Math_tools::from_one_hot_encoding(arma::Col<Arg> col_vec)
{
  return index = col_vec.index_max();
}

template<typename Arg>
bool
Math_tools::hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2)
{
  bool distance = false;
  if (v1.size() == v2.size()) {
    if (v1 == v2) {
      distance = true;
    }
  } else {
    logger::logger_->error(
      "Can not calculate hamming on different size vectors");
  }
  return distance;
}

template<typename T>
int
Math_tools::ecludian_distance(T d1, T d2)
{
  double distance = std::sqrt(std::pow((d1), 2) - std::pow((d2), 2));
  return distance;
}

template<typename Arg>
std::vector<double>
Math_tools::to_std_vector(Arg arg)
{
  std::vector<double> vec;
  if (arg.is_empty()) {
    logger::logger_->error("Can not convert empty vector to std vector");
  }
  if (std::is_same<Arg, arma::rowvec>::value) {
    vec.resize(arg.n_cols);
    vec = arma::conv_to<std::vector<double>>::from(arg);
  } else if (std::is_same<Arg, arma::colvec>::value) {
    vec.resize(arg.n_rows);
    vec = arma::conv_to<std::vector<double>>::from(arg);
  }
  logger::logger_->debug("Converted to std vector: {}", vec);
  return vec;
}

template<typename T1, typename T2>
std::vector<T2>
Math_tools::map_to_vector(std::map<T1, T2> m)
{
  std::vector<T2> vec;
  for (typename std::map<T1, T2>::iterator it = m.begin(); it != m.end();
       ++it) {
    vec.push_back(it->second);
  }
  return vec;
}

template<typename T1, typename T2>
arma::colvec
Math_tools::map_to_arma(std::map<T1, T2> m)
{
  std::vector<T2> vec;
  for (typename std::map<T1, T2>::iterator it = m.begin(); it != m.end();
       ++it) {
    vec.push_back(it->second);
  }
  arma::colvec arma_vec = arma::conv_to<arma::colvec>::from(vec);

  return arma_vec;
}

template<typename T>
double
Math_tools::distance_a_2_b(std::vector<lt::position3D<T>> positions,
                           unsigned int id_a,
                           unsigned int id_b)
{
  lt::position3D<double> dist;
  /*  Distance between a and b */
  dist.x = positions.at(id_a).x - positions.at(id_b).x;
  dist.y = positions.at(id_a).y - positions.at(id_b).y;
  dist.z = positions.at(id_a).z - positions.at(id_b).z;

  double distance = std::sqrt(std::pow((dist.x), 2) + std::pow((dist.y), 2) +
                              std::pow((dist.z), 2));
  return distance;
}

template<typename T>
std::map<unsigned int, double>
Math_tools::distances_to_neighbors(unsigned int id,
                                   std::vector<unsigned int> nearest_neighbors,
                                   std::vector<lt::position3D<T>> positions)
{
  std::map<unsigned int, double> distances;
  for (auto&& i : nearest_neighbors) {
    distances.insert({ i, distance_a_2_b(positions, id, i) });
  }
  return distances;
}

template<typename T>
double
Math_tools::traveled_distances(lt::position3D<T> pos_t,
                               lt::position3D<T> pos_t_1)
{
  lt::position3D<double> dist;

  /* Travelled distance between time steps */
  dist.x = pos_t.x - pos_t_1.x;
  dist.y = pos_t.y - pos_t_1.y;
  dist.z = pos_t.z - pos_t_1.z;

  double traveled_distance;

  /*  Summing up vectors, the total is the distance travelled by each
      agent during each trajectory */

  /*  Distance travelled by the leader */
  traveled_distance = std::sqrt(std::pow((dist.x), 2) + std::pow((dist.y), 2) +
                                std::pow((dist.z), 2));
  return traveled_distance;
}

template<typename T>
T
Math_tools::pythagore_leg(T leg, T hypotenuse)
{
  T leg_2 = std::sqrt(std::pow(hypotenuse, 2) - std::pow(leg, 2));
  return leg_2;
}

template<typename T>
T
Math_tools::pythagore_hypotenuse(T leg_1, T leg_2)
{
  T hypotenuse = std::sqrt(std::pow(leg_1, 2) + std::pow(leg_2, 2));
  return hypotenuse;
}

