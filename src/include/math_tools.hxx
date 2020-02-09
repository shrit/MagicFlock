#pragma once

template<typename T>
double
Math_tools::deformation_error_one_follower(lt::triangle<T> old_dist,
                                           lt::triangle<T> new_dist)
{
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);
  double error = diff_f1 + diff_f2;
  /*  Recalculate the Error between quadcopters  */
  return error;
}

template<typename T>
double
Math_tools::gaussian_noise(std::vector<lt::triangle<T>> distances,
                           std::vector<T> drift_f3)
{
  std::vector<double> ideal_f3;

  std::transform(distances.begin(),
                 distances.end(),
                 std::back_inserter(ideal_f3),
                 [](lt::triangle<double> const& t) { return t.f3; });

  std::adjacent_difference(
    ideal_f3.begin(), ideal_f3.end(), std::back_inserter(drift_f3));

  /* The difference in distances needs to be in absolute value */
  /*  This has a tremendous cost since we need to re */
  double (*fabs)(double) = &std::fabs;
  std::transform(drift_f3.begin(), drift_f3.end(), drift_f3.begin(), fabs);

  // adding one here to remove the first element of adjacent difference
  double noise_mean =
    std::accumulate(drift_f3.begin() + 1, drift_f3.end(), 0.0) /
    drift_f3.size();
  return noise_mean;
}

template<typename T>
long long unsigned int
Math_tools::index_of_max_value(const std::vector<T>& vec)
{
  auto highest = std::max_element(vec.begin(), vec.end());
  return std::distance(vec.begin(), highest);
}

template<typename T>
long long unsigned int
Math_tools::index_of_min_value(const std::vector<T>& vec)
{
  auto highest = std::min_element(vec.begin(), vec.end());
  return std::distance(vec.begin(), highest);
}

template<typename T>
bool
Math_tools::is_good_shape(unsigned int id,
                          std::vector<unsigned int> nearest_neighbors,
                          std::vector<lt::position3D<T>> positions)
{
  bool value = true;
  std::vector<double> distances =
    map_to_vector(distances_to_neighbors(id, nearest_neighbors, positions));
  logger::logger_->debug("Distances to other quadrotors: {} ", distances);
  if (std::any_of(distances.begin(), distances.end(), [&](const double& i) {
        if ((i < lower_threshold_) and (i > upper_threshold_)) {
          return true;
        } else
          return false;
      })) {
    value = false;
  }
  return value;
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
std::vector<bool>
Math_tools::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
{
  std::vector<bool> one_hot(number_of_class, 0);

  if (number_of_class > static_cast<int>(arg)) {
    one_hot.at(static_cast<int>(arg)) = 1;
  } else {
    logger::logger_->error(
      "Can not convert to one hot, please add more classes...");
  }
  return one_hot;
}

template<typename Arg>
int
Math_tools::from_one_hot_encoding(std::vector<Arg> values)
{
  auto it = std::find(values.begin(), values.end(), 1);
  int index = std::distance(values.begin(), it);
  return index;
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
  // Can not remember the reason for this reverse, verify for ann
  std::reverse(vec.begin(), vec.end());
  logger::logger_->info("Converted to std vector: {}", vec);
  return vec;
}

template<typename T1, typename T2>
std::vector<T2>
Math_tools::map_to_vector(std::map<T1, T2> m)
{
  std::vector<T2> vec;
  for (typename std::map<T1, T2>::iterator it = m.begin(); it != m.end(); ++it) {
    vec.push_back(it->second);
  }
  return vec;
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

template<typename Arg>
Arg
Math_tools::mean(std::vector<Arg> vec)
{
  size_t sz = vec.size();
  if (sz == 0)
    return -1;

  return std::accumulate(vec.begin(), vec.end(), 0.0) / vec.size();
}

template<typename Arg>
Arg
Math_tools::variance(std::vector<Arg> vec)
{
  /*  note that diff_f3_ was used here */
  size_t sz = vec.size();
  if (sz == 1)
    return -1;

  /*  Do not take the first value */
  Arg mean = std::accumulate(vec.begin() + 1, vec.end(), 0.0) / vec.size();

  return std::accumulate(vec.begin(),
                         vec.end(),
                         0.0,
                         [&mean, &sz](double accumulator, const double& val) {
                           return accumulator +
                                  ((val - mean) * (val - mean) / (sz - 1));
                         });
}
