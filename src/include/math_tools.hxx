#pragma once

# include "math_tools.hh"

template <typename T>
double Math_tools::deformation_error_one_follower(lt::triangle<T> old_dist,
						  lt::triangle<T> new_dist)
{
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);

  double error = diff_f1 + diff_f2;

  /*  Recalculate the Error between quadcopters  */
  return error;
}

template <typename T>
double Math_tools::gaussian_noise(std::vector<lt::triangle<T>> distances,
				  std::vector<T> drift_f3)
{
  std::vector<double> ideal_f3;

  std::transform(distances.begin(), distances.end(), std::back_inserter(ideal_f3),
		 [](lt::triangle<double> const& t) { return t.f3; });

  std::adjacent_difference(ideal_f3.begin(), ideal_f3.end(), std::back_inserter(drift_f3));

  /* The difference in distances needs to be in absolute value */
  /*  This has a tremendous cost since we need to re */
  double (*fabs)(double) = &std::fabs;
  std::transform(drift_f3.begin(), drift_f3.end(), drift_f3.begin(), fabs);

  // adding one here to remove the first element of adjacent difference
  double noise_mean = std::accumulate(drift_f3.begin() + 1,
				   drift_f3.end(), 0.0)/drift_f3.size();
  return noise_mean;
}

template <typename T>
long long unsigned int Math_tools::
index_of_max_value(const std::vector<T>& vec)
{
    auto highest = std::max_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), highest);
}

template <typename T>
long long unsigned int Math_tools::
index_of_min_value(const std::vector<T>& vec)
{
    auto highest = std::min_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), highest);
}

bool Math_tools::
is_good_shape(unsigned int id,                               
	      std::vector<unsigned int> nearest_neighbors,   
	      std::vector<lt::position3D<double>> positions)
{
  bool value = false;
  std::vector<double> distances = distances_to_neighbors(id, nearest_neighbors, positions);
  for (auto&& i : distances) {
    if((i > lower_threshold_)  and
       (i < upper_threshold_)) {
      value = true;
    }
  }
  return value;
}

template <typename T>
void Math_tools::
histogram(T times)
{
  auto it = histo_.find(times);

  if (it != histo_.end()) {
    it->second = it->second + 1;
  } else {
    histo_.insert(std::pair<T, T>(times, 1));
  }
}

template <typename T>
std::map<T, T> Math_tools::
get_histogram()
{
  return histo_;
}

/*  Simple implementation, need more logical one */
template <typename Arg, typename Arg2>
std::vector<bool> Math_tools::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
{
  std::vector<bool> one_hot (number_of_class, 0);
  // // to check
  // if constexpr (std::is_same<Arg,
  // 		typename std::underlying_type<Quadcopter<Arg>::Action>::type>::value) {
  if( number_of_class > static_cast<int>(arg)) {
    one_hot.at(static_cast<int>(arg)) = 1 ;
  } else {
    LogErr() << "Can not convert to one hot, please add more classes..." ;
  }
  return one_hot;
}

template <typename Arg>
std::vector<double> Math_tools::to_std_vector(Arg arg)
{
  std::vector<double> vec;
  if (arg.is_empty()) {
    LogInfo() << "Can not convert empty vector to std vector";
  }
  if (std::is_same<Arg, arma::rowvec>::value) {
    vec.resize(arg.n_cols);
    vec = arma::conv_to<std::vector<double>>::from(arg);    
  } else if (std::is_same<Arg, arma::colvec>::value) {
    vec.resize(arg.n_rows);
    vec = arma::conv_to<std::vector<double>>::from(arg);
  }
  std::reverse(vec.begin(), vec.end());
  LogInfo() << "Vector control_prediction: " << vec;
  return vec;
}

double Math_tools::distance_a_2_b(std::vector<lt::position3D<double>> positions,
				  unsigned int id_a,
				  unsigned int id_b)
{
  lt::position3D<double> dist;
  /*  Distance between a and b */
    dist.x = positions.at(id_a).x - positions.at(id_b).x;
    dist.y = positions.at(id_a).y - positions.at(id_b).y;
    dist.z = positions.at(id_a).z - positions.at(id_b).z;
    
    double distance = std::sqrt(std::pow((dist.x), 2) +
				std::pow((dist.y), 2) +
				std::pow((dist.z), 2));    
    return distance;
}

std::vector<double> Math_tools::distances_to_neighbors(unsigned int id,
						       std::vector<unsigned int> nearest_neighbors,
						       std::vector<lt::position3D<double>> positions)
{
  std::vector<double> distances;
  for (auto&& i : nearest_neighbors) {
    distances.push_back(distance_a_2_b(positions, id, i));
  }
  return distances;
}

template <typename T>
double Math_tools::traveled_distances(lt::position3D<T> pos_t,
				      lt::position3D<T> pos_t_1)
{
  lt::position3D<double> dist;

    /* Travelled distance between time steps */
  dist.x =  pos_t.x - pos_t_1.x;
  dist.y =  pos_t.y - pos_t_1.y;
  dist.z =  pos_t.z - pos_t_1.z;
  
  double traveled_distance;

  /*  Summing up vectors, the total is the distance travelled by each
      agent during each trajectory */
  
  /*  Distance travelled by the leader */
  traveled_distance = std::sqrt(std::pow((dist.x), 2) +
				std::pow((dist.y), 2) +
				std::pow((dist.z), 2));  
  return traveled_distance;
}

template <typename T>
T Math_tools::pythagore_leg(T leg, T hypotenuse)
{
  T leg_2 = std::sqrt( std::pow(hypotenuse, 2)
		       - std::pow(leg, 2) ) ;
  return leg_2;
}

template <typename T>
T Math_tools::pythagore_hypotenuse(T leg_1, T leg_2)
{
  T hypotenuse = std::sqrt( std::pow(leg_1, 2)
			    + std::pow(leg_2, 2) ) ;
  return hypotenuse;
}

template <typename Arg>
Arg Math_tools::mean(std::vector<Arg> vec)
{
  size_t sz = vec.size();
  if (sz == 0)
    return -1;

  return std::accumulate(vec.begin() , vec.end(),
			 0.0)/vec.size();
}

template <typename Arg>
Arg Math_tools::variance(std::vector<Arg> vec)
{
  /*  note that diff_f3_ was used here */
  size_t sz = vec.size();
  if (sz == 1)
    return -1;

  /*  Do not take the first value */
  Arg mean = std::accumulate(vec.begin() + 1,
			     vec.end(), 0.0)/vec.size();

  return std::accumulate(vec.begin(), vec.end(), 0.0,
			 [&mean, &sz](double accumulator, const double& val) {
			   return accumulator +
			     ((val - mean)*(val - mean) / (sz - 1));
			 } );
}
