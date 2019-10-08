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
index_of_highest_value(const std::vector<T>& vec)
{
  // Find Smallest Value in vec
    auto highest = std::max_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), highest);
}

template <typename T>
bool Math_tools::
is_triangle(lt::triangle<T> t)
{
  bool value = false;
  if((t.f1 + t.f2 >  lower_threshold_.at(0))  and
     (t.f1 + t.f2 <  upper_threshold_.at(0))) {
    if ((t.f1 + t.f3 >  lower_threshold_.at(1))  and
	(t.f1 + t.f3 <  upper_threshold_.at(1))) {
      if ((t.f2 + t.f3 >  lower_threshold_.at(2))  and
	  (t.f2 + t.f3 <  upper_threshold_.at(2)))
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

template <typename T>
lt::triangle<double> Math_tools::
triangle_side_2D(lt::positions<lt::position3D<T>> pos)
{
    lt::position3D<double> dist, dist2, dist3;

    /*  Distance between leader and FF */
    dist.x =  pos.leader.x - pos.f1.x;
    dist.y =  pos.leader.y - pos.f1.y;

    /* Distance between leader and TF */
    dist2.x = pos.leader.x - pos.f2.x;
    dist2.y = pos.leader.y - pos.f2.y;

    /* Distance between TF and FF */
    dist3.x = pos.f1.x - pos.f2.x;
    dist3.y = pos.f1.y - pos.f2.y;

    lt::triangle<double> t;

    t.f3 = std::sqrt(std::pow((dist.x), 2) +
		     std::pow((dist.y), 2));

    t.f1 = std::sqrt(std::pow((dist2.x), 2)+
		     std::pow((dist2.y), 2));

    t.f2 = std::sqrt(std::pow((dist3.x), 2)+
		     std::pow((dist3.y), 2));
    /*  it return the traingle side */
    return t;
}

template <typename T>
lt::triangle<double> Math_tools::
triangle_side_3D(lt::positions<lt::position3D<T>> pos)
{
    lt::position3D<double> dist, dist2, dist3;

    /*  Distance between leader and FF */
    dist.x =  pos.leader.x - pos.f1.x;
    dist.y =  pos.leader.y - pos.f1.y;
    dist.z =  pos.leader.z - pos.f1.z;

    /* Distance between leader and TF */
    dist2.x = pos.leader.x - pos.f2.x;
    dist2.y = pos.leader.y - pos.f2.y;
    dist2.z = pos.leader.z - pos.f2.z;

    /* Distance between TF and FF */
    dist3.x = pos.f1.x - pos.f2.x;
    dist3.y = pos.f1.y - pos.f2.y;
    dist3.z = pos.f1.z - pos.f2.z;

    lt::triangle<double> t;

    t.f3 = std::sqrt(std::pow((dist.x), 2) +
		     std::pow((dist.y), 2) +
		     std::pow((dist.z), 2));

    t.f1 = std::sqrt(std::pow((dist2.x), 2) +
		     std::pow((dist2.y), 2) +
		     std::pow((dist2.z), 2));

    t.f2 = std::sqrt(std::pow((dist3.x), 2) +
		     std::pow((dist3.y), 2) +
		     std::pow((dist3.z), 2));
    /*  it return the traingle side */
    return t;
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

template <typename T>
lt::dist3D<double> Math_tools::traveled_distances(lt::positions<lt::position3D<T>> pos_t,
						     lt::positions<lt::position3D<T>> pos_t_1)
{
  lt::position3D<double> dist_leader, dist_f1, dist_f2;

    /* Travelled distance between time steps for leader  */
  dist_leader.x =  pos_t.leader.x - pos_t_1.leader.x;
  dist_leader.y =  pos_t.leader.y - pos_t_1.leader.y;
  dist_leader.z =  pos_t.leader.z - pos_t_1.leader.z;
  
  /* Travelled distance between time steps for f1  */
  dist_f1.x = pos_t.f1.x - pos_t_1.f1.x;
  dist_f1.y = pos_t.f1.y - pos_t_1.f1.y;
  dist_f1.z = pos_t.f1.z - pos_t_1.f1.z;
  
  /* Travelled distance between time steps for f2  */
  dist_f2.x = pos_t.f2.x - pos_t_1.f2.x;
  dist_f2.y = pos_t.f2.y - pos_t_1.f2.y;
  dist_f2.z = pos_t.f2.z - pos_t_1.f2.z;

  lt::dist3D<double> distance3D;

  /*  Summing up vectors, the total is the distance travelled by each
      agent during each trajectory */
  
  /*  Distance travelled by the leader */
  distance3D.d1 = std::sqrt(std::pow((dist_leader.x), 2) +
			    std::pow((dist_leader.y), 2) +
			    std::pow((dist_leader.z), 2));

  /*  Distance travelled by the f1 */
  distance3D.d2 = std::sqrt(std::pow((dist_f1.x), 2) +
			    std::pow((dist_f1.y), 2) +
			    std::pow((dist_f1.z), 2));

  /*  Distance travelled by the f2 */
  distance3D.d3 = std::sqrt(std::pow((dist_f2.x), 2) +
			    std::pow((dist_f2.y), 2) +
			    std::pow((dist_f2.z), 2));
  
  return distance3D;
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
