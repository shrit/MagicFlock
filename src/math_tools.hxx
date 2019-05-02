# include "math_tools.hh"


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
index_of_smallest_value(const std::vector<T>& vec)
{ 
  // Find Smallest Value in vec
    auto smallest = std::min_element(vec.begin(), vec.end());
    return std::distance(vec.begin(), smallest);
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
lt::triangle<double> Math_tools::
triangle_side(lt::positions<T> pos)
{
    lt::position<double> dist, dist2, dist3;
    
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
		    std::pow((dist.y), 2));
    
    t.f1 = std::sqrt(std::pow((dist2.x), 2)+
		    std::pow((dist2.y), 2));
    
    t.f2 = std::sqrt(std::pow((dist3.x), 2)+
		    std::pow((dist3.y), 2));
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
Arg Math_tools::variance(std::vector<Arg> vec)
{
  /*  note that diff_f3_ was used here */
  size_t sz = vec.size();
  if (sz == 1)
    return 0.0;

  /*  Do not take the first value */
  Arg mean = std::accumulate(vec.begin() + 1, 
			     vec.end(), 0.0)/vec.size();
  
  return std::accumulate(vec.begin(), vec.end(), 0.0, 
			 [&mean, &sz](double accumulator, const double& val) {		      
			   return accumulator +
			     ((val - mean)*(val - mean) / (sz - 1));
			 } );  
}
