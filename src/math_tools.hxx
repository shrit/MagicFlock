# include "math_tools.hh"


Math_tools::Math_tools()
  :   lower_threshold_{4, 4, 4},
      upper_threshold_{9, 9, 9}
{}

double Math_tools::gaussian_noise(std::vector<lt::triangle<double>> distances,
				  std::vector<double> drift_f3)
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

bool Math_tools::
is_triangle(lt::triangle<double> t)
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

lt::triangle<double> Math_tools::
triangle_side(lt::positions<double> pos)
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

template <typename Arg, typename Arg2>
std::vector<bool> Math_tools::to_one_hot_encoding(Arg arg, Arg2 number_of_class)
{
  std::vector<bool> one_hot;
  // to check 
  //  if constexpr (std::is_same<Arg, enum>::value) {
  
  if (static_cast<int>(arg) == 0 )
    one_hot.push_back(1);
  for (int i =0 ; i< number_of_class-1 ; ++i)
      one_hot.push_back(0);
    
    /*  To be continued in a recursive way */
    // }  
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


