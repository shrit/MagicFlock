# include "math_tools.hh"

template <typename Arg, typename... Args>
Arg Math_tools::cantor_pairing(Arg&& arg, Args&&... args)
{  
  Arg unique = 0.5*(arg + std::forward<Args> args)*(arg + std::forward<Args> args + 1) +
    std::forward<Args> args;
  
  return (cantor_pairing(unique, ... ));     
}

double ::gaussian_noise(std::vector<lt::triangle<double>> distances)
{
  std::vector<double> ideal_f3;
  
  std::transform(distances.begin(), distances.end(), std::back_inserter(ideal_f3),
		 [](lt::triangle<double> const& t) { return t.f3; });
  
  std::adjacent_difference(ideal_f3.begin(), ideal_f3.end(), std::back_inserter(diff_f3_));

  /* The difference in distances needs to be in absolute value */
  /*  This has a tremendous cost since we need to re */
  double (*fabs)(double) = &std::fabs;
  std::transform(diff_f3_.begin(), diff_f3_.end(), diff_f3_.begin(), fabs);
  
  //  LogInfo() << "difference f3: " << diff_f3_;
  
  // adding one here to remove the first element of adjacent difference
  double noise_mean = std::accumulate(diff_f3_.begin() + 1, 
				   diff_f3_.end(), 0.0)/diff_f3_.size();
  return noise_mean;
}

bool ::
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

lt::triangle<double>::
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

double ::
variance(double mean)
{  
  size_t sz = diff_f3_.size();
  if (sz == 1)
    return 0.0;
  
  return std::accumulate(diff_f3_.begin(), diff_f3_.end(), 0.0, 
			 [&mean, &sz](double accumulator, const double& val) {		      
			   return accumulator +
			     ((val - mean)*(val - mean) / (sz - 1));
			 } );  
}


