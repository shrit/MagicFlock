#pragma once

template<class QuadrotorType>
CheckShape<QuadrotorType>::CheckShape(double lower_threshold,
                                      double upper_threshold)
  : lower_threshold_(lower_threshold)
  , upper_threshold_(upper_threshold)
{}

template<class QuadrotorType>
double&
CheckShape<QuadrotorType>::lower_threshold()
{
  return lower_threshold_;
}

template<class QuadrotorType>
double
CheckShape<QuadrotorType>::lower_threshold() const
{
  return lower_threshold_;
}

template<class QuadrotorType>
double&
CheckShape<QuadrotorType>::upper_threshold()
{
  return upper_threshold_;
}

template<class QuadrotorType>
double
CheckShape<QuadrotorType>::upper_threshold() const
{
  return upper_threshold_;
}

template<class QuadrotorType>
bool
CheckShape<QuadrotorType>::is_good_shape(std::vector<QuadrotorType> quads)
{
  bool good_shape = true;
  // std::vector<double> distances =
  //   vec_.map_to_vector(dist_.distances_to_neighbors(id, nearest_neighbors,
  //   positions));
  // logger::logger_->debug("Distances to other quadrotors: {} ", distances);
  // if (std::any_of(distances.begin(), distances.end(), [&](const double& i) {
  //       if ((i < lower_threshold_) or (i > upper_threshold_)) {
  //         return true;
  //       } else
  //         return false;
  //     })) {
  //   good_shape = false;
  // }
  return good_shape;
}
