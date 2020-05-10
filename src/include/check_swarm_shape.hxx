#pragma once

template<class QuadrotorType>
bool
CheckShape<QuadrotorType>::is_good_shape(std::vector<QuadrotorType> quads)
{
  bool good_shape = true;
  // std::vector<double> distances =
  //   vec_.map_to_vector(dist_.distances_to_neighbors(id, nearest_neighbors, positions));
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
