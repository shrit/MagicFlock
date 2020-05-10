#include "include/check_swarm_shape.hh"

bool
CheckShape::is_good_shape(unsigned int id,
                          std::vector<unsigned int> nearest_neighbors,
                          std::vector<ignition::math::Vector3d> positions)
{
  bool good_shape = true;
  std::vector<double> distances =
    vec_.map_to_vector(dist_.distances_to_neighbors(id, nearest_neighbors, positions));
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
