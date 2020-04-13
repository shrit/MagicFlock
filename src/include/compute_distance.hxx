#pragma once

template<typename T>
std::map<unsigned int, double>
ComputeDistance::distances_to_neighbors(unsigned int id,
                                   std::vector<unsigned int> nearest_neighbors,
                                   std::vector<ignition::math::Vector3d> positions)
{
  std::map<unsigned int, double> distances;
  for (auto&& i : nearest_neighbors) {
    distances.insert({ i, distance_a_2_b(positions, id, i) });
  }
  return distances;
}

template<typename T>
double
ComputeDistance::distance_a_2_b(std::vector<ignition::math::Vector3d > positions,
                           unsigned int id_a,
                           unsigned int id_b)
{
  ignition::math::Vector3d dist;
  /*  Distance between a and b */
  dist.x() = positions.at(id_a).x() - positions.at(id_b).x();
  dist.y() = positions.at(id_a).y() - positions.at(id_b).y();
  dist.z() = positions.at(id_a).z() - positions.at(id_b).z();

  double distance = std::sqrt(std::pow((dist.x()), 2) + std::pow((dist.y()), 2) +
                              std::pow((dist.z()), 2));
  return distance;
}

template<typename Arg>
bool
ComputeDistance::hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2)
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
ComputeDistance::ecludian_distance(T d1, T d2)
{
  double distance = std::sqrt(std::pow((d1), 2) - std::pow((d2), 2));
  return distance;
}

template<typename T>
double
ComputeDistance::traveled_distances(ignition::math::Vector3d  pos_t,
                               ignition::math::Vector3d  pos_t_1)
{
  ignition::math::Vector3d dist;

  /* Travelled distance between time steps */
  dist.x() = pos_t.x() - pos_t_1.x();
  dist.y() = pos_t.y() - pos_t_1.y();
  dist.z() = pos_t.z() - pos_t_1.z();

  double traveled_distance;

  /*  Summing up vectors, the total is the distance travelled by each
      agent during each trajectory */

  /*  Distance travelled by the leader */
  traveled_distance = std::sqrt(std::pow((dist.x()), 2) + std::pow((dist.y()), 2) +
                                std::pow((dist.z()), 2));
  return traveled_distance;
}
