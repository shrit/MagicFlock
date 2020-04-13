#pragma once

#include <map>
#include <vector>

#include <ignition/math6/ignition/math/Vector3.hh>

#include "global.hh"

namespace lt = local_types;

class ComputeDistance
{
public:
  ComputeDistance() {}

  std::map<unsigned int, double> distances_to_neighbors(
    unsigned int id,
    std::vector<unsigned int> nearest_neighbors,
    std::vector<ignition::math::Vector3d> positions);

  double distance_a_2_b(std::vector<ignition::math::Vector3d> positions,
                        unsigned int id_a,
                        unsigned int id_b);

  template<typename Arg>
  bool hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2);

  template<typename T>
  int ecludian_distance(T d1, T d2);

  double traveled_distances(ignition::math::Vector3d pos_t,
                            ignition::math::Vector3d pos_t_1);
};

#include "compute_distance.hxx"
