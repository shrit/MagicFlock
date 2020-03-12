#pragma once

#include <map>
#include <vector>

#include "global.hh"

namespace lt = local_types;

class ComputeDistance
{
public:
  ComputeDistance() {}

  template<typename T>
  std::map<unsigned int, double> distances_to_neighbors(
    unsigned int id,
    std::vector<unsigned int> nearest_neighbors,
    std::vector<lt::position3D<T>> positions);

  template<typename T>
  double distance_a_2_b(std::vector<lt::position3D<T>> positions,
                        unsigned int id_a,
                        unsigned int id_b);

  template<typename Arg>
  bool hamming_distance_one_hot(std::vector<Arg> v1, std::vector<Arg> v2);

  template<typename T>
  int ecludian_distance(T d1, T d2);

  template<typename T>
  double traveled_distances(lt::position3D<T> pos_t, lt::position3D<T> pos_t_1);
};

#include "compute_distance.hxx"
