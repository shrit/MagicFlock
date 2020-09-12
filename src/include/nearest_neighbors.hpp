#pragma once

#include <map>
#include <vector>

#include "real_time_samples.hh"

/* This class parse the number of NearestNeighbor 
 *  by extracting their id from the a vector*/

template<class NeighborType>
class NearestNeighbors
{
public:
  NearestNeighbors(std::vector<NeighborType> neighbors);
  std::vector<unsigned int> search();

private:
  std::vector<NeighborType> neighbors_;
};

#include "nearest_neighbors_impl.hpp"
