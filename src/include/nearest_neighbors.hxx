#pragma once

template<class NeighborType>
NearestNeighbors<NeighborType>::NearestNeighbors(
  std::vector<NeighborType> neighbors)
  : neighbors_(neighbors)
{}

template<class NeighborType>
std::vector<unsigned int>
NearestNeighbors<NeighborType>::search()
{
  std::vector<unsigned int> ids;
  for (auto it : neighbors_) {
    std::string name = it->name();
    unsigned int id = static_cast<unsigned int>(std::atoi(name.back()));
    ids.push_back(id);
  }
  return ids;
}
