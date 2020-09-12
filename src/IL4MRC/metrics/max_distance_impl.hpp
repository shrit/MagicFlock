#pragma once

template<QuadrotorType>
MaxDistance<QuadrotorType>::check_distance(
  const std::vector<QuadrotorType>& quads);
{
  double max_distance;
  for (auto&& q : quads) {
    for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
      max_distance = q.position().Distance(q.neighbor_positions().at(i));
      if (max_distance < max_distance_) {
        max_distance_ = max_distance;
      }
    }
  }
}
