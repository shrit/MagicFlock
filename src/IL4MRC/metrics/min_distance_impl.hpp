#pragma once

template<QuadrotorType>
MinDistance<QuadrotorType>::check_distance(
  const std::vector<QuadrotorType>& quads);
{
  double min_distance;
  for (auto&& q : quads) {
    for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
      min_distance = q.position().Distance(q.neighbor_positions().at(i));
      if (min_distance < min_distance_) {
        min_distance_ = min_distance;
      }
    }
  }
}
