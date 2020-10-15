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
CheckShape<QuadrotorType>::is_good_shape(
  const std::vector<QuadrotorType>& quads)
{
  bool good_shape = true;
  for (auto&& q : quads) {

    for (std::size_t i = 0; i < q.neighbor_positions().size(); ++i) {
      double distance = q.position().Distance(q.neighbor_positions().at(i));
      ILMR::logger::logger_->debug("Distance to neigh: {}", distance);
      if ((distance < lower_threshold_) or (distance > upper_threshold_)) {
        good_shape = false;
        break;
      }
    }
  }
  return good_shape;
}
