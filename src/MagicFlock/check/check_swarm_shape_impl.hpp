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
  for (int j = 1; j < quads.size(); ++j) {
    // Starting checking only the follower, do not include the leader.
    for (std::size_t i = 1; i < quads.at(j).neighbors().size(); ++i) {
      double distance = quads.at(j).position().Distance(quads.at(j).neighbors().at(i).position);
      ILMR::logger::logger_->debug("Distance to neigh: {}", distance);
      if ((distance < lower_threshold_) or (distance > upper_threshold_)) {
        good_shape = false;
        goto final;
      }
    }
  }
final:
  return good_shape;
}
