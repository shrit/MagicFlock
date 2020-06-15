#pragma once

template<typename type>
ExpoMovingAverage<type>::ExpoMovingAverage()
{
  // Nothing to do here
}

template<typename type>
ExpoMovingAverage<type>::ExpoMovingAverage(double alpha)
  : alpha_(alpha)
{
  if (alpha_ > 1 and alpha_ < 0) {
    logger::logger_->error("Alpha has not been intilized correctly, it"
                           "should be between 0 and 1",
                           alpha_);
    exit(0);
  }
}

template<typename type>
ExpoMovingAverage<type>::ExpoMovingAverage(double alpha,
                                           const type& initial_value)
  : alpha_(alpha)
  , value_(initial_value)
{
  if (alpha_ > 1 and alpha_ < 0) {
    logger::logger_->error("Alpha has not been intilized correctly, it"
                           "should be between 0 and 1",
                           alpha_);
    exit(0);
  }
}

template<typename type>
type
ExpoMovingAverage<type>::input(const type& new_value)
{
  return value_ = alpha_ * new_value + (1 - alpha_) * value_;
}

template<typename type>
const double&
ExpoMovingAverage<type>::alpha() const
{
  return alpha_;
}

template<typename type>
const type&
ExpoMovingAverage<type>::output() const
{
  return value_;
}

template<typename type>
type&
ExpoMovingAverage<type>::initial_value()
{
  return initial_value_;
}

template<typename type>
type
ExpoMovingAverage<type>::initial_value() const
{
  return initial_value_;
}

template<typename type>
void
ExpoMovingAverage<type>::reset()
{
  value_ = initial_value_;
}
