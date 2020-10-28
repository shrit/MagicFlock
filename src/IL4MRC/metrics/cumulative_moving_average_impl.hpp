#pragma once

template<typename type>
CumulativeMovingAverage<type>::CumulativeMovingAverage()
{
  // Nothing to do here
}

template<typename type>
CumulativeMovingAverage<type>::CumulativeMovingAverage(
  const type& initial_value)
  : value_(initial_value)
{
  // Nothing to do here
}

template<typename type>
type
CumulativeMovingAverage<type>::input(const type& new_value)
{
  count_ = count_ + 1;
  return value_ = value_ + ((new_value - value_) / (count_ + 1));
}

template<typename type>
const type&
CumulativeMovingAverage<type>::output() const
{
  return value_;
}

template<typename type>
type&
CumulativeMovingAverage<type>::initial_value()
{
  return initial_value_;
}

template<typename type>
type
CumulativeMovingAverage<type>::initial_value() const
{
  return initial_value_;
}

template<typename type>
void
CumulativeMovingAverage<type>::reset()
{
  count_ = 0;
  value_ = initial_value_;
}
