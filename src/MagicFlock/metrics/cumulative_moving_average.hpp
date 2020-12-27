#pragma once

#include <IL4MRC/util/logger.hpp>

template<typename type>
class CumulativeMovingAverage
{
public:
  CumulativeMovingAverage();
  CumulativeMovingAverage(const type& initial_value);

  type input(const type& new_value);
  const type& output() const;

  type initial_value() const;
  type& initial_value();

  void reset();

private:
  int count_ = 0;
  type initial_value_; //! Initial value.
  type value_;               //! Stored value.
};

#include "cumulative_moving_average_impl.hpp"
