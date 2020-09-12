#pragma once

#include "time.hpp"

class TimeSteps
{

public:
  TimeSteps();

  void tic();
  int steps() const;

  void reset();
  double elapsed_time_between_last_2_steps() const;

private:
  int counter_ = 0;
  double elapsed_time_ = 0;
};
