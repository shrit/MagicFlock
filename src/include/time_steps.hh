#pragma once

# include "timer.hh"

class TimeSteps {

public:
  
  TimeSteps();

  void tic();
  int steps() const;

  void reset_time_steps();  
  double elapsed_time_between_last_2_steps() const;
  
private:  
  int counter_ = 0;
  double elapsed_time_ = 0;
};
