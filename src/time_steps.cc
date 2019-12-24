# include "include/time_steps.hh"


TimeSteps::TimeSteps()
{}

void TimeSteps::tic()
{
  Timer timer;
  elapsed_time_ = timer.stop();
  counter_++;
  timer.start();
}

int TimeSteps::steps() const
{ return counter_; }

void TimeSteps::reset()
{ counter_ = 0; }

double TimeSteps::elapsed_time_between_last_2_steps() const
{ return elapsed_time_; }
