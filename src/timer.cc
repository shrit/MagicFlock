# include "include/timer.hh"

Timer::Timer()
{}

void Timer::start()
{
  starting_time_ = std::chrono::steady_clock::now();
}

double Timer::stop()
{
  std::chrono::steady_clock::time_point end_time = std::chrono::steady_clock::now();        
  std::chrono::duration<double> elapsed_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time - starting_time_);

  return elapsed_time.count();  
}
