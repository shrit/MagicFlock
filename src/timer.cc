#include "include/timer.hh"

Timer::Timer() {}

void
Timer::start()
{
  starting_time_ = std::chrono::steady_clock::now();
}

int
Timer::stop()
{
  std::chrono::steady_clock::time_point end_time =
    std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                              starting_time_);

  return elapsed_time.count();
}

std::string
Timer::stop_and_get_time()
{
  std::chrono::steady_clock::time_point end_time =
    std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                              starting_time_);
  int total = elapsed_time.count();
  int minutes = total / 60;
  int seconds = total % 60;
  int hours = minutes / 60;
  minutes = minutes % minutes;

  std::string time = std::to_string(hours) + ":" + std::to_string(minutes) +
                     ":" + std::to_string(seconds);

  return time;
}
