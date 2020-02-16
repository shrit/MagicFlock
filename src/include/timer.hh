#pragma once

#include <chrono>
#include <string>

class Timer
{

public:
  Timer();
  void start();
  int stop();
  std::string stop_and_get_time();

private:
  std::chrono::steady_clock::time_point starting_time_;
};
