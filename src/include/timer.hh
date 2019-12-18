#pragma once

# include <chrono>

class Timer {

public:

  Timer();
  void start();
  double stop();

private:

  std::chrono::steady_clock::time_point starting_time_;

};
