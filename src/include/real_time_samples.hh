#pragma once

#include <chrono>
#include <functional>
#include <future>

class RTSamples
{
public:
  RTSamples();
  ~RTSamples();

  void stop();
  void start(int interval, std::function<void(void)> func);
  bool is_running() const noexcept;

private:
  std::atomic<bool> _execute;
  std::thread _thd;
};
