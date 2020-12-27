#include "logger.hpp"
#include "time.hpp"

class Scheduler
{

public:
  Scheduler()
  {
    // Nothing to do here
  }

  void start() { timer_.start(); }

  void each_interval(int sec,
                     bool& value,
                     std::function<void(void)> execute_function)
  {
    elapsed_time_ = timer_.stop();
    logger::logger_->info("Model time in seconds {}", elapsed_time_);

    if (elapsed_time_ > passed_time_ + sec) {
      logger::logger_->info("Interval has passed change the value", elapsed_time_);
      passed_time_ = elapsed_time_;
      if (value == true)
        value = false;
      else {
        value = true;
      }
      execute_function();
    }
  }

  void change_after(int sec, bool& value) {}

  void reset()
  {
    elapsed_time_ = 0;
    passed_time_ = 0;
  }

private:
  Timer timer_;
  double elapsed_time_, passed_time_;
  int counter_;
};
