template<class QuadrotorType>
SignalNoise<QuadrotorType>::SignalNoise(std::vector<QuadrotorType>& quadrotors,
                                    std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , passed_time_(0.0)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
{}

template<class QuadrotorType>
void
SignalNoise<QuadrotorType>::run(std::function<void(void)> reset)
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
   logger_->info("Episode : {}", episode_);
    timer_.start();
    time_steps_.reset();

    Timer model_time;
    model_time.start();

      while (true) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        double passed_time = model_time.stop();
        logger_->info("Model time in seconds {}", passed_time);
        if (passed_time > passed_time_ + 120) {
          logger_->info("Seconds have passed change the model");
          passed_time_ = passed_time;
          break;
        }
        go_to_destination(2); // refactor this max speed in the flocking model
      }

  }
}
