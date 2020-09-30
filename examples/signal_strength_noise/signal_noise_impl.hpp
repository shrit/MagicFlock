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
SignalNoise<QuadrotorType>::calculate_RSSI_mean_variance()
{
  for (auto&& it : quadrotors_) {
     arma::mat dataMat = it.all_states().Data();
     arma::colvec mean_vec = arma::mean(dataMat, 0);
     mean_all_quad_.push_back(mean_vec);
  }
}

template<class QuadrotorType>
void
SignalNoise<QuadrotorType>::run(std::function<void(void)> reset)
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
   logger_->info("Episode : {}", episode_);
    timer_.start();
    time_steps_.reset();

    for (auto&& it : quadrotors_) {
      it.start_sampling_rt_state(50);
    }

    std::this_thread::sleep_for(std::chrono::seconds(120));

    for (auto&& it : quadrotors_) {
      it.stop_sampling_rt_state();
    }

    calculate_RSSI_mean_variance();
    for (size_t i = 0; i < mean_all_quad_.size(); ++i) {
      logger_->info("Mean value for signal strength for quad {}", mean_all_quad_.at(i));
    }
  }
}
