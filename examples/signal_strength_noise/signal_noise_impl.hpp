template<class QuadrotorType>
SignalNoise<QuadrotorType>::SignalNoise(std::vector<QuadrotorType>& quadrotors,
                                        std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , stddev_all_quad_(14)
  , var_all_quad_(14)
  , logger_(logger)
{}

template<class QuadrotorType>
void
SignalNoise<QuadrotorType>::calculate_RSSI_mean_variance()
{
  arma::mat dataMat;
  for (auto&& it : quadrotors_) {
    for (size_t i = 0; i < it.all_states().size(); ++i) {
      arma::colvec dataCol = it.all_states().at(i).Data();
      dataMat.insert_cols(dataMat.n_cols, dataCol);
    }
    arma::colvec mean_vec = arma::mean(dataMat, 1);
    arma::colvec range_vec = arma::range(dataMat, 1);
    mean_all_quad_.push_back(mean_vec);
    range_all_quad_.push_back(range_vec);

    for (arma::uword i = 0; i < dataMat.n_rows; ++i) {
      double stddev = arma::stddev(dataMat.row(i), 0);
      double var = arma::var(dataMat.row(i), 0);

      stddev_all_quad_.at(i).push_back(stddev);
      var_all_quad_.at(i).push_back(var);
    }
  }
}

template<class QuadrotorType>
void
SignalNoise<QuadrotorType>::run(std::function<void(void)> reset)
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
    logger_->info("Episode : {}", episode_);

    for (auto&& it : quadrotors_) {
      it.start_sampling_rt_state(50);
    }

    std::this_thread::sleep_for(std::chrono::seconds(120));

    for (auto&& it : quadrotors_) {
      it.stop_sampling_rt_state();
    }

    calculate_RSSI_mean_variance();
    for (size_t i = 0; i < mean_all_quad_.size(); ++i) {
      logger_->info("Mean values for signal strength for quad {}\n {}",
                    i,
                    mean_all_quad_.at(i));
      for (size_t j = 0; j < mean_all_quad_.size(); ++j) {
        logger_->info(
          "Standard deviation values for signal strength for quad {}\n {}\n",
          i,
          stddev_all_quad_.at(i).at(j));
        logger_->info("Variance values for signal strength for quad {}\n {}\n",
                      i,
                      var_all_quad_.at(i).at(j));
      }
    }
  }
}
