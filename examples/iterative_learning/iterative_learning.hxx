#pragma once

template<class QuadrotorType>
Iterative_learning<QuadrotorType>::Iterative_learning(
  std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
{
  // Nothing to do here
}

template<class QuadrotorType>
void
Iterative_learning<QuadrotorType>::generate_trajectory_using_model()
{
  std::vector<std::thread> threads;
  double max_speed = 2;

  for (auto&& i : quadrotors_) {
    AnnStatePredictor<QuadrotorType> predict(
      "/meta/lemon/examples/iterative_learning/build/model.txt", "model", i);
    ContinuousActions action = predict.best_predicted_action();
    i.current_action() = action;
  }

  /*  Threading QuadCopter */
  for (auto&& it : quadrotors_) {
    threads.push_back(std::thread([&]() {
      swarm_.one_quad_execute_trajectory(
        it.id(), it.current_action(), max_speed);
    }));
  }

  /* We need to wait until the quadcopters finish their actions */
  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
Iterative_learning<QuadrotorType>::run(std::function<void(void)> reset)
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    timer_.start();
    start_episode_ = swarm_.in_air(15);

    while (true) {
      generate_trajectory_using_model();

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      /*  Check the geometrical shape */
      bool shape = swarm_.examin_swarm_shape();

      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
    }

    for (auto&& it : quadrotors_) {
      it.stop_sampling_rt_state();
    }

    swarm_.land();

    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time: {}", flight_time);

    /* Resetting the entire swarm after the end of each episode*/
    reset();

    logger_->info("The quadcopters have been reset...");
    std::this_thread::sleep_for(std::chrono::seconds(35));
    logger_->flush();
  }
}
