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

  for (auto&& i : quadrotors_) {
    AnnActionPredictor<QuadrotorType> predict(
      "/meta/lemon/examples/iterative_learning/build/model.bin", "model", i);
    ContinuousActions action = predict.best_predicted_action();
    i.current_action() = action;
  }

  /*  Threading QuadCopter */
  for (auto&& it : quadrotors_) {
    threads.push_back(std::thread([&]() {
      swarm_.one_quad_execute_trajectory(
        it.id(), it.current_action());
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
    time_steps_.reset();
    swarm_.in_air_async(15);

    ignition::math::Vector3d destination{ 163, 0, 20 };

    for (auto&& it : quadrotors_) {
      it.start_sampling_rt_state(50);
    }

    while (true) {
      generate_trajectory_using_model();

      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      /*  Check the geometrical shape */
      bool shape = swarm_.examin_swarm_shape();
      bool has_arrived = swarm_.examin_destination(destination);

      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }

      if (has_arrived) {
        logger_->info("Quadrotors have arrived at specificed destination. "
                      "ending the episode.");
        break;
      }

      /* Register results */
      /* Save position of quadrotor each 200 ms*/
      for (auto&& it : quadrotors_) {
        it.save_position(std::to_string(episode_));
      }
      double maxD = max_distance_.check_global_distance(quadrotors_);
      double minD = min_distance_.check_global_distance(quadrotors_);

      quadrotors_.at(0).save_values("distance_metric", maxD, minD);
    }
      
    for (auto&& it : quadrotors_) {
      it.stop_sampling_rt_state();
    }

    /* Landing is blocking untill all quadrotors in the swarm touch the
     * ground */
    swarm_.stop_offboard_mode_async();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    swarm_.land();
    std::this_thread::sleep_for(std::chrono::seconds(3));

    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time: {}", flight_time);

    /* Resetting the entire swarm after the end of each episode*/
    reset();

    logger_->info("The quadrotors have been reset...");
    std::this_thread::sleep_for(std::chrono::seconds(35));
    logger_->flush();
  }
}
