#pragma once

template<class QuadrotorType>
Iterative_learning<QuadrotorType>::Iterative_learning(
  std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , passed_time_(0.0)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
  , distribution_int_(0, 3)
  , generator_(random_dev())
{
  // Nothing to do here
}

template<class QuadrotorType>
void
Iterative_learning<QuadrotorType>::generate_trajectory_using_model()
{
  std::vector<std::thread> threads;

  quadrotors_.at(0).current_action().action() = dest_;
  /*  Threading Quadrotor */
  for (auto&& it : quadrotors_) {
    threads.push_back(std::thread([&]() {
      swarm_.one_quad_execute_trajectory(it.id(), it.current_action());
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

    /* Resetting the entire swarm after the end of each episode*/
    reset();
    logger_->info("The quadrotors have been reset...");
    std::this_thread::sleep_for(std::chrono::seconds(35));
    logger_->info("Episode : {}", episode_);
    timer_.start();
    time_steps_.reset();
    swarm_.in_air_async(40);
    int random = 0;
    int count = 0;
    bool leader = true;
    std::vector<ignition::math::Vector3d> destinations{
      { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }
    };

    ignition::math::Vector3d up{ 0, 0, +1.5 };
    quadrotors_.at(0).current_action().action() = up;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    random = distribution_int_(generator_);
    quadrotors_.at(0).current_action().action() = dest_;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(5));
    Timer model_time;
    model_time.start();

    logger_->info("Taking off has finished. Start the flocking model");
    ignition::math::Vector4d gains{ 1, 7, 1, 100 };
    ignition::math::Vector3d max_speed{ 1, 1, 0.3 };
    ignition::math::Vector4d axis_speed{ 0.1, 0.1, 0.09, 4 };
    std::function<void(void)> action_model = [&]() {
      if (count % 2 == 0) {
        for (std::size_t i = 1; i < quadrotors_.size(); ++i) {
          logger_->info("Start the flocking model");
          quadrotors_.at(i).flocking_model(
            gains, quadrotors_.at(0).position(), max_speed, leader);
        }
      } else {
        for (auto&& i : quadrotors_) {
          AnnActionPredictor<QuadrotorType> predict(
            "/meta/lemon/examples/iterative_learning/build/model.bin",
            "model",
            i);
          ContinuousActions action = predict.best_predicted_action();
          i.current_action() = action;
          i.current_action().action().Z() = 0;
        }
      }
    };

    std::function<void(void)> trajectory = [&]() {
      this->generate_trajectory_using_model();
    };

    while (true) {
      bool shape = swarm_.examin_swarm_shape(0.1, 35);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }

      elapsed_time_ = model_time.stop();
      logger_->info("Model time in seconds {}", elapsed_time_);

      if (elapsed_time_ > passed_time_ + 2) {
        logger_->info("Seconds have passed change the model");
        random = distribution_int_(generator_);
        count++;
      }

      if (elapsed_time_ > passed_time_ + 60) {
        passed_time_ = elapsed_time_;
        break;
      }

      for (auto&& it : quadrotors_) {
        it.sample_state_action_state(action_model, trajectory);
      }

      if (count % 2 == 0) {
        logger_->info("Change leader destination NOW");
        dest_ = destinations.at(random);
      }

      // if (count % 2 == 0) {
      for (auto&& it : quadrotors_) {
        // Let us see if these are still necessary
        logger_->info("Registering States, last state, Current state {} {}",
                      it.last_state().Data(),
                      it.current_state().Data());
        arma::colvec check_double =
          it.current_state().Data() - it.last_state().Data();
        if (!check_double.is_zero()) {
          it.save_dataset_sasas();
          logger_->info("Current State {}", it.current_state().Data());
        }
      }
      // }

      /*  Check the geometrical shape */
      shape = swarm_.examin_swarm_shape(0.2, 35);
      // bool has_arrived = swarm_.examin_destination(destination);

      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }

      // if (has_arrived) {
      //   logger_->info("Quadrotors have arrived at specificed destination. "
      //                 "ending the episode.");
      //   break;
      // }

      /* Register results */
      /* Save position of quadrotor each 200 ms*/
      // for (auto&& it : quadrotors_) {
      //   it.save_position(std::to_string(episode_));
      // }
      // double maxD = max_distance_.check_global_distance(quadrotors_);
      // double minD = min_distance_.check_global_distance(quadrotors_);

      // quadrotors_.at(0).save_values("distance_metric", maxD, minD);
    }

    for (auto&& it : quadrotors_) {
      it.stop_sampling_rt_state();
    }

    /* Landing is blocking untill all quadrotors in the swarm touch the
     * ground */
    swarm_.stop_offboard_mode_async();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    swarm_.land();
    swarm_.disarm_async();
    std::this_thread::sleep_for(std::chrono::seconds(3));
    passed_time_ = 0;
    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time: {}", flight_time);
  }
}
