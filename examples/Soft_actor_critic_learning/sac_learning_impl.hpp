#pragma once

template<class QuadrotorType>
SoftActorCritic<QuadrotorType>::SoftActorCritic(
  std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
  , sac_(quadrotors)
{
  // Nothing to do here
}

template<class QuadrotorType>
void
SoftActorCritic<QuadrotorType>::go_to_destination()
{
  std::vector<std::thread> threads;

  /*  Threading QuadCopter */
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
SoftActorCritic<QuadrotorType>::stop()
{
  std::vector<std::thread> threads;
  ignition::math::Vector3d stop{ 0, 0, 0 };
  /*  Threading Quadrotors */
  for (auto&& it : quadrotors_) {
    threads.push_back(std::thread([&]() {
      it.current_action().action() = stop;
      swarm_.one_quad_execute_trajectory(it.id(), it.current_action());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
SoftActorCritic<QuadrotorType>::run(std::function<void(void)> reset)
{
  //! Set up actor and critic network to start training
  sac_.SacNetwork();

  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    timer_.start();
    time_steps_.reset();
    swarm_.in_air_async(15);

    ignition::math::Vector3d destination{ 163, 0, 20 };
    ignition::math::Vector3d max_speed{ 2, 2, 0.2 };
    ignition::math::Vector4d gains{ 1, 7, 1, 100 };

    // Examin if the takeoff has been well worked
    bool takeoff_shape = swarm_.examin_swarm_shape(0.5, 10);
    if (takeoff_shape) {
      logger_->info("Quadrotors are far from each other, ending the episode");

      for (auto&& it : quadrotors_) {
        logger_->info("Start the flocking model");
        it.start_flocking_model(gains, destination, max_speed);
      }
      for (auto&& it : quadrotors_) {
        it.start_sampling_rt_state(50);
      }

      std::size_t consecutiveEpisode = 100;
      std::size_t Steps = 10000;

      std::function<void(void)> trajectory = [this]() {
        this->go_to_destination();
      };

      std::function<void(void)> stop_trajectory = [this]() { this->stop(); };

      std::function<double()> reward = [this]() {
        return reward_.calculate_reward(quadrotors_.at(0));
      };

      std::function<bool()> shape = [&]() {
        bool isTerminal = false;
        bool temp = swarm_.examin_swarm_shape(0.5, 30);
        bool temp2 = swarm_.examin_destination(destination);
        if (temp) {
          isTerminal = temp;
          logger_->info(
            "Quadrotors are far from each other, ending the episode");
        } else if (temp2) {
          logger_->info("Quadrotors have arrived at specificed destination. "
                        "ending the episode.");
          isTerminal = temp2;
        }
        return isTerminal;
      };

      sac_.train(
        consecutiveEpisode, Steps, trajectory, stop_trajectory, reward, shape);

      // quadrotors_.at(0).save_values("distance_metric", maxD, minD);

      for (auto&& it : quadrotors_) {
        it.stop_flocking_model();
      }

      for (auto&& it : quadrotors_) {
        it.stop_sampling_rt_state();
      }
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
