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
  , sac_(quadrotors.at(0))
{
  // Nothing to do here
}

template<class QuadrotorType>
void
SoftActorCritic<QuadrotorType>::generate_trajectory_using_model()
{
  std::vector<std::thread> threads;

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

    for (auto&& it : quadrotors_) {
      it.start_sampling_rt_state(50);
    }

    for (auto&& it : quadrotors_) {
      logger_->info("Start the flocking model");
      it.start_flocking_model(gains, destination, max_speed);
    }

    sac_.train(generate_trajectory_using_model,
          swarm_.examin_swarm_shape,
          reward_.calculate_reward // We need to pass destination
    );

    /*  Check the destination  */
    bool has_arrived = swarm_.examin_destination(destination);

    if (has_arrived) {
      logger_->info("Quadrotors have arrived at specificed destination. "
                    "ending the episode.");
      break;
    }

    // quadrotors_.at(0).save_values("distance_metric", maxD, minD);

    for (auto&& it : quadrotors_) {
      it.stop_flocking_model();
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
