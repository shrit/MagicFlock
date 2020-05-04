#pragma once

template<class QuadrotorType>
Generator<QuadrotorType>::Generator(
  std::vector<std::shared_ptr<QuadrotorType>> quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
{}

/*  Phase one: Data Set generation */
template<class QuadrotorType>
void
Generator<QuadrotorType>::generate_trajectory()
{
  //  std::vector<std::thread> threads;

  /*  Do not allow follower to move, at this time step,
        block the follower and log not move*/

  /*  Threading Quadrotors */
  // threads.push_back(std::thread([&]() {

  // }));

  // /*  Threading Quadrotors */
  // threads.push_back(std::thread([&]() {

  // }));

  // /* Get the next state at time t + 1  */
  // logger_->info("Sampling states at t +1");

  // threads.push_back(std::thread([&]() {}));

  // threads.push_back(std::thread([&]() {}));

  // for (auto& thread : threads) {
  //   thread.join();
  // }
}

template<class QuadrotorType>
void
Generator<QuadrotorType>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    timer_.start();
    start_episode_ = swarm_.in_air(25);
    /*  Collect dataset by creating a set of trajectories, each time
                                the leader_and the follower execute their
       trajectory randomly, we check the geometrical figure (whether the
       follower is too close or too far) finally we break the loop after 10
       trajectorise of 1 second each */

    if (start_episode_) {
      /*  Verify that vectors are clear when starting new episode */
      logger_->info("Taking off has finished. Start generating trajectories");
      time_steps_.reset();
      //    while (start_episode_) {
      //    generate_trajectory();

      // leader_->reset_all_actions();
      // follower_1_->register_data_set();
      // follower_2_->register_data_set();

      // follower_1_->register_episodes(episode_);

      /*  Check the geometrical shape */
      //     std::vector<bool> shapes;
      //     for (auto it : quadrotors_) {
      //       shapes.push_back(it.examin_geometric_shape());
      //     }
      //     if (std::any_of(shapes.begin(), shapes.end(), [](const bool&
      //     shape) {
      //           if (!shape)
      //             return true;
      //           else
      //             return false;
      //         })) {
      //       logger_->info("The geometrical figure is no longer conserved");
      //       break;
      //     }
      //     time_steps_.tic();
      //     logger_->flush();
      //   }
      // }
      // /*  Save a version of the time steps to create a histogram */
      // follower_1_->register_histogram(time_steps_.steps());
      // follower_2_->register_histogram(time_steps_.steps());

      /* Landing is blocking untill touching the ground*/
      swarm_.land();
      std::string flight_time = timer_.stop_and_get_time();
      logger_->info("Flight time: {}", flight_time);
    }

    /* Resetting the entire swarm after the end of each episode*/
    // leader_->reset_models();

    logger_->info("All quadrotors have been reset...");

    std::this_thread::sleep_for(std::chrono::seconds(35));

    /*BIAS accelerometer problem after resetting the models*/

    /*  The only possible solution was to change the upper limit value
     * for the bias inside thee code of the firmware directly. The
     * solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they propose
     * to increase the value of COM_ARM_EKF_AB. Note that, the default
     * value is 0.00024 I have increased it to 0.00054 which is very
     * high to the usual stadard. Otherwise there is no way to do the
     * simulation. Remember, the reboot() function in the MAVSDK
     * action class is not implemented at the time of writing this
     * comment, and maybe it will never be implemented as it is quite
     * complicated to reboot the px4 software from the simulator. I
     * understand this choice, we need to leave a big sleep_for after
     * resetting the quadcopters, that is going to helpe resetting the
     * accelerometer values without any problems!
     */

    /*  I have quite tested a lot of different solution. Frankly, if I am
     * going to find a better one, I will replace it directly. */
  }
}
