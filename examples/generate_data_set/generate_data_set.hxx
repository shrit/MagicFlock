#pragma once

template<class flight_controller_t, class QuadrotorType>
Generator<flight_controller_t, QuadrotorType>::Generator(
  std::vector<std::shared_ptr<flight_controller_t>> quads,
  const std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , swarm_(std::move(quads))
  , quadrotors_(std::move(quadrotors))
  , logger_(logger)
{
  /*  Allow easier access and debugging to all quadrotors state */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
  leader_2_ = std::next(quadrotors_.begin(), 3);
}

/*  Phase one: Data Set generation */
template<class flight_controller_t, class QuadrotorType>
void
Generator<flight_controller_t, QuadrotorType>::generate_trajectory()
{
  ActionGenerator<QuadrotorType> leader_generator(leader_);
  ActionGenerator<QuadrotorType> follower_1_generator(follower_1_);
  ActionGenerator<QuadrotorType> follower_2_generator(follower_2_);

  std::vector<std::thread> threads;

  /*  Sample the state at time t */
  follower_1_->sample_state();
  follower_2_->sample_state();

  /*  Generate action for the leader */
  leader_->current_action(
    leader_generator.generate_persistant_action(10, time_steps_.steps()));
  leader_2_->current_action(leader_->current_action());

  /*  Do not allow follower to move, at this time step,
        block the follower and log not move*/
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  logger_->info("Charlie distances to others before leader actions, {}",
                        follower_1_->distances_to_neighbors());

  logger_->info("Bob distances to others before leader actions, {}",
                        follower_2_->distances_to_neighbors());

  logger_->info(
    "Current action leader: {}",
    leader_generator.action_to_str(leader_->current_action()));
  /*  Threading Quadrotors */
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));

  /*  Threading Quadrotors */
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_2_->id(), leader_2_->current_action(), leader_2_->speed(), 1000);
  }));

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  logger_->info("Charlie distances to others after leader actions, {}",
                        follower_1_->distances_to_neighbors());

  logger_->info("Bob distances to others after leader actions, {}",
                        follower_2_->distances_to_neighbors());

  /* Get the next state at time t + 1  */
  logger_->info("Sampling states at t +1");

  follower_1_->sample_state();
  follower_2_->sample_state();

  follower_1_->current_action(follower_1_generator.generate_random_action());
  follower_2_->current_action(follower_2_generator.generate_random_action());

  threads.push_back(std::thread([&]() {
    logger_->info(
      "Current action follower 1: {}",
      follower_1_generator.action_to_str(follower_1_->current_action()));
    swarm_.one_quad_execute_trajectory(follower_1_->id(),
                                       follower_1_->current_action(),
                                       follower_1_->speed(),
                                       1000);
  }));

  threads.push_back(std::thread([&]() {
    logger_->info(
      "Current action follower 2: {}",
      follower_2_generator.action_to_str(follower_2_->current_action()));
    swarm_.one_quad_execute_trajectory(follower_2_->id(),
                                       follower_2_->current_action(),
                                       follower_2_->speed(),
                                       1000);
  }));

  for (auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /* Get the next state at time t + 2  */
  logger_->info("Sampling states at t+2");
  follower_1_->sample_state();
  follower_2_->sample_state();

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

template<class flight_controller_t, class QuadrotorType>
void
Generator<flight_controller_t, QuadrotorType>::run()
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
      while (start_episode_) {
        generate_trajectory();

        leader_->reset_all_actions();
        follower_1_->register_data_set();
        follower_2_->register_data_set();

        follower_1_->register_episodes(episode_);

        /*  Check the geometrical shape */
        std::vector<bool> shapes;
        for (auto it : quadrotors_) {
          shapes.push_back(it.examin_geometric_shape());
        }
        if (std::any_of(shapes.begin(), shapes.end(), [](const bool& shape) {
              if (!shape)
                return true;
              else
                return false;
            })) {
          logger_->info("The geometrical figure is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger_->flush();
      }
    }
    /*  Save a version of the time steps to create a histogram */
    follower_1_->register_histogram(time_steps_.steps());
    follower_2_->register_histogram(time_steps_.steps());

    /* Landing is blocking untill touching the ground*/
    swarm_.land();
    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time: {}", flight_time);

    /* Resetting the entire swarm after the end of each episode*/
    leader_->reset_models();

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

    /*  I have quite tested a lot of different solution. Frankly, if I am going
     * to find a better one, I will replace it directly. */
  }
}
