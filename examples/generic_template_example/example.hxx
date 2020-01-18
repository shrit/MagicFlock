#pragma once

template<class flight_controller_t, class simulator_t>
Example<flight_controller_t, simulator_t>::Example(
  std::vector<std::shared_ptr<flight_controller_t>> quads,
  const std::vector<Quadrotor<simulator_t>>& quadrotors,
  std::shared_ptr<simulator_t> sim_interface,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , sim_interface_(std::move(sim_interface))
  , swarm_(std::move(quads))
  , quadrotors_(std::move(quadrotors))
  , logger_(logger)
{
  /*  Allow easier access and debugging to all quadrotors */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
  leader_2_ = std::next(quadrotors_.begin(), 3);
}

/* This function allow users to generate trajectory for quadrotors,
 * the function is called after the taking off process have been completed.
 * The user can make quadrotors move at the same time steps or at different time
 * steps.
 * */
template<class flight_controller_t, class simulator_t>
void
Example<flight_controller_t, simulator_t>::generate_trajectory()
{
  /* Action class have several different type of action generators.*/
  Actions action;
  std::vector<std::thread> threads;

  /*  In discrete time we have to sample state at this time steps in order to
   * recover the state of the follower, users can choose to sample states for
   * leaders and the followers. */
  follower_1_->sample_state();
  follower_2_->sample_state();

  /* Example for genenrate action for the leaders*/
  leader_->current_action(action.random_action_generator());
  leader_2_->current_action(leader_->current_action());

  /*  Do not allow follower to move, at this time step,
        block the follower and log not move*/
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  /*  Thread are used to ensure quadrotors are moving togother at the same time
   * (in this case for the leaders */
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

  /* Get the next state at time t + 1  */
  logger::logger_->info("Sampling states at t +1");

  follower_1_->sample_state();
  follower_2_->sample_state();

  follower_1_->current_action(action.random_action_generator());
  follower_2_->current_action(action.random_action_generator());

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(follower_1_->id(),
                                       follower_1_->current_action(),
                                       follower_1_->speed(),
                                       1000);
  }));

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(follower_2_->id(),
                                       follower_2_->current_action(),
                                       follower_2_->speed(),
                                       1000);
  }));

  for (auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  /* Get the next state at time t + 2  */
  logger::logger_->info("Sampling states at t+2");
  follower_1_->sample_state();
  follower_2_->sample_state();
}

template<class flight_controller_t, class simulator_t>
void
Example<flight_controller_t, simulator_t>::run()
{
  /*Flying episodes, each episode consiste of taking off the quadrotors,
   * flying for a specific amount of time, and at the end of the episode, all
   * the quadrotors land and reset to their  original position*/
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger::logger_->info("Episode : {}", episode_);

    start_episode_ = swarm_.in_air(25);

    if (start_episode_) {
      /* We reset time steps in each new episode */
      time_steps_.reset();
      /* This loop is called at the end of taking off process. Here, we ask
       * quadrotors to execute trajectories, then we examin the geometrical
       * shape to see if the quadrotors are far from each other or too close. If
       * this is the case, we land the swam and reset the quadrotors to start a
       * new episode*/
      while (start_episode_) {
        generate_trajectory();

        follower_1_->register_data_set();
        follower_2_->register_data_set();

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
          logger::logger_->info(
            "The geometrical figure is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger::logger_->flush();
      }
    }
    /*  Save a version of the time steps to create a histogram */
    follower_1_->register_histogram(time_steps_.steps());
    follower_2_->register_histogram(time_steps_.steps());

    /* Landing is blocking untill all the quadrotors touch the ground*/
    swarm_.land();

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger::logger_->info("All quadrotors have been reset...");

    std::this_thread::sleep_for(std::chrono::seconds(25));

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
