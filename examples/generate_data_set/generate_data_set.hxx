#pragma once

template<class QuadrotorType>
Generator<QuadrotorType>::Generator(
  std::vector<QuadrotorType>& quadrotors,
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
Generator<QuadrotorType>::go_to_destination()
{
  std::vector<std::thread> threads;

  /*  Threading Quadrotors */
  for (auto it : quadrotors_) {
    threads.push_back(std::thread([&]() {

    }));
  }
  /* Move the Examination the geomatrical shape inside the flocking model*/
  // for (auto it : quadrotors_) {
  //   shapes.push_back(it.examin_geometric_shape());
  // }
  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
Generator<QuadrotorType>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    timer_.start();
    //start_episode_ = 
    swarm_.in_air_async(15);

    /* Collect dataset by creating a specific destination.
     * Each quadrotor use the flocking model to stay close to
     * their neighbors. All quadrotors have the same destination.
     * An episode ends when a quadrotor reachs the destination,
     * or quadrotors are very dispersed.
     */

    std::this_thread::sleep_for(std::chrono::seconds(20));
    // if (start_episode_) {
    //   /*  Verify that vectors are clear when starting new episode */
    //   logger_->info("Taking off has finished. Start sampling dataset");
    //   for (auto it : quadrotors_) {
    //     it->start_sampling_rt_state(50);
    //   }
    //   time_steps_.reset();
    //   //go_to_destination();

    //   // follower_1_->register_data_set();
    //   // follower_2_->register_data_set();

    //   // follower_1_->register_episodes(episode_);

    //   logger_->flush();
    // }

    // /*  Save a version of the time steps to create a histogram */
    // follower_1_->register_histogram(time_steps_.steps());
    // follower_2_->register_histogram(time_steps_.steps());

    // for (auto it : quadrotors_) {
    //   it->stop_sampling_rt_state();
    // }

    // std::string flight_time = timer_.stop_and_get_time();
    // logger_->info("Flight time: {}", flight_time);
    /* Landing is blocking untill all quadrotors in the swarm touch the
     * ground */
    swarm_.stop_offboard_mode_async();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    swarm_.land();

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
