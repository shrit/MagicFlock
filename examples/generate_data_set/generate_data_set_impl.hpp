#pragma once

template<class QuadrotorType>
Generator<QuadrotorType>::Generator(std::vector<QuadrotorType>& quadrotors,
                                    std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , passed_time_(0.0)
  , episode_time_(0.0)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
  , distribution_int_(0, 3)
  , distribution_int_time(15, 40)
  , generator_(random_dev())
{}

template<class QuadrotorType>
void
Generator<QuadrotorType>::execute_trajectory(QuadrotorType& quad)
{
  if (quad.id() != 0)
    swarm_.one_quad_execute_trajectory(quad.id(), quad.current_action());
}

template<class QuadrotorType>
void
Generator<QuadrotorType>::run(std::function<void(void)> reset)
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
    /* Resetting the entire swarm after the end of each episode*/
    reset();
    logger_->info("All quadrotors have been reset...");
    std::this_thread::sleep_for(std::chrono::seconds(35));

    logger_->info("Episode : {}", episode_);
    timer_.start();
    time_steps_.reset();
    swarm_.in_air_async(40);
    int random = 0;
    int time_random = 0;
    std::vector<ignition::math::Vector3d> destinations{
      { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }
    };
    ignition::math::Vector3d up{ 0, 0, +1.5 };
    quadrotors_.at(0).current_action().action() = up;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    random = distribution_int_(generator_);
    time_random = distribution_int_time(generator_);
    dest_ = destinations.at(random);
    quadrotors_.at(0).current_action().action() = dest_;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());

    std::this_thread::sleep_for(std::chrono::seconds(time_random));

    /**
     * Collect dataset by creating a specific destination.
     * Each quadrotor use the flocking model to stay close to
     * its neighbors. All quadrotors have the same destination.
     * An episode ends when a quadrotor reachs the destination,
     * or quadrotors are very dispersed.
     */
    /*  Verify that vectors are clear when starting new episode */
    logger_->info("Taking off has finished. Start the flocking model");
    ignition::math::Vector4d gains{ 1, 7, 1, 100 };
    ignition::math::Vector3d max_speed{ 1, 1, 0.0 };
    ignition::math::Vector4d axis_speed{ 0.1, 0.1, 0.09, 4 };

    Timer model_time;
    model_time.start();
    int count = 0;
    bool is_leader = true;

    std::function<void(QuadrotorType&, QuadrotorType&)> action_model =
      [&](QuadrotorType& leader, QuadrotorType& quad) {
        // if (count % 2 == 0) {
        if (quad.id() != 0) {
          logger_->info("Start the flocking model");
          logger_->info("quad id {}", quad.id());
          quad.flocking_model(gains, leader.position(), max_speed, is_leader);
        }
        // } else {
        // quad.random_model(axis_speed, elapsed_time_);
        // }
      };

    std::function<void(QuadrotorType&)> trajectory = [&](QuadrotorType& quad) {
      this->execute_trajectory(quad);
    };

    /* Let us see how these quadrotors are going to move */
    while (true) {
      bool shape = swarm_.examin_swarm_shape(0.1, 30);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }

      elapsed_time_ = model_time.stop();
      logger_->info("Model time in seconds {}", elapsed_time_);

      if (elapsed_time_ > passed_time_ + 10) {
        logger_->info("Change the leader destination {}", elapsed_time_);
        passed_time_ = elapsed_time_;
        random = distribution_int_(generator_);
        count++;
      }

      std::vector<std::thread> threads;
      for (auto&& it : quadrotors_) {
        threads.push_back(std::thread([&]() {
          it.sample_state_action_state(
            action_model, trajectory, it, quadrotors_.at(0));
          logger_->info("Saving dataset!");
          arma::colvec check_double =
            it.current_state().Data() - it.last_state().Data();
          if (!check_double.is_zero()) {
            it.save_dataset_sasas();
          }
        }));
      }

      for (auto& thread : threads) {
        thread.join();
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(250));

      if (count % 2 == 0) {
        logger_->info("Change leader destination NOW");
        dest_ = destinations.at(random);
        // leader actions
        quadrotors_.at(0).current_action().action() = dest_;
        swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                           quadrotors_.at(0).current_action());
      }

      shape = swarm_.examin_swarm_shape(0.2, 50);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      if (elapsed_time_ > episode_time_ + 120) {
        episode_time_ = elapsed_time_;
        break;
      }
    }

    passed_time_ = 0;
    episode_time_ = 0;
    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time: {}", flight_time);
    /* Landing is blocking untill all quadrotors in the swarm touch the
     * ground */
    swarm_.stop_offboard_mode_async();
    std::this_thread::sleep_for(std::chrono::seconds(1));
    swarm_.land();

    swarm_.disarm_async();
    /* Wait to be sure that all of the quads have disarmed */
    std::this_thread::sleep_for(std::chrono::seconds(3));

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
  }
}
