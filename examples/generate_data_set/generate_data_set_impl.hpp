#pragma once

template<class QuadrotorType>
Generator<QuadrotorType>::Generator(std::vector<QuadrotorType>& quadrotors,
                                    std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , start_episode_(false)
  , passed_time_(0.0)
  , swarm_(quadrotors)
  , quadrotors_(quadrotors)
  , logger_(logger)
  , distribution_int_(0, 3)
  , generator_(random_dev())
{}

template<class QuadrotorType>
void
Generator<QuadrotorType>::go_to_destination(int count)
{
  std::vector<std::thread> threads;
  /*  Threading Quadrotors */
  for (auto&& it : quadrotors_) {
    threads.push_back(std::thread([&]() {
      swarm_.one_quad_execute_trajectory(it.id(), it.current_action());
    }));
  }

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class QuadrotorType>
void
Generator<QuadrotorType>::stop()
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

    ignition::math::Vector3d up{ 0, 0, -1.5 };
    quadrotors_.at(0).current_action().action() = up;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    ignition::math::Vector3d forward{ 1, 0, 0 };
    quadrotors_.at(0).current_action().action() = forward;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(5));

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
    ignition::math::Vector3d max_speed{ 1.5, 1.5, 0 };
    ignition::math::Vector4d axis_speed{ 0.35, 0.35, 0.15, 4 };

    //! This destination goes forward
    ignition::math::Vector3d destination_forward{ 163, 0, 40 };
    //! This destination goes backward
    ignition::math::Vector3d destination_backword{ -163, 0, 20 };

    //! This destination goes left
    ignition::math::Vector3d destination_left{ 0, 163, 20 };

    //! This destination goes right
    ignition::math::Vector3d destination_right{ 0, -163, 20 };
    Timer model_time;
    model_time.start();
    int count = 0;
    // Check the shape of the swarm, if one is missing then land.

    bool leader = true;
    std::function<void(void)> action_model = [&]() {
      if (count % 2 == 0) {
        for (std::size_t i = 1; i < quadrotors_.size(); ++i) {
          logger_->info("Start the flocking model");
          quadrotors_.at(i).start_flocking_model(
            gains, quadrotors_.at(0).position(), max_speed, leader);
        }
      } else {
        for (auto&& it : quadrotors_) {
          it.random_model(axis_speed, elapsed_time_);
        }
      }
    };

    std::function<void(void)> trajectory = [&]() {
      this->go_to_destination(count);
    };

    /* Let us see how these quadrotors are going to move */
    while (true) {
      bool shape = swarm_.examin_swarm_shape(0.3, 16);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      elapsed_time_ = model_time.stop();
      logger_->info("Model time in seconds {}", elapsed_time_);
      if (elapsed_time_ > passed_time_ + 3) {
        logger_->info("Seconds have passed change the model");
        passed_time_ = elapsed_time_;
        count++;
      }
      if (count % 2 == 0) {
        for (auto&& it : quadrotors_) {
          it.sample_state_action_state(action_model, trajectory);
        }
      }

      std::vector<ignition::math::Vector3d> destinations{
        { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }
      };

      int random = distribution_int_(generator_);
      if (count % 4 == 0) {
        logger_->info("Change leader destination NOW");
        dest_ = destinations.at(random);
      }
      quadrotors_.at(0).current_action().action() = dest_;

      for (auto&& it : quadrotors_) {
        // Let us see if these are still necessary
        arma::colvec check_double =
          it.current_state().Data() - it.last_state().Data();
        if (!check_double.is_zero())
          it.save_dataset_sasas();
      }

      shape = swarm_.examin_swarm_shape(0.2, 10);
      // bool has_arrived = swarm_.examin_destination(destination_forward);

      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      // if (has_arrived) {
      //   logger_->info("Quadrotors have arrived at specified destination. "
      //                 "ending the episode.");
      //   break;
      // }
    }
    passed_time_ = 0;
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
