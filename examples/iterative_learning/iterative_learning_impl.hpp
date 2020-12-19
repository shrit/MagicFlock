#pragma once

template<class QuadrotorType>
Iterative_learning<QuadrotorType>::Iterative_learning(
  std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , passed_time_(0.0)
  , episode_time_(0.0)
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
Iterative_learning<QuadrotorType>::execute_trajectory(QuadrotorType& quad)
{
  if (quad.id() != 0)
    swarm_.one_quad_execute_trajectory(quad.id(), quad.current_action());
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
    // bool is_leader = true;
    std::vector<ignition::math::Vector3d> destinations{
      { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }
    };

    ignition::math::Vector3d up{ 0, 0, +1.5 };
    quadrotors_.at(0).current_action().action() = up;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(1));
    random = distribution_int_(generator_);
    dest_ = destinations.at(random);
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

    std::function<void(QuadrotorType&, QuadrotorType&)>

      action_model = [&](QuadrotorType& leader, QuadrotorType& quad) {
        // if (count % 2 == 0) {
        //   if (quad.id() != 0)
        //     quad.flocking_model(gains, leader.position(), max_speed,
        //     is_leader);

        // } else {
        if (quad.id() != 0) {
          AnnStatePredictor<QuadrotorType> ann(quad);
          DiscretActions mig_action = ann.best_predicted_mig_action(
            "/meta/lemon/examples/iterative_learning/build/leader/model.bin",
            "model");
          DiscretActions cohsep_action = ann.best_predicted_cohsep_action(
            "/meta/lemon/examples/iterative_learning/build/followers/"
            "model.bin",
            "model");
          quad.current_action().action() = mig_action.action();
          // + cohsep_action.action();
          // i.current_action().action().Z() = 0;
          quad.all_actions().push_back(quad.current_action());
        }
        // }
      };

    std::function<void(QuadrotorType & quad)> trajectory =
      [&](QuadrotorType& quad) { this->execute_trajectory(quad); };

    while (true) {
      bool shape = swarm_.examin_swarm_shape(0.1, 35);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      elapsed_time_ = model_time.stop();

      if (elapsed_time_ > passed_time_ + 2) {
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
          logger_->info("Saving dataset NOW!!!!!!!!!!!!!");
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
        quadrotors_.at(0).current_action().action() = dest_;
        swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                           quadrotors_.at(0).current_action());
      }

      /*  Check the geometrical shape */
      shape = swarm_.examin_swarm_shape(0.2, 35);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      if (elapsed_time_ > passed_time_ + 60) {
        episode_time_ = elapsed_time_;
        break;
      }
      /* Register results */
      /* Save position of quadrotor each 200 ms*/
      // for (auto&& it : quadrotors_) {
      //   it.save_position(std::to_string(episode_));
      // }
      // double maxD = max_distance_.check_global_distance(quadrotors_);
      // double minD = min_distance_.check_global_distance(quadrotors_);

      // quadrotors_.at(0).save_values("distance_metric", maxD, minD);
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
