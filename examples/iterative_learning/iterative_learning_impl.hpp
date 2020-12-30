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
    int timeSteps = 0;
    sched.start();
    // bool flocking = true;
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

    ignition::math::Vector4d gains{ 1, 7, 1, 100 };
    ignition::math::Vector3d max_speed{ 1, 1, 0.3 };

    std::function<void(QuadrotorType&, QuadrotorType&)> action_model =
      [&](QuadrotorType& leader, QuadrotorType& quad) {
        // if (flocking) {
        //   if (quad.id() != 0)
        //     quad.flocking_model(gains, leader.position(), max_speed,
        //     is_leader);
        // } else {
        if (quad.id() != 0) {
          AnnStatePredictor<QuadrotorType> ann(quad);
          DiscretActions mig_action = ann.best_predicted_mig_action(
            "/meta/MagicFlock/examples/iterative_learning/build/leader/model.bin",
            "model");
          DiscretActions cohsep_action = ann.best_predicted_cohsep_action(
            "/meta/MagicFlock/examples/iterative_learning/build/followers/"
            "model.bin",
            "model");
          // quad.predicted_actions(
          //   mig_action.action_to_int(mig_action.action()));

          quad.current_action().action() = mig_action.action();
          // + cohsep_action.followers_action();
          // i.current_action()._action().Z() = 0;
        }
        //  }
      };

    std::function<void(QuadrotorType & quad)> trajectory =
      [&](QuadrotorType& quad) { this->execute_trajectory(quad); };

    while (true) {
      // Check shape after taking off
      bool shape = swarm_.examin_swarm_shape(0.1, 35);
      if (!shape) {
        logger_->info("Quadrotors are far from each other, ending the episode");
        break;
      }
      // increase time steps.
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      timeSteps++;

      std::vector<std::thread> threads;
      for (auto&& it : quadrotors_) {
        threads.push_back(std::thread([&]() {
          it.sample_state_action_state(
            action_model, trajectory, it, quadrotors_.at(0));
          logger_->info("Saving dataset NOW");
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

      // Leader changes its action each 10 times steps.
      if (timeSteps % 20 == 0) {
        logger_->info("Change leader destination NOW");
        random = distribution_int_(generator_);
        dest_ = destinations.at(random);
        quadrotors_.at(0).current_action().action() = dest_;
        swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                           quadrotors_.at(0).current_action());
      }

      for (auto&& it : quadrotors_) {
        if (it.id() != 0) {
          if (!it.predicted_actions().empty()) {
            // arma::colvec predicted =
            // arma_.vec_to_arma(it.predicted_actions()); arma::colvec pdf =
            // arma::normpdf(
            //   predicted, arma::mean(predicted), arma::stddev(predicted));
            // logger::logger_->info("The PDF of actions is {}", pdf);
            // DiscretActions action;
            // it.current_action() = action.int_to_action(pdf.index_max());
            // it.all_actions().push_back(it.current_action());
            // it.predicted_actions().clear();
          }
        }
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
