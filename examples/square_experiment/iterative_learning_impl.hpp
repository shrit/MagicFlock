#pragma once

template<class QuadrotorType>
Iterative_learning<QuadrotorType>::Iterative_learning(
  std::vector<QuadrotorType>& quadrotors,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , passed_time_(0.0)
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

    swarm_.in_air_async(40);

    int timeSteps = 0;
    int leader_change = 0;
    size_t inc = 0;

    Timer model_time;
    model_time.start();

    std::vector<ignition::math::Vector3d> destinations{
      { 1, 0, 0 }, { -1, 0, 0 }, { 0, 1, 0 }, { 0, -1, 0 }
    };

    std::vector<ignition::math::Vector3d> leader_trajectory{
      { -1, 0, 0 }, { -1, 0, 0 }, { 0, -1, 0 }, { 0, -1, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 0, +1, 0 }, { 0, +1, 0 },
      { -1, 0, 0 }, { -1, 0, 0 }, { 0, -1, 0 }, { 0, -1, 0 }, { 1, 0, 0 }, { 1, 0, 0 }, { 0, +1, 0 }, { 0, +1, 0 },
      };

    ignition::math::Vector3d up{ 0, 0, +1.5 };
    quadrotors_.at(0).current_action().action() = up;
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(1));

    quadrotors_.at(0).current_action().action() = destinations.at(2);
    swarm_.one_quad_execute_trajectory(quadrotors_.at(0).id(),
                                       quadrotors_.at(0).current_action());
    std::this_thread::sleep_for(std::chrono::seconds(5));

    ignition::math::Vector4d gains{ 1, 7, 1, 100 };
    ignition::math::Vector3d max_speed{ 1, 1, 0.3 };

    std::function<void(QuadrotorType&, QuadrotorType&)> action_model =
      [&](QuadrotorType& leader, QuadrotorType& quad) {
        logger_->info("EXECUTE THE PREDICTION MODEL NOW");
        if (quad.id() != 0) {
          AnnActionPredictor<QuadrotorType> ann(quad);
          ContinuousActions mig_action = ann.best_predicted_mig_action(
            "/meta/MagicFlock/examples/iterative_learning/build/leader/"
            "model.bin",
            "model");
          ContinuousActions cohsep_action = ann.best_predicted_cohsep_action(
            "/meta/MagicFlock/examples/iterative_learning/build/"
            "followers/"
            "model.bin",
            "model");
          quad.current_action().leader_action() = mig_action.action();
          quad.current_action().followers_action() = cohsep_action.action();
          quad.current_action().action() =
            mig_action.action() + cohsep_action.action();
          quad.all_actions().push_back(quad.current_action());
        }
      };

    std::function<void(QuadrotorType & quad)> trajectory =
      [&](QuadrotorType& quad) { this->execute_trajectory(quad); };

    for (int i = 0; i < 10; ++i) {
      for (auto&& it : quadrotors_) {
        it.sample_state();
      }
    }

    bool shape = swarm_.examin_swarm_shape(0.1, 30);
    if (shape) {
      while (true) {
        // Check shape after taking off
        shape = swarm_.examin_swarm_shape(0.0, 30);
        if (!shape) {
          logger_->info(
            "Quadrotors are far from each other, ending the episode");
          break;
        }
        // increase time steps.
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
        elapsed_time_ = model_time.stop();
        logger_->info("Model time in seconds {}", elapsed_time_);
        leader_change++;
        if (elapsed_time_ > passed_time_ + 5) {
          passed_time_ = elapsed_time_;
          timeSteps++;
          logger_->info("Increase timeSteps {}", timeSteps);
        }

        std::vector<std::thread> threads;
        for (auto&& it : quadrotors_) {
          threads.push_back(std::thread([&]() {
            it.sample_state_action_state(
              action_model, trajectory, it, quadrotors_.at(0));
            logger_->info("Saving dataset NOW");
            if (timeSteps % 2 == 0) {
              arma::colvec check_double =
                it.current_state().Data() - it.last_state().Data();
              if (!check_double.is_zero()) {
                it.save_dataset_ssssa();
              }
            }
            it.save_position();
          }));
        }

        double maxD = max_distance_.check_global_distance_no_leader(quadrotors_);
        double minD = min_distance_.check_global_distance_no_leader(quadrotors_);

        quadrotors_.at(0).save_values("distance_metric", maxD, minD);

        for (auto& thread : threads) {
          thread.join();
        }

        if (leader_change % 20 == 0) {
          if (inc < leader_trajectory.size()) {
            quadrotors_.at(0).current_action().action() =
              leader_trajectory.at(inc);
            swarm_.one_quad_execute_trajectory(
              quadrotors_.at(0).id(), quadrotors_.at(0).current_action());
            inc = inc + 1;
          } else {
            break;
          }
        }

        /*  Check the geometrical shape */
        shape = swarm_.examin_swarm_shape(0.0, 30);
        if (!shape) {
          logger_->info(
            "Quadrotors are far from each other, ending the episode");
          break;
        }
      }
    }
    passed_time_ = 0;

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
