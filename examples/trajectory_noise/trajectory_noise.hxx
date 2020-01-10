#pragma once

#include "trajectory_noise.hh"

template<class flight_controller_t, class simulator_t>
TrajectoryNoise<flight_controller_t, simulator_t>::TrajectoryNoise(
  std::vector<std::shared_ptr<flight_controller_t>> quads,
  const std::vector<Quadrotor<simulator_t>>& quadrotors,
  std::shared_ptr<simulator_t> sim_interface,
  std::shared_ptr<spdlog::logger> logger)
  : count_(0)
  , episode_(0)
  , max_episode_(1)
  , sim_interface_(std::move(sim_interface))
  , swarm_(std::move(quads))
  , quadrotors_(std::move(quadrotors))
  , logger_(logger)
{
  data_set_.init_dataset_directory();
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
}

template<class flight_controller_t, class simulator_t>
void
TrajectoryNoise<flight_controller_t, simulator_t>::test_trajectory(
  bool stop_down_action)
{

  Actions action;
  std::vector<std::thread> threads;

  leader_->current_action(action_.random_action_generator_with_all_conditions(
    leader_->last_action()));
  if (stop_down_action == true) {
    while (leader_->current_action() == Actions::Action::down) {
      leader_->current_action(
        action_.random_action_generator_with_all_conditions(
          leader_->last_action()));
    }
  }

  follower_1_->sample_state();
  follower_2_->sample_state();

  /*  Execute a trajectory for 1 seconds */

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  follower_1_->sample_state();
  follower_2_->sample_state();

  threads.push_back(std::thread([&]() {
    logger::logger_->info("Current action follower 1: {}",
                          action.action_to_str(follower_1_->current_action()));
    swarm_.one_quad_execute_trajectory(
      follower_1_->id(), leader_->current_action(), follower_1_->speed(), 1000);
  }));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  follower_2_->sample_state();

  threads.push_back(std::thread([&]() {
    logger::logger_->info("Current action follower 2: {}",
                          action.action_to_str(follower_2_->current_action()));
    swarm_.one_quad_execute_trajectory(
      follower_2_->id(), leader_->current_action(), follower_2_->speed(), 1000);
  }));

  for (auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  follower_2_->sample_state();
}

template<class flight_controller_t, class simulator_t>
void
TrajectoryNoise<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads:
     * Arm + takeoff + offboard mode + moving + land
     * Then: repeat each episode.
     */
    logger_->info("Episode : {}", episode_);
    start_episode_ = swarm_.in_air(20);
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (start_episode_) {
      count_ = 0;
      while (count_ < 1000) {
        /*  Verify that the quadrotors is not close to the ground */
        if (sim_interface_->positions().at(1).z < 15) {
          test_trajectory(true);
        } else {
          test_trajectory(false);
        }
        logger_->info("Distance to neighbors: {}",
                      follower_1_->last_state().neighbor_dists_3D());
        logger_->info("Distance to neighbors: {}",
                      follower_2_->last_state().neighbor_dists_3D());

        double d1 = follower_1_->last_state().distance_to(leader_->id());
        double d2 = follower_1_->current_state().distance_to(leader_->id());

        double traveled_distance = std::fabs(d2 - d1);
        logger_->info("Difference of distance between quadrotor and follower "
                      "after action: {}",
                      traveled_distance);
        if (count_ > 0) {
          /* Construct the experinces vector, Study the distribution
             law for the each action over time. This is essential as
             the distance traveled by each action are not the same for
             the same amount of time */

          /*  ADD PAST A_t-1 distances */

          if (leader_->current_action() == Actions::Action::forward and
              leader_->last_action() == Actions::Action::left) {
            logger_->info("F + L");
            forward_action_vec_.push_back(traveled_distance);
            f_k_l_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::forward and
                     leader_->last_action() == Actions::Action::right) {
            logger_->info("F + R");
            forward_action_vec_.push_back(traveled_distance);
            f_k_r_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::forward and
                     leader_->last_action() == Actions::Action::up) {
            logger_->info("F + U");
            forward_action_vec_.push_back(traveled_distance);
            f_k_u_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::forward and
                     leader_->last_action() == Actions::Action::down) {
            logger_->info("F + D");
            forward_action_vec_.push_back(traveled_distance);
            f_k_d_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::backward and
                     leader_->last_action() == Actions::Action::left) {
            logger_->info("B + L");
            backward_action_vec_.push_back(traveled_distance);
            b_k_l_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::backward and
                     leader_->last_action() == Actions::Action::right) {
            logger_->info("B + R");
            backward_action_vec_.push_back(traveled_distance);
            b_k_r_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::backward and
                     leader_->last_action() == Actions::Action::up) {
            logger_->info("B + U");
            backward_action_vec_.push_back(traveled_distance);
            b_k_u_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::backward and
                     leader_->last_action() == Actions::Action::down) {
            logger_->info("B + D");
            backward_action_vec_.push_back(traveled_distance);
            b_k_d_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::left and
                     leader_->last_action() == Actions::Action::forward) {
            logger_->info("L + F");
            left_action_vec_.push_back(traveled_distance);
            l_k_f_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::left and
                     leader_->last_action() == Actions::Action::backward) {
            logger_->info("L + B");
            left_action_vec_.push_back(traveled_distance);
            l_k_b_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::left and
                     leader_->last_action() == Actions::Action::up) {
            logger_->info("L + U");
            left_action_vec_.push_back(traveled_distance);
            l_k_u_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::left and
                     leader_->last_action() == Actions::Action::down) {
            logger_->info("L + D");
            left_action_vec_.push_back(traveled_distance);
            l_k_d_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::right and
                     leader_->last_action() == Actions::Action::forward) {
            logger_->info("R + F");
            right_action_vec_.push_back(traveled_distance);
            r_k_f_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::right and
                     leader_->last_action() == Actions::Action::backward) {
            logger_->info("R + B");
            right_action_vec_.push_back(traveled_distance);
            r_k_b_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::right and
                     leader_->last_action() == Actions::Action::up) {
            logger_->info("R + U");
            right_action_vec_.push_back(traveled_distance);
            r_k_u_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::right and
                     leader_->last_action() == Actions::Action::down) {
            logger_->info("R + D");
            right_action_vec_.push_back(traveled_distance);
            r_k_d_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::up and
                     leader_->last_action() == Actions::Action::forward) {

            up_action_vec_.push_back(traveled_distance);
            u_k_f_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::up and
                     leader_->last_action() == Actions::Action::backward) {

            up_action_vec_.push_back(traveled_distance);
            u_k_b_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::up and
                     leader_->last_action() == Actions::Action::left) {

            up_action_vec_.push_back(traveled_distance);
            u_k_l_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::up and
                     leader_->last_action() == Actions::Action::right) {

            up_action_vec_.push_back(traveled_distance);
            u_k_r_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::down and
                     leader_->last_action() == Actions::Action::forward) {
            down_action_vec_.push_back(traveled_distance);
            d_k_f_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::down and
                     leader_->last_action() == Actions::Action::backward) {
            down_action_vec_.push_back(traveled_distance);
            d_k_b_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::down and
                     leader_->last_action() == Actions::Action::left) {
            down_action_vec_.push_back(traveled_distance);
            d_k_l_action_vec_.push_back(traveled_distance);

          } else if (leader_->current_action() == Actions::Action::down and
                     leader_->last_action() == Actions::Action::right) {
            down_action_vec_.push_back(traveled_distance);
            d_k_r_action_vec_.push_back(traveled_distance);
          }
        }
        ++count_;
      }
      /*  Calculate the mean value */ /*  Replace logging with a file */
      logger_->info("Mean of Forward: {}", mtools_.mean(forward_action_vec_));
      logger_->info("Mean of Backward: {}", mtools_.mean(backward_action_vec_));
      logger_->info("Mean of Left: {}", mtools_.mean(left_action_vec_));
      logger_->info("Mean of Right: {}", mtools_.mean(right_action_vec_));
      logger_->info("Mean of up: {}", mtools_.mean(up_action_vec_));
      logger_->info("Mean of down: {}", mtools_.mean(down_action_vec_));

      logger_->info("Mean of Forward knowing left: {}",
                    mtools_.mean(f_k_l_action_vec_));
      logger_->info("Mean of Forward knowing right: {}",
                    mtools_.mean(f_k_r_action_vec_));
      logger_->info("Mean of Forward knowing up: {}",
                    mtools_.mean(f_k_u_action_vec_));
      logger_->info("Mean of Forward knowing down: {}",
                    mtools_.mean(f_k_d_action_vec_));

      logger_->info("Mean of backward knowing left: {}",
                    mtools_.mean(b_k_l_action_vec_));
      logger_->info("Mean of backward knowing right: {}",
                    mtools_.mean(b_k_r_action_vec_));
      logger_->info("Mean of backward knowing up: {}",
                    mtools_.mean(b_k_u_action_vec_));
      logger_->info("Mean of backward knowing down: {}",
                    mtools_.mean(b_k_d_action_vec_));

      logger_->info("Mean of left knowing forward: {}",
                    mtools_.mean(l_k_f_action_vec_));
      logger_->info("Mean of left knowing backward: {}",
                    mtools_.mean(l_k_b_action_vec_));
      logger_->info("Mean of left knowing up: {}",
                    mtools_.mean(l_k_u_action_vec_));
      logger_->info("Mean of left knowing down: {}",
                    mtools_.mean(l_k_d_action_vec_));

      logger_->info("Variance of Forward: {}",
                    mtools_.variance(forward_action_vec_));
      logger_->info("Variance of Backward: {}",
                    mtools_.variance(backward_action_vec_));
      logger_->info("Variance of Left: {}", mtools_.variance(left_action_vec_));
      logger_->info("Variance of Right: {}",
                    mtools_.variance(right_action_vec_));
      logger_->info("Variance of up: {}", mtools_.variance(up_action_vec_));
      logger_->info("Variance of down: {}", mtools_.variance(down_action_vec_));

      logger_->info("Variance of Forward knowing left: {}",
                    mtools_.variance(f_k_l_action_vec_));
      logger_->info("Variance of Forward knowing right: {}",
                    mtools_.variance(f_k_r_action_vec_));
      logger_->info("Variance of Forward knowing up: {}",
                    mtools_.variance(f_k_u_action_vec_));
      logger_->info("Variance of Forward knowing down: {}",
                    mtools_.variance(f_k_d_action_vec_));

      logger_->info("Variance of backward knowing left: {}",
                    mtools_.variance(b_k_l_action_vec_));
      logger_->info("Variance of backward knowing right: {}",
                    mtools_.variance(b_k_r_action_vec_));
      logger_->info("Variance of backward knowing up: {}",
                    mtools_.variance(b_k_u_action_vec_));
      logger_->info("Variance of backward knowing down: {}",
                    mtools_.variance(b_k_d_action_vec_));

      logger_->info("Variance of left knowing forward: {}",
                    mtools_.variance(l_k_f_action_vec_));
      logger_->info("Variance of left knowing backward: {}",
                    mtools_.variance(l_k_b_action_vec_));
      logger_->info("Variance of left knowing up: {}",
                    mtools_.variance(l_k_u_action_vec_));
      logger_->info("Variance of left knowing down: {}",
                    mtools_.variance(l_k_d_action_vec_));
    }

    /*  Landing */
    swarm_.land();

    /*  Resetting */
    sim_interface_->reset_models();
    logger_->info("All quadrotors have been reset...");

    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}
