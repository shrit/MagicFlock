#pragma once

#include "real_time_trajectory.hh"

template<class flight_controller_t, class simulator_t>
RTTrajectory<flight_controller_t, simulator_t>::RTTrajectory(
  std::vector<std::shared_ptr<flight_controller_t>> quads,
  const std::vector<Quadrotor<simulator_t>>& quadrotors,
  std::shared_ptr<simulator_t> sim_interface,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
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
RTTrajectory<flight_controller_t, simulator_t>::start_trajectory(
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

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  threads.push_back(std::thread([&]() {
    logger::logger_->info("Current action follower 1: {}",
                          action.action_to_str(leader_->current_action()));
    swarm_.one_quad_execute_trajectory(
      follower_1_->id(), leader_->current_action(), follower_1_->speed(), 1000);
  }));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      follower_2_->id(), leader_->current_action(), follower_2_->speed(), 1000);
  }));

  for (auto& thread : threads) {
    thread.join();
  }
}

template<class flight_controller_t, class simulator_t>
void
RTTrajectory<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    start_episode_ = swarm_.in_air(20);
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (start_episode_) {
      time_steps_.reset();
      leader_->start_sampling_rt_state(50);
      follower_1_->start_sampling_rt_state(50);
      follower_2_->start_sampling_rt_state(50);
      while (time_steps_.steps() < 20) {
        /*  Verify that the quadrotors is not close to the ground */
        if (leader_->height() < 15) {
          start_trajectory(true);
        } else {
          start_trajectory(false);
        }

        time_steps_.tic();

        std::vector<State<simulator_t>> states = leader_->all_states();
        std::vector<double> f1_dist;
        for (auto i : states) {
          f1_dist.push_back(i.distance_to(follower_1_->id()));
          logger_->info("Distance in real time: {}",
                        i.distance_to(follower_1_->id()));
        }
        Actions action;
        for (auto it : f1_dist) {
          data_set_.save_csv_data_set_2_file(
            action.action_to_str(leader_->current_action()) +
              std::to_string(time_steps_.steps()),
            it);
        }

        leader_->reset_all_states();
        follower_1_->reset_all_states();
        follower_2_->reset_all_states();
      }
      leader_->stop_sampling_rt_state();
      follower_1_->stop_sampling_rt_state();
      follower_2_->stop_sampling_rt_state();

      /*  Landing */
      swarm_.land();

      /*  Resetting */
      sim_interface_->reset_models();
      logger_->info("All quadrotors have been reset...");

      std::this_thread::sleep_for(std::chrono::seconds(15));
    }
  }
}
