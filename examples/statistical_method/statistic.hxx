#pragma once

#include "statistic.hh"

template<class flight_controller_t, class simulator_t>
Statistic<flight_controller_t, simulator_t>::Statistic(
  std::vector<std::shared_ptr<flight_controller_t>> iris_x,
  const std::vector<Quadrotor<simulator_t>>& quadrotors,
  std::shared_ptr<simulator_t> gzs,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , sim_interface_(std::move(gzs))
  , swarm_(std::move(iris_x))
  , quadrotors_(std::move(quadrotors))
  , logger_(logger)
{
  /*  Allow easier access and debugging to all quadrotors state */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
}

template<class flight_controller_t, class simulator_t>
void
Statistic<flight_controller_t, simulator_t>::generate_trajectory_using_model(
  bool change_leader_action,
  bool stop_down_action)
{
  Actions action;
  std::vector<std::thread> threads;

  /*  Pick leader action, change it or keep it */
  if (change_leader_action == true) {
    leader_->current_action(
      action.random_action_generator_with_only_opposed_condition(
        leader_->last_action()));
  } else if (stop_down_action == true) {
    while (leader_->current_action() == Actions::Action::down) {
      leader_->current_action(
        action.random_action_generator_with_only_opposed_condition(
          leader_->last_action()));
    }
  }
  /*  Sample the state at time t*/
  follower_2_->sample_state();

  /* Followers actions always equal to no move at this instant t */
  follower_2_->current_action(Actions::Action::NoMove);

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      follower_1_->id(), leader_->current_action(), follower_1_->speed(), 1000);
  }));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  logger_->info("Leaders actions: {}",
                action.action_to_str(leader_->current_action()));
  /*  Sample the state at time t + 1*/
  follower_2_->sample_state();

  KnnPredictor<simulator_t> predict_f2(
    "/meta/lemon/dataset/Dataset_one_drone_2019-Oct-19/22:39:42.csv",
    follower_2_);
  predict_f2.predict(4);
  Actions::Action follower_2_action = predict_f2.get_predicted_action();
  follower_2_->current_action(follower_2_action);
  logger_->info("Follower 2 predicited action {}",
                action.action_to_str(follower_2_action));

  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(follower_2_->id(),
                                       follower_2_->current_action(),
                                       follower_2_->speed(),
                                       1000);
  }));

  for (auto& thread : threads) {
    thread.join();
  }
  /*  Sample the state at time t + 2 final state */
  follower_2_->sample_state();
}

template<class flight_controller_t, class simulator_t>
void
Statistic<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads:
     * Arm + takeoff + offboard mode + moving + land
     * Then: repeat each episode.
     */
    logger_->info("Episode :{}", episode_);

    /* Stop the episode if one of the quad has fallen to arm */
    stop_episode_ = false;
    bool arm = swarm_.arm();
    if (!arm)
      stop_episode_ = true;

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(25);
    if (!takeoff)
      stop_episode_ = true;

    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed();

    /*  Switch to offboard mode, Allow the control */
    bool offboard_mode = swarm_.start_offboard_mode();
    if (!offboard_mode)
      stop_episode_ = true;

    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode_) {
      time_steps_.reset();
      while (!stop_episode_) {

        if (time_steps_.steps() == 0) {
          generate_trajectory_using_model(true, false);
          // Change each 10 times the direction of the leader
        } else if (time_steps_.steps() % 10 == 0) {
          generate_trajectory_using_model(true, false);

          // } else if (leader_->current_state().rt_height() < 15 or
          //            follower_1_->current_state().rt_height() < 15) {

          //   generate_trajectory_using_model(false, true);

        } else {
          generate_trajectory_using_model(false, false);
        }

        std::vector<lt::position3D<double>> new_positions =
          sim_interface_->positions();
        logger_->info("New positions :{}", new_positions);

        follower_2_->register_data_set();
        follower_2_->reset_all_states();
        follower_2_->reset_all_actions();

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
          logger_->info("The geometrical is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger_->flush();
      }
    }

    follower_2_->register_histogram(time_steps_.steps());
    swarm_.land();

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger_->info("The quadcopters have been reset...");
    logger_->info("Waiting untill the kalaman filter to reset...");
    std::this_thread::sleep_for(std::chrono::seconds(25));
    logger_->info("Kalaman filter reset...");
  }
}
