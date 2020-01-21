#pragma once

template<class flight_controller_t, class simulator_t>
Statistic<flight_controller_t, simulator_t>::Statistic(
  std::vector<std::shared_ptr<flight_controller_t>> iris_x,
  const std::vector<Quadrotor<simulator_t>>& quadrotors,
  std::shared_ptr<simulator_t> gzs,
  std::shared_ptr<spdlog::logger> logger)
  : episode_(0)
  , max_episode_(10000)
  , sim_interface_(std::move(gzs))
  , start_episode_(false)
  , swarm_(std::move(iris_x))
  , quadrotors_(std::move(quadrotors))
  , logger_(logger)
{
  /*  Allow easier access and debugging to all quadrotors state */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
  leader_2_ = std::next(quadrotors_.begin(), 3);
}

template<class flight_controller_t, class simulator_t>
void
Statistic<flight_controller_t, simulator_t>::generate_trajectory_using_model(
  bool change_leader_action,
  bool stop_going_down)
{
  Actions action;
  std::vector<std::thread> threads;

  /*  Pick leader action, change it or keep it */
  leader_->current_action(
    action.generate_leader_action(change_leader_action,
                                  stop_going_down,
                                  leader_->last_action(),
                                  leader_->current_action()));

  leader_2_->current_action(leader_->current_action());
  /*  Sample the state at time t*/
  follower_1_->sample_state();
  follower_2_->sample_state();

  /* Followers actions always equal to no move at this instant t */
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  /*  Make leader move at the same time step */
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_2_->id(), leader_2_->current_action(), leader_2_->speed(), 1000);
  }));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  logger_->info("Leaders actions: {}",
                action.action_to_str(leader_->current_action()));

  /*  Sample the state at time t + 1*/
  follower_1_->sample_state();
  follower_2_->sample_state();

  KnnPredictor<simulator_t> predict_f1(
    "/meta/lemon/examples/generate_data_set/dataset/2020-Jan-18/17:05:08.csv_follower_1.csv",
    follower_1_);
  predict_f1.predict(4);
  Actions::Action follower_1_action = predict_f1.get_predicted_action();
  follower_1_->current_action(follower_1_action);
  logger_->info("Follower 1 (Charlie) predicited action {}",
                action.action_to_str(follower_1_action));

  KnnPredictor<simulator_t> predict_f2(
    "/meta/lemon/examples/generate_data_set/dataset/2020-Jan-18/17:05:08.csv_follower_2.csv",
    follower_2_);
  predict_f2.predict(4);
  Actions::Action follower_2_action = predict_f2.get_predicted_action();
  follower_2_->current_action(follower_2_action);
  logger_->info("Follower 2 (Bob) predicited action {}",
                action.action_to_str(follower_2_action));

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
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));  

  /*  Sample the state at time t + 2 final state */
  follower_1_->sample_state();
  follower_2_->sample_state();
}

template<class flight_controller_t, class simulator_t>
void
Statistic<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode :{}", episode_);

    /* Stop the episode if one of the quad has fallen to arm */
    start_episode_ = swarm_.in_air(25);

    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (start_episode_) {
      time_steps_.reset();
      while (start_episode_) {

        if (time_steps_.steps() % 10 == 0) {
          generate_trajectory_using_model(true, false);

        } else if (leader_->height() < 15 or leader_2_->height() < 15) {
          generate_trajectory_using_model(false, true);

        } else {
          generate_trajectory_using_model(false, false);
        }

        leader_->reset_all_actions();

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
          logger_->info("The geometrical is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger_->flush();
      }
    }

    follower_1_->register_histogram(time_steps_.steps());
    follower_2_->register_histogram(time_steps_.steps());
    follower_1_->reset_all_states();    
    follower_2_->reset_all_states();
    swarm_.land();

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger_->info("The quadcopters have been reset...");
    logger_->info("Waiting untill the kalaman filter to reset...");
    std::this_thread::sleep_for(std::chrono::seconds(25));
    logger_->info("Kalaman filter reset...");
  }
}
