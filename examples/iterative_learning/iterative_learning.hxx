#pragma once

template<class flight_controller_t, class simulator_t>
Iterative_learning<flight_controller_t, simulator_t>::Iterative_learning(
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
  leader_2_ = std::next(quadrotors_.begin(), 3);
}

template<class flight_controller_t, class simulator_t>
void
Iterative_learning<flight_controller_t, simulator_t>::
  generate_trajectory_using_model()
{
  ActionGenerator<simulator_t> leader_generator(leader_);
  ActionGenerator<simulator_t> follower_1_generator(follower_1_);
  ActionGenerator<simulator_t> follower_2_generator(follower_2_);  

  std::vector<std::thread> threads;

  /*  Sample the state at time t */
  leader_->sample_state();
  follower_1_->sample_state();
  follower_2_->sample_state();

  leader_->current_action(
    leader_generator.generate_persistant_action(10, time_steps_.steps()));
  leader_2_->current_action(leader_->current_action());

  /* Followers actions always equal to no move at this instant t */
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  logger_->info("Leader (Alice) action: {}",
                leader_generator.action_to_str(leader_->current_action()));

  /*  Threading QuadCopter */
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

  /*  Sample the state at time t + 1*/
  follower_1_->sample_state();
  follower_2_->sample_state();

  AnnEnhancedPredictor<simulator_t> predict_f1(
    "/meta/lemon/examples/iterative_learning/build/f1/model.txt",
    "model",
    "/meta/lemon/examples/iterative_learning/build/error_f1/model.txt",
    "model",
    follower_1_);

  Actions::Action follower_1_action = predict_f1.best_predicted_action();

  AnnEnhancedPredictor<simulator_t> predict_f2(
    "/meta/lemon/examples/iterative_learning/build/f2/model.txt",
    "model",
    "/meta/lemon/examples/iterative_learning/build/error_f2/model.txt",
    "model",
    follower_2_);

  Actions::Action follower_2_action = predict_f2.best_predicted_action();

  follower_1_->current_action(follower_1_action);
  follower_2_->current_action(follower_2_action);

  logger_->info("Follower 1 (Chalrie) action: {}",
                follower_1_generator.action_to_str(follower_1_->current_action()));

  logger_->info("Follower 2 (Bob) action: {}",
                follower_2_generator.action_to_str(follower_2_->current_action()));

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

  /* We need to wait until the quadcopters finish their actions */
  for (auto& thread : threads) {
    thread.join();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /*  Sample the state at time t + 2 */
  follower_1_->sample_state();
  follower_2_->sample_state();

  evaluate_model.input(leader_->current_action(),
                       follower_1_->current_action(),
                       follower_2_->current_action());

  double loss_f1 = predict_f1.real_time_loss();
  double loss_f2 = predict_f2.real_time_loss();
  logger_->info("Real time loss f1: {} ", loss_f1);
  logger_->info("Real time loss f2: {}", loss_f2);
}

template<class flight_controller_t, class simulator_t>
void
Iterative_learning<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger_->info("Episode : {}", episode_);
    timer_.start();
    start_episode_ = swarm_.in_air(25);

    if (start_episode_) {
      time_steps_.reset();
      while (start_episode_) {
        generate_trajectory_using_model();

        leader_->reset_all_actions();
        follower_1_->register_data_set_with_loss();
        follower_2_->register_data_set_with_loss();

        /*  Check the geometrical shape */
        std::vector<bool> shapes;
        for (auto it : quadrotors_) {
          shapes.push_back(it.examin_geometric_shape());
        }
        if (std::any_of(shapes.begin(), shapes.end(), [](const bool& shape) {
              return shape == false;
            })) {
          logger_->info("The geometrical shape is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger_->flush();
      }
    }
    follower_1_->reset_all_states();
    follower_2_->reset_all_states();

    follower_1_->register_histogram(time_steps_.steps());
    follower_2_->register_histogram(time_steps_.steps());
    swarm_.land();

    std::string flight_time = timer_.stop_and_get_time();
    logger_->info("Flight time for this episode:", flight_time);

    logger_->info("Model evaluation: Both Same Action as leader : {}",
                          evaluate_model.output().at(0));
    logger_->info("Follower 1 and leader same action {}",
                          evaluate_model.output().at(1));
    logger_->info("Follower 2 and leader same action {}",
                          evaluate_model.output().at(2));
    logger_->info("Bad action follower 1 {}",
                          evaluate_model.output().at(3));
    logger_->info("Bad action follower 2 {}",
                          evaluate_model.output().at(4));
    logger_->info("good action follower 1 {}",
                          evaluate_model.output().at(5));
    logger_->info("good action follower 2 {}",
                          evaluate_model.output().at(6));
    logger_->info("Total count: {}", evaluate_model.output().at(7));

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger_->info("The quadcopters have been reset...");
    logger_->info("Waiting untill the kalaman filter to reset...");
    std::this_thread::sleep_for(std::chrono::seconds(35));
    logger_->info("Kalaman filter reset...");
    logger_->flush();
  }
}
