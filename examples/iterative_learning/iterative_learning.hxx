#pragma once

#include "iterative_learning.hh"

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
}

template<class flight_controller_t, class simulator_t>
void
Iterative_learning<flight_controller_t, simulator_t>::
  generate_trajectory_using_model(bool change_leader_action,
                                  bool stop_down_action)
{
  Actions action;
  std::vector<std::thread> threads;

  /*  Sample the state at time t */
  leader_->sample_state();
  follower_1_->sample_state();
  follower_2_->sample_state();

  /*  Pick leader action, change it or keep it */

  Actions::Action leader_action = action.generate_leader_action(
      change_leader_action, stop_down_action, leader_->last_action());
  
  if (leader_action == Actions::Action::up or leader_action == Actions::Action::down) {
    leader_action = action.generate_leader_action(
      change_leader_action, stop_down_action, leader_->last_action());
  }

  if (time_steps_.steps() != 0) {
    leader_action = action.validate_leader_action(
      leader_->current_state().distance_to(follower_2_->id()),
      leader_->last_state().distance_to(follower_2_->id()),
      leader_->current_state().distance_to(follower_1_->id()),
      leader_->last_state().distance_to(follower_1_->id()),
      leader_action);
  }
  leader_->current_action(leader_action);

  /* Followers actions always equal to no move at this instant t */
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  logger_->info("Leader (Alice) action: {}",
                action.action_to_str(leader_->current_action()));

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&]() {
    swarm_.one_quad_execute_trajectory(
      leader_->id(), leader_->current_action(), leader_->speed(), 1000);
  }));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1500));

  /*  Sample the state at time t + 1*/
  follower_1_->sample_state();
  follower_2_->sample_state();

  Predictor<simulator_t> predict_f1(
    "regression",
    "/meta/lemon/examples/iterative_learning/build/f1/model.txt",
    "model",
    follower_1_);

  Actions::Action follower_1_action = predict_f1.get_predicted_action();

  Predictor<simulator_t> predict_f2(
    "regression",
    "/meta/lemon/examples/iterative_learning/build/f2/model.txt",
    "model",
    follower_2_);

  Actions::Action follower_2_action = predict_f2.get_predicted_action();

  // if (follower_1_action == follower_2_action) {
  follower_1_->current_action(follower_1_action);
  follower_2_->current_action(follower_2_action);
  follower_1_->save_action_in_container(follower_1_action);
  follower_2_->save_action_in_container(follower_2_action);

  logger_->info("Follower 1 (Chalrie) action: {}",
                action.action_to_str(follower_1_->current_action()));

  logger_->info("Follower 2 (Bob) action: {}",
                action.action_to_str(follower_2_->current_action()));

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

  /* Take a tuple here  */
  double loss_f1 = predict_f1.real_time_loss();
  double loss_f2 = predict_f2.real_time_loss();
  logger::logger_->info("Real time loss f1: {} ", loss_f1);
  logger::logger_->info("Real time loss f2: {}", loss_f2);
}

template<class flight_controller_t, class simulator_t>
void
Iterative_learning<flight_controller_t, simulator_t>::run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    logger::logger_->info("Episode : {}", episode_);

    start_episode_ = swarm_.in_air(25);

    if (start_episode_) {
      time_steps_.reset();
      while (start_episode_) {
        if (time_steps_.steps() % 10 == 0) {
          generate_trajectory_using_model(true, false);
          // Change each 10 times the direction of the leader
        } else if (leader_->height() < 15) {
          generate_trajectory_using_model(false, true);
        } else {
          generate_trajectory_using_model(false, false);
        }

        follower_1_->register_data_set();
        follower_2_->register_data_set();

        follower_1_->reset_all_states();
        follower_2_->reset_all_states();

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
          logger::logger_->info("The geometrical shape is no longer conserved");
          break;
        }
        time_steps_.tic();
        logger::logger_->flush();
      }
    }
    follower_1_->register_histogram(time_steps_.steps());
    follower_2_->register_histogram(time_steps_.steps());
    swarm_.land();

    logger::logger_->info("Model evaluation: Both Same Action as leader : {}",
                          evaluate_model.output().at(0));
    logger::logger_->info("Follower 1 and leader same action {}",
                          evaluate_model.output().at(1));
    logger::logger_->info("Follower 2 and leader same action {}",
                          evaluate_model.output().at(2));
    logger::logger_->info("Bad action follower 1 {}",
                          evaluate_model.output().at(3));
    logger::logger_->info("Bad action follower 2 {}",
                          evaluate_model.output().at(4));
    logger::logger_->info("good action follower 1 {}",
                          evaluate_model.output().at(5));
    logger::logger_->info("good action follower 2 {}",
                          evaluate_model.output().at(6));
    logger::logger_->info("Total count: {}", evaluate_model.output().at(7));

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger::logger_->info("The quadcopters have been reset...");
    logger::logger_->info("Waiting untill the kalaman filter to reset...");
    std::this_thread::sleep_for(std::chrono::seconds(25));
    logger::logger_->info("Kalaman filter reset...");
    logger::logger_->flush();
  }
}
