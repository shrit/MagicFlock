#pragma once

# include "iterative_learning.hh"

template <class flight_controller_t,
          class simulator_t>
Iterative_learning<flight_controller_t, simulator_t>::
Iterative_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
                   const std::vector<Quadrotor<simulator_t>>& quadrotors,
                   std::shared_ptr<simulator_t> gzs)
  :episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(gzs)),
   swarm_(std::move(iris_x)),
   quadrotors_(std::move(quadrotors))
{
  /*  Allow easier access and debugging to all quadrotors state */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
}

template <class flight_controller_t,
          class simulator_t>
void Iterative_learning<flight_controller_t,
                        simulator_t>::
generate_trajectory_using_model(bool change_leader_action,
                                bool stop_down_action)
{
  Actions action;
  std::vector<std::thread> threads;

  /*  Pick leader action, change it or keep it */
  if (change_leader_action == true) {
    leader_->current_action(
                            action.random_action_generator_with_only_opposed_condition
                            (leader_->last_action()));
  } else if (stop_down_action == true) {
    while (leader_->current_action() == Actions::Action::down) {
      leader_->current_action(
                              action.random_action_generator_with_only_opposed_condition
                              (leader_->last_action()));
    }
  }

  /*  Sample the state at time t = 0 only for the first follower */
  follower_1_->sample_state();
	follower_2_->sample_state();
  /* Followers actions always equal to no move at this instant t */
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
                                  swarm_.one_quad_execute_trajectory(leader_->id(),
                                                                     leader_->current_action(),
                                                                     leader_->speed(),
                                                                     1000);
                                }));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  /*  Sample the state at time t + 1*/
  follower_1_->sample_state();
  follower_2_->sample_state();

  Predictor<simulator_t> predict_f1("regression",
                                    "/meta/lemon/examples/iterative_learning/build/f1/model.txt",
                                    "model",
                                    follower_1_);

  Actions::Action follower_1_action = predict_f1.get_predicted_action();
  follower_1_->current_action(follower_1_action);

	Predictor<simulator_t> predict_f2("regression",
                                    "/meta/lemon/examples/iterative_learning/build/f2/model.txt",
                                    "model",
                                    follower_2_);

  Actions::Action follower_2_action = predict_f2.get_predicted_action();
  follower_2_->current_action(follower_2_action);

	threads.push_back(std::thread([&](){
                                  swarm_.one_quad_execute_trajectory(follower_1_->id(),
                                                                     follower_1_->current_action(),
                                                                     follower_1_->speed(),1000);
                                }));
  threads.push_back(std::thread([&](){
                                  swarm_.one_quad_execute_trajectory(follower_2_->id(),
                                                                     follower_2_->current_action(),
                                                                     follower_2_->speed(),1000);
                                }));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	evaluate_model.input(leader_->current_action(),
                      follower_1_->current_action(),
                      follower_2_->current_action());

/*  Sample the state at time t + 2 */
  follower_1_->sample_state();
  follower_2_->sample_state();

  for (auto& thread : threads) {
    thread.join();
  }

  /* Take a tuple here  */
  double loss_f1 = predict_f1.real_time_loss();
  double loss_f2 = predict_f2.real_time_loss();
  logger::logger_->info("Real time loss f1: {} ", loss_f1);
  logger::logger_->info("Real time loss f2: {}", loss_f2);
}

template <class flight_controller_t,
          class simulator_t>
void Iterative_learning<flight_controller_t, simulator_t>::
run()
{
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
    logger::logger_->info("Episode : {}", episode_);
		start_episode_ = swarm_.in_air(25);

    if (start_episode_) {
			time_steps_.reset();
      while (start_episode_) {
        if (time_steps_.steps() == 0)  {
          generate_trajectory_using_model(true, false);
          //Change each 10 times the direction of the leader
        } else if (time_steps_.steps() % 10 == 0) {
          generate_trajectory_using_model(true, false);
        } else if (sim_interface_->positions().at(0).z < 15
                   or sim_interface_->positions().at(1).z < 15) {
          generate_trajectory_using_model(false, true);
        } else {
          generate_trajectory_using_model(false, false);
        }

        std::vector<lt::position3D<double>> new_positions = sim_interface_->positions();
        logger::logger_->info("New positions : {}", new_positions);

        follower_1_->register_data_set();
        follower_2_->register_data_set();

        follower_1_->reset_all_states();
        follower_2_->reset_all_states();
        follower_1_->reset_all_actions();
        follower_2_->reset_all_actions();
        
        /*  Check the geometrical shape */
        std::vector<bool> shapes;
        for (auto it : quadrotors_) {
          shapes.push_back(it.examin_geometric_shape());
        }
        if (std::any_of(shapes.begin(), shapes.end(), [](const bool& shape){
                                                        if (!shape) return true;
                                                        else return false;
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
    step_errors_.clear();
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
		logger::logger_->info("Total count: {}",
			evaluate_model.output().at(7));

    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    logger::logger_->info("The quadcopters have been reset...");
    logger::logger_->info("Waiting untill the kalaman filter to reset...");
    std::this_thread::sleep_for(std::chrono::seconds(25));
    logger::logger_->info( "Kalaman filter reset...");
    logger::logger_->flush();
  }
}
