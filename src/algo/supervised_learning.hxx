#pragma once

# include "supervised_learning.hh"

template <class flight_controller_t,
	  class simulator_t>
Supervised_learning<flight_controller_t, simulator_t>::
Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
		    const std::vector<Quadrotor<simulator_t>>& quadrotors,
		    std::shared_ptr<simulator_t> gzs)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(gzs)),
   swarm_(std::move(iris_x)),
   quadrotors_(std::move(quadrotors))
{}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t,
		simulator_t>::
generate_trajectory_using_model(bool change_leader_action,
				bool stop_down_action)
{
    Actions action;
  /*  Allow easier access and debugging to all quadrotors state */
    leader_ = quadrotors_.begin();
    follower_1_ = std::next(quadrotors_.begin(), 1);
    follower_2_ = std::next(quadrotors_.begin(), 2);
    
  /* We need to pass State, nextState, and all possible action */
  std::vector<std::thread> threads;
  
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
      
  /*  Sample the state at time t */
  follower_1_->sample_state();
  follower_2_->sample_state();
  
  /* Follower action always equal to no move at this instant t */
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);
  
  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("l",
								     leader_->current_action(),
								     1000);
				}));
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f1",
								     leader_->current_action(),
								     1000);
				}));
				    
  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 
  /*  Sample the state at time t */
  follower_1_->sample_state();
  follower_2_->sample_state();
  
  Predictor predict("regression", "follower_1", quadrotors_);

  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = predict.create_absolute_features_matrix();
    
  /*  Predict the next state using the above data */
  auto matrix_best_action = predict.predict(features);
  Actions::Action predicted_follower_action;
  std::tie(std::ignore, std::ignore, predicted_follower_action) = matrix_best_action;
  
  follower_1_->current_action(predicted_follower_action);
  
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f1",
								     follower_1_->current_action(),
								     1000);  				   
  				}));
  
  for (auto& thread : threads) {
    thread.join();
  }

  /*  Get error of deformation to improve percision later and to
      verify the model accuracy */
    /*  Sample the state at time t */
  follower_1_->sample_state(); /*  Final state */
  follower_2_->sample_state();

  /* Take a tuple here  */
  double loss = predict.real_time_loss(matrix_best_action);
  LogInfo() << "Real time loss: " << loss;
  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, nextState.distances_3D()));
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  regression_ = settings.regression();

  /*  To be removed just check that every thing is fine */
  
  /*  Recover the initial state as an observer state */
  /*  This state will be used directly instead of original_dist */
  // Quadrotor::State<simulator_t> ObserverState(sim_interface_);
  // original_dist_ = ObserverState.distances_3D();
  // height_diff_ = ObserverState.height_difference();
  
  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads:
     * Arm + takeoff + offboard mode + moving + land
     * Then: repeat each episode.
     */
    LogInfo() << "Episode : " << episode_ ;

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

      count_ = 0;
      std::vector<lt::triangle<double>> new_triangle;

      while (count_ < 1000) {
	
	if (count_ == 0 ) {
	  generate_trajectory_using_model(true, false);
	  //Change each 10 times the direction of the leader
	} else if (count_ % 10 == 0) {
	  generate_trajectory_using_model(true, false);
	} else if (sim_interface_->positions().follower_1.z < 15
		   or sim_interface_->positions().follower_2.z < 15) {
	  generate_trajectory_using_model(false, true);
	} else {
	  generate_trajectory_using_model(false, false);
	}
	
	lt::positions<lt::position3D<double>> new_positions = sim_interface_->positions();

	LogInfo() << "New positions : " << new_positions;
		
	new_triangle.push_back(mtools_.triangle_side_3D(new_positions));
	
	if(regression_) {
	  follower_1_->register_data_set();
	  follower_2_->register_data_set();
	}
	states_.clear();
	action_follower_.clear();
	time_step_vector_.push_back(count_);
	
	/*  Need to verify that the controller is working,
	    use the triangle test to figure out after each iteration*/
	if (mtools_.is_triangle(mtools_.triangle_side_3D(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  data_set_.save_controller_count(count_);
	  break;
	}
	++count_;
      }
    }
    
    follower_1_->register_histogram(count_);
    follower_2_->register_histogram(count_);
    
    /*  Get the flight error as the mean of the step error */
    double mean_error = mtools_.mean(step_errors_);

    if (mean_error != -1) {
      /*  We need to figure out what to do with this value 
       and whether if it is useful or not*/
      // data_set_.save_error_file(mean_error);
      flight_errors_.push_back(mean_error);
    }
   
    step_errors_.clear();
    swarm_.land();
    
    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();
    
    LogInfo() << "The quadcopters have been reset...";
    LogInfo() << "Waiting untill the kalaman filter to reset...";    
    std::this_thread::sleep_for(std::chrono::seconds(25));
    LogInfo() << "Kalaman filter reset...";        
  }
}
