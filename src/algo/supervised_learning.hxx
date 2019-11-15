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
{
  data_set_.init_dataset_directory();
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t,
		simulator_t>::
generate_trajectory_using_model(bool change_leader_action,
				bool stop_down_action)
{
    Actions action;
  /*  Allow easier access and debugging to all quadrotors state */
  auto leader = quadrotors_.begin();
  auto follower_1_ = std::next(quadrotors_.begin(), 1);
  auto follower_2_ = std::next(quadrotors_.begin(), 2);
  
  /* We need to pass State, nextState, and all possible action */
  std::vector<std::thread> threads;
  
  if (change_leader_action == true) {
    leader_.current_action(
			   action.random_action_generator_with_only_opposed_condition
			   (leader_.last_action()));
  } else if (stop_down_action == true) {
    while (leader_.current_action() == Actions::Action::down) {
      leader_.current_action(
			     action.random_action_generator_with_only_opposed_condition
			     (leader_.last_action()));
    }    
  }
      
  /*  Sample the state at time t */
  follower_1_.sample_state();
  follower_2_.sample_state();
  
  /* Follower action always equal to no move at this instant t */
  follower_1_.current_action(Actions::Action::NoMove);
  follower_2_.current_action(Actions::Action::NoMove);
  
  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("l",
								     leader_.current_action(),
								     1000);
				}));
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f1",
								     leader_.current_action(),
								     1000);
				}));
				    
  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
 
  /*  Sample the state at time t */
  follower_1_.sample_state();
  follower_2_.sample_state();
  
  Predictor predict("regression");

  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = predict.create_absolute_features_matrix(states_, action_follower_.back());
    
  /*  Predict the next state using the above data */
  auto matrix_best_action = predict.predict(features);
  Actions::Action predicted_follower_action;
  std::tie(std::ignore, std::ignore, predicted_follower_action) = matrix_best_action;
  action_follower_.push_back(predicted_follower_action);
  
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f2",
								     action_follower_.back(),								          1000);  				   
  				}));
  
  for (auto& thread : threads) {
    thread.join();
  }

  /*  Get error of deformation to improve percision later and to
      verify the model accuracy */
    /*  Sample the state at time t */
  follower_1_.sample_state(); /*  Final state */
  follower_2_.sample_state();

  /* Take a tuple here  */
  double loss = predict.real_time_loss(states_,
				       matrix_best_action);
  LogInfo() << "Real time loss: " << loss;
  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, nextState.distances_3D()));
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  robot_.init();
  classification_ = settings.classification();
  regression_ = settings.regression();
  /*  Recover the initial state as an observer state */
  /*  This state will be used directly instead of original_dist */
  Quadrotor::State<simulator_t> ObserverState(sim_interface_);
  original_dist_ = ObserverState.distances_3D();
  height_diff_ = ObserverState.height_difference();
  
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
			
  	/*  Handle fake takeoff.. */
   if (sim_interface_->positions().f1.z < 8
       or sim_interface_->positions().f2.z < 8) {
      stop_episode_ = true;      
   }
			      
    if (!stop_episode_) {

      count_ = 0;
      std::vector<lt::triangle<double>> new_triangle;

      while (count_ < 500) {
	/*  Do online learning... */
        Rewards::Reward reward =
	  Rewards::Reward::Unknown;

	if (count_ == 0 ) {
	  generate_trajectory_using_model(true, false);
	  //Change each 10 times the direction of the leader
	} else if (count_ % 10 == 0) {
	  generate_trajectory_using_model(true, false);
	} else if (sim_interface_->positions().f1.z < 15
		   or sim_interface_->positions().f2.z < 15) {
	  generate_trajectory_using_model(false, true);
	} else {
	  generate_trajectory_using_model(false, false);
	}
	
	lt::positions<lt::position3D<double>> new_positions = sim_interface_->positions();

	LogInfo() << "New positions : " << new_positions;
		
	new_triangle.push_back(mtools_.triangle_side_3D(new_positions));
	
	if (classification_) {
	  if (count_ == 0 ) {
	    reward = robot_.action_evaluator(original_dist_,
					     new_triangle.at(count_));
	    
	  } else {
	    reward = robot_.action_evaluator(new_triangle.at(count_ -1),
					     new_triangle.at(count_));
	  }
	}       	     
	/* Log online dataset */	
	if (classification_) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 7),
				      states_.back(),
				      mtools_.to_one_hot_encoding(reward, 4)
				      );
	}
	
	if (regression_) {
	  
	  data_set_.save_csv_data_set(states_.front(),       
				      mtools_.to_one_hot_encoding(action_follower_.front(), 7),
				      *(it),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 7),
				      states_.back()
				      );	  
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
    /*  Add 1 to adjust count to one instead of starting by zero */
    count_ = count_ + 1;
    /*  Save a version of the time steps to create a histogram */
    mtools_.histogram(count_);
    data_set_.save_histogram(mtools_.get_histogram<int>());

    /*  Get the flight error as the mean of the step error */
    double mean_error = mtools_.mean(step_errors_);

    if (mean_error != -1) {
      data_set_.save_error_file(mean_error);
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
