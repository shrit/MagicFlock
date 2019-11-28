#pragma once

# include "trajectory_noise.hh"

template<class flight_controller_t,
	 class simulator_t>
TrajectoryNoise<flight_controller_t, simulator_t>::
TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		const std::vector<Quadrotor<simulator_t>>& quadrotors,
		std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   episode_(0),
   max_episode_(1),
   sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads)),
   quadrotors_(std::move(quadrotors))
{
  data_set_.init_dataset_directory();
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::
test_trajectory(bool stop_down_action)
{
  follower_1_->current_action(
			      action_.random_action_generator_with_all_conditions
			      (follower_1_->last_action()));
  if (stop_down_action == true) {
    while (follower_1_->current_action() == Actions::Action::down) {
      follower_1_->current_action(
				  action_.random_action_generator_with_all_conditions
				  (follower_1_->last_action()));
    }
  }  
  /*  Execute a trajectory for 1 seconds */
  swarm_.one_quad_execute_trajectory(follower_1_->id(),
				     follower_1_->current_action(),
				     1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::run()
{
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
    bool arm = swarm_.arm_specific_quadrotor(follower_1_->id());
    if(!arm)
      stop_episode_ = true;
    
    /*  Test with out this sleep, verify if landing state still the same or not*/
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff_specific_quadrotor(20, follower_1_->id());
    if (!takeoff)
      stop_episode_ = true;
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed_specific_quadrotor(follower_1_->id());

     /*  Switch to offboard mode, Allow the control */
    /* Stop the episode is the one quadcopter have fallen to set
       offbaord mode */    
    bool offboard_mode = swarm_.start_offboard_mode_specific_quadrotor(follower_1_->id());
    if(!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode_) {
      
      count_ = 0;      
      while (count_  < 1000 ) {
	std::vector<lt::position3D<double>> positions_before_action =
	  sim_interface_->positions();
	/*  Verify that the quadrotors is not close to the ground */
	if (sim_interface_->positions().at(1).z < 15) {
	  test_trajectory(true);
	} else  {
	  test_trajectory(false);
	}
	/* Get the actual position, test if the triangle is OK */
	std::vector<lt::position3D<double>> positions_after_action =
	  sim_interface_->positions();

	/*  Take the distance traveled by the leader  */
	/*  Just a hack need to find a better solution later */
        double traveled_distance = mtools_.traveled_distances(positions_before_action.at(0),
							      positions_after_action.at(0));
	
	LogInfo() << "Traveled Distance by the quadrotor : " << traveled_distance;
	if (count_ > 0) {

	  /* Construct the experinces vector, Study the distribution
	     law for the each action over time. This is essential as
	     the distance traveled by each action are not the same for
	     the same amount of time */

	  /*  ADD PAST A_t-1 distances */
	  
	  if (follower_1_->current_action() == Actions::Action::forward and
	      follower_1_->last_action() == Actions::Action::left) {
	    LogInfo() << "F + L";
	    forward_action_vec_.push_back(traveled_distance);
	    f_k_l_action_vec_.push_back(traveled_distance);
	    
	  } else if (follower_1_->current_action() == Actions::Action::forward and
		     follower_1_->last_action() == Actions::Action::right) {
	    LogInfo() << "F + R";
	    forward_action_vec_.push_back(traveled_distance);
	    f_k_r_action_vec_.push_back(traveled_distance);	   
	    
	  } else if (follower_1_->current_action() == Actions::Action::forward and
		     follower_1_->last_action() == Actions::Action::up) {
	    LogInfo() << "F + U";
	    forward_action_vec_.push_back(traveled_distance);
	    f_k_u_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::forward and 
	    	     follower_1_->last_action() == Actions::Action::down) {
	    LogInfo() << "F + D";
	    forward_action_vec_.push_back(traveled_distance);
	    f_k_d_action_vec_.push_back(traveled_distance);
	    
	  } else if (follower_1_->current_action() == Actions::Action::backward and
		     follower_1_->last_action() == Actions::Action::left) {
	    LogInfo() << "B + L";
	    backward_action_vec_.push_back(traveled_distance);
	    b_k_l_action_vec_.push_back(traveled_distance);

	  } else if (follower_1_->current_action() == Actions::Action::backward and
	    	     follower_1_->last_action() == Actions::Action::right) {
	    LogInfo() << "B + R";
	    backward_action_vec_.push_back(traveled_distance);
	    b_k_r_action_vec_.push_back(traveled_distance);

	  } else if (follower_1_->current_action() == Actions::Action::backward and
		     follower_1_->last_action() == Actions::Action::up) {
	    LogInfo() << "B + U";
	    backward_action_vec_.push_back(traveled_distance);
	    b_k_u_action_vec_.push_back(traveled_distance);

	  } else if (follower_1_->current_action() == Actions::Action::backward and
	    	     follower_1_->last_action() == Actions::Action::down) {
	    LogInfo() << "B + D";
	    backward_action_vec_.push_back(traveled_distance);
	    b_k_d_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::left and
		     follower_1_->last_action() == Actions::Action::forward) {
	    LogInfo() << "L + F";
	    left_action_vec_.push_back(traveled_distance);
	    l_k_f_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::left and
		     follower_1_->last_action() == Actions::Action::backward) {
	    LogInfo() << "L + B";
	    left_action_vec_.push_back(traveled_distance);
	    l_k_b_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::left and
		     follower_1_->last_action() == Actions::Action::up) {
	    LogInfo() << "L + U";
	    left_action_vec_.push_back(traveled_distance);
	    l_k_u_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::left and
		     follower_1_->last_action() == Actions::Action::down) {
	    LogInfo() << "L + D";
	    left_action_vec_.push_back(traveled_distance);
	    l_k_d_action_vec_.push_back(traveled_distance);	    
	  
	  } else if (follower_1_->current_action() == Actions::Action::right and
		     follower_1_->last_action() == Actions::Action::forward) {
	    LogInfo() << "R + F";
	    right_action_vec_.push_back(traveled_distance);
	    r_k_f_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::right and
		     follower_1_->last_action() == Actions::Action::backward) {
	    LogInfo() << "R + B";
	    right_action_vec_.push_back(traveled_distance);
	    r_k_b_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::right and
		     follower_1_->last_action() == Actions::Action::up) {
	    LogInfo() << "R + U";
	    right_action_vec_.push_back(traveled_distance);
	    r_k_u_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::right and
		     follower_1_->last_action() == Actions::Action::down) {
	    LogInfo() << "R + D";
	    right_action_vec_.push_back(traveled_distance);
	    r_k_d_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::up and
		     follower_1_->last_action() == Actions::Action::forward) {

	    up_action_vec_.push_back(traveled_distance);
	    u_k_f_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::up and
		     follower_1_->last_action() == Actions::Action::backward) {
	    
	    up_action_vec_.push_back(traveled_distance);
	    u_k_b_action_vec_.push_back(traveled_distance);  
	    
	  } else if (follower_1_->current_action() == Actions::Action::up and
		     follower_1_->last_action() == Actions::Action::left) {

	    up_action_vec_.push_back(traveled_distance);
	    u_k_l_action_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::up and
		     follower_1_->last_action() == Actions::Action::right) {

	    up_action_vec_.push_back(traveled_distance);
	    u_k_r_action_.push_back(traveled_distance);	    
	    	    
	  } else if (follower_1_->current_action() == Actions::Action::down and
		     follower_1_->last_action() == Actions::Action::forward) {
	    down_action_vec_.push_back(traveled_distance);
	    d_k_f_action_1_vec_.push_back(traveled_distance);	    

	  } else if (follower_1_->current_action() == Actions::Action::down and
		     follower_1_->last_action() == Actions::Action::backward) {
	    down_action_vec_.push_back(traveled_distance);
	    d_k_b_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::down and
		     follower_1_->last_action() == Actions::Action::left) {
	    down_action_vec_.push_back(traveled_distance);
	    d_k_l_action_vec_.push_back(traveled_distance);	    
	    
	  } else if (follower_1_->current_action() == Actions::Action::down and
		     follower_1_->last_action() == Actions::Action::right) {
	    down_action_->current_action()vec_.push_back(traveled_distance);
	    d_k_r_action_->current_action()vec_.push_back(traveled_distance);	    
	  }	  
	}
	// follower_1->last_action(follower_1_->current_action()); //no longer needed
	++count_;
      }
      /*  Calculate the mean value */ /*  Replace logging with a file */
      LogInfo() << "Mean of Forward: " <<  mtools_.mean(forward_action_vec_);
      LogInfo() << "Mean of Backward: " <<  mtools_.mean(backward_action_vec_);
      LogInfo() << "Mean of Left: " <<  mtools_.mean(left_action_vec_);
      LogInfo() << "Mean of Right: " <<  mtools_.mean(right_action_vec_);
      LogInfo() << "Mean of up: " <<  mtools_.mean(up_action_vec_);
      LogInfo() << "Mean of down: " <<  mtools_.mean(down_action_vec_);
      
      LogInfo() << "Mean of Forward knowing left: " <<  mtools_.mean(f_k_l_action_vec_);
      LogInfo() << "Mean of Forward knowing right: " <<  mtools_.mean(f_k_r_action_vec_);
      LogInfo() << "Mean of Forward knowing up: " <<  mtools_.mean(f_k_u_action_vec_);
      LogInfo() << "Mean of Forward knowing down: " <<  mtools_.mean(f_k_d_action_vec_);

      LogInfo() << "Mean of backward knowing left: " <<  mtools_.mean(b_k_l_action_vec_);
      LogInfo() << "Mean of backward knowing right: " <<  mtools_.mean(b_k_r_action_vec_);
      LogInfo() << "Mean of backward knowing up: " <<  mtools_.mean(b_k_u_action_vec_);
      LogInfo() << "Mean of backward knowing down: " <<  mtools_.mean(b_k_d_action_vec_);

      LogInfo() << "Mean of left knowing forward: " <<  mtools_.mean(l_k_f_action_vec_);
      LogInfo() << "Mean of left knowing backward: " <<  mtools_.mean(l_k_b_action_vec_);
      LogInfo() << "Mean of left knowing up: " <<  mtools_.mean(l_k_u_action_vec_);
      LogInfo() << "Mean of left knowing down: " <<  mtools_.mean(l_k_d_action_vec_);

      LogInfo() << "Variance of Forward: " <<  mtools_.variance(forward_action_vec_);
      LogInfo() << "Variance of Backward: " <<  mtools_.variance(backward_action_vec_);
      LogInfo() << "Variance of Left: " <<  mtools_.variance(left_action_vec_);
      LogInfo() << "Variance of Right: " <<  mtools_.variance(right_action_vec_);
      LogInfo() << "Variance of up: " <<  mtools_.variance(up_action_vec_);
      LogInfo() << "Variance of down: " <<  mtools_.variance(down_action_vec_);

      LogInfo() << "Variance of Forward knowing left: " <<  mtools_.variance(f_k_l_action_vec_);
      LogInfo() << "Variance of Forward knowing right: " <<  mtools_.variance(f_k_r_action_vec_);
      LogInfo() << "Variance of Forward knowing up: " <<  mtools_.variance(f_k_u_action_vec_);
      LogInfo() << "Variance of Forward knowing down: " <<  mtools_.variance(f_k_d_action_vec_);

      LogInfo() << "Variance of backward knowing left: " <<  mtools_.variance(b_k_l_action_vec_);
      LogInfo() << "Variance of backward knowing right: " <<  mtools_.variance(b_k_r_action_vec_);
      LogInfo() << "Variance of backward knowing up: " <<  mtools_.variance(b_k_u_action_vec_);
      LogInfo() << "Variance of backward knowing down: " <<  mtools_.variance(b_k_d_action_vec_);

      LogInfo() << "Variance of left knowing forward: " <<  mtools_.variance(l_k_f_action_vec_);
      LogInfo() << "Variance of left knowing backward: " <<  mtools_.variance(l_k_b_action_vec_);
      LogInfo() << "Variance of left knowing up: " <<  mtools_.variance(l_k_u_action_vec_);
      LogInfo() << "Variance of left knowing down: " <<  mtools_.variance(l_k_d_action_vec_);
      
    }   
    
    /*  Landing */
    swarm_.land_specific_quadrotor(leader_->id());
    
    /*  Resetting */
    sim_interface_->reset_models();
    LogInfo() << "All quadrotors have been reset...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}


  
