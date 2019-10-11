#pragma once

# include "trajectory_noise.hh"

template<class flight_controller_t,
	 class simulator_t>
TrajectoryNoise<flight_controller_t, simulator_t>::
TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   episode_(0),
   max_episode_(1),
   sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads))
{
  data_set_.init_dataset_directory();
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::
test_trajectory()
{
  Quadcopter robot;

  action_ = robot.random_action_generator();
  
  if (saved_action_ == Quadcopter::Action::backward) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::forward or
	   action_ == Quadcopter::Action::backward ) {
      action_ = robot.random_action_generator();
    }
  } else if (saved_action_ == Quadcopter::Action::down) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::up or
	   action_ == Quadcopter::Action::down) {
      action_ = robot.random_action_generator();
    }
  } else if (saved_action_ == Quadcopter::Action::up) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::down or
	   action_ == Quadcopter::Action::up) {
      action_ = robot.random_action_generator();
    }
  } else if (saved_action_ == Quadcopter::Action::forward) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::backward or
	   action_ == Quadcopter::Action::forward) {
      action_ = robot.random_action_generator();
    }
  } else if (saved_action_ == Quadcopter::Action::right) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::left or
	   action_ == Quadcopter::Action::right) {
      action_ = robot.random_action_generator();
    }
  } else if (saved_action_ == Quadcopter::Action::left) {
    action_ = robot.random_action_generator();
    while (action_ == Quadcopter::Action::right or
	   action_ == Quadcopter::Action::left) {
      action_ = robot.random_action_generator();
    }
  }
  /*  Execute a trajectory for 1 seconds */
  swarm_.one_quad_execute_trajectory("l" ,
				     action_,
				     1000);
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::run(/*  enter quadcopter number */)
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
    bool arm = swarm_.arm_specific_quadrotor("l");
    if(!arm)
      stop_episode_ = true;
    
    /*  Test with out this sleep, verify if landing state still the same or not*/
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff_specific_quadrotor(20, "l");
    if (!takeoff)
      stop_episode_ = true;
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed_specific_quadrotor("l");

     /*  Switch to offboard mode, Allow the control */
    /* Stop the episode is the one quadcopter have fallen to set
       offbaord mode */    
    bool offboard_mode = swarm_.start_offboard_mode_specific_quadrotor("l");
    if(!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode_) {
      
      count_ = 0;      
      while (count_  < 1000 ) {
	lt::positions<lt::position3D<double>> positions_before_action =
	  sim_interface_->positions();
	
	test_trajectory();
	/* Get the actual position, test if the triangle is OK */
	lt::positions<lt::position3D<double>> positions_after_action =
	  sim_interface_->positions();
		
	lt::dist3D<double> quadrotor_distance = mtools_.traveled_distances(positions_before_action,
									   positions_after_action);
	
	LogInfo() << "Traveled Distance by the leader : " << quadrotor_distance.d1;
	if (count_ > 0) {

	  /* Construct the experinces vector, Study the distribution
	     law for the each action over time. This is essential as
	     the distance traveled by each action are not the same for
	     the same amount of time */

	  /*  ADD PAST A_t-1 distances */
	  
	  if (action_ == Quadcopter::Action::forward and
	      saved_action_ == Quadcopter::Action::left) {
	    LogInfo() << "F + L";
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_l_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::forward and
		     saved_action_ == Quadcopter::Action::right) {
	    LogInfo() << "F + R";
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_r_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_r_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::forward and
		     saved_action_ == Quadcopter::Action::up) {
	    LogInfo() << "F + U";
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    f_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::forward and 
	    	     saved_action_ == Quadcopter::Action::down) {
	    LogInfo() << "F + D";
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    f_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::backward and
		     saved_action_ == Quadcopter::Action::left) {
	    LogInfo() << "B + L";
	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_l_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
	    	     saved_action_ == Quadcopter::Action::right) {
	    LogInfo() << "B + R";
	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_r_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_r_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
		     saved_action_ == Quadcopter::Action::up) {
	    LogInfo() << "B + U";
	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
	    	     saved_action_ == Quadcopter::Action::down) {
	    LogInfo() << "B + D";
	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::forward) {
	    LogInfo() << "L + F";
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::backward) {
	    LogInfo() << "L + B";
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_b_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::up) {
	    LogInfo() << "L + U";
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::down) {
	    LogInfo() << "L + D";
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);	  
	  
	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::forward) {
	    LogInfo() << "R + F";
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::backward) {
	    LogInfo() << "R + B";
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_b_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::up) {
	    LogInfo() << "R + U";
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::down) {
	    LogInfo() << "R + D";
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::up and
		     saved_action_ == Quadcopter::Action::forward) {

	    up_action_vec_.push_back(quadrotor_distance.d1);
	    u_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    u_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::up and
		     saved_action_ == Quadcopter::Action::backward) {
	    
	    up_action_vec_.push_back(quadrotor_distance.d1);
	    u_k_b_action_vec_.push_back(quadrotor_distance.d1);  
	    u_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::up and
		     saved_action_ == Quadcopter::Action::left) {

	    up_action_vec_.push_back(quadrotor_distance.d1);
	    u_k_l_action_vec_.push_back(quadrotor_distance.d1);	    
	    u_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::up and
		     saved_action_ == Quadcopter::Action::right) {

	    up_action_vec_.push_back(quadrotor_distance.d1);
	    u_k_r_action_vec_.push_back(quadrotor_distance.d1);	    
	    u_k_r_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    	    
	  } else if (action_ == Quadcopter::Action::down and
		     saved_action_ == Quadcopter::Action::forward) {
	    down_action_vec_.push_back(quadrotor_distance.d1);
	    d_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    d_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::down and
		     saved_action_ == Quadcopter::Action::backward) {
	    down_action_vec_.push_back(quadrotor_distance.d1);
	    d_k_b_action_vec_.push_back(quadrotor_distance.d1);	    
	    d_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::down and
		     saved_action_ == Quadcopter::Action::left) {
	    down_action_vec_.push_back(quadrotor_distance.d1);
	    d_k_l_action_vec_.push_back(quadrotor_distance.d1);	    
	    d_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::down and
		     saved_action_ == Quadcopter::Action::right) {
	    down_action_vec_.push_back(quadrotor_distance.d1);
	    d_k_l_action_vec_.push_back(quadrotor_distance.d1);	    
	    d_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);
	  }	  
	}
	saved_action_ = action_;
	saved_quadrotor_distance_ = quadrotor_distance;
	
	++count_ ;
      }
      /*  Calculate the mean value */
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
    
    /*  Logging */
    
    /*  Landing */
    swarm_.land_specific_quadrotor("l");
    
    /*  Resetting */
    sim_interface_->reset_models();

    LogInfo() << "All quadrotors have been reset...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}


  
