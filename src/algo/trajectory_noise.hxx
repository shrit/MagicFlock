#pragma once

# include "trajectory_noise.hh"

template<class flight_controller_t,
	 class simulator_t>
TrajectoryNoise<flight_controller_t, simulator_t>::
TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads))
{
  data_set_.init_dataset_directory();
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::
test_trajectory(bool random_action)
{
  Quadcopter robot;

  if (random_action == true) {   
    action_ = robot.random_action_generator();

    if (action_ == saved_action_) {
      action_ = robot.random_action_generator();
      if (action_ == Quadcopter::Action::forward and
	  saved_action_ == Quadcopter::Action::backward)
	action_ = robot.random_action_generator();
    } else if (action_ == Quadcopter::Action::up and
	  saved_action_ == Quadcopter::Action::down) {
      action_ = robot.random_action_generator();
    } else if (action_ == Quadcopter::Action::down and
	       saved_action_ == Quadcopter::Action::up) {
      action_ = robot.random_action_generator();
    } else if (action_ == Quadcopter::Action::backward and
	       saved_action_ == Quadcopter::Action::forward) {
      action_ = robot.random_action_generator();
    } else if (action_ == Quadcopter::Action::left and
	       saved_action_ == Quadcopter::Action::right) {
      action_ = robot.random_action_generator();
      } else if (action_ == Quadcopter::Action::right and
	       saved_action_ == Quadcopter::Action::left) {
      action_ = robot.random_action_generator();
    }    
    saved_action_ = action_;
        
  } else {
    action_ = saved_action_;
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
    bool takeoff = swarm_.takeoff_specific_quadrotor(5, "l");
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
      while (count_  < 300 ) {
	lt::positions<lt::position3D<double>> positions_before_action =
	  sim_interface_->positions();
	
	test_trajectory(true);
	/* Get the actual position, test if the triangle is OK */
	lt::positions<lt::position3D<double>> positions_after_action =
	  sim_interface_->positions();
	
	
	lt::dist3D<double> quadrotor_distance = mtools_.traveled_distances(positions_before_action,
									   positions_after_action);
	
	LogInfo() << "Travelled Distance by the leader : " << quadrotor_distance.d1;
	if (count_ == 1) {

	  /* Construct the experinces vector, Study the distribution
	     law for the each action over time. This is essential as
	     the distance traveled by each action are not the same for
	     the same amount of time */

	  /*  ADD PAST A_t-1 distances */
	  
	  if (action_ == Quadcopter::Action::forward and
	      saved_action_ == Quadcopter::Action::left) {

	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_l_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::forward and
		     saved_action_ == Quadcopter::Action::right) {
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_r_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_r_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::forward and
		     saved_action_ == Quadcopter::Action::up) {
	    
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    f_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::forward and 
	    	     saved_action_ == Quadcopter::Action::down) {
	    
	    forward_action_vec_.push_back(quadrotor_distance.d1);
	    f_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    f_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::backward and
		     saved_action_ == Quadcopter::Action::left) {

	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_l_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_l_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
	    	     saved_action_ == Quadcopter::Action::right) {

	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_r_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_r_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
		     saved_action_ == Quadcopter::Action::up) {
	    
	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::backward and
	    	     saved_action_ == Quadcopter::Action::down) {

	    backward_action_vec_.push_back(quadrotor_distance.d1);
	    b_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    b_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::forward) {

	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::backward) {
	    
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_b_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::up) {
	    
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::left and
		     saved_action_ == Quadcopter::Action::down) {
	    
	    left_action_vec_.push_back(quadrotor_distance.d1);
	    l_k_d_action_vec_.push_back(quadrotor_distance.d1);	    
	    l_k_d_action_vec_.push_back(saved_quadrotor_distance_.d1);	  
	  
	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::forward) {
	    
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_f_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_f_action_vec_.push_back(saved_quadrotor_distance_.d1);
	    
	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::backward) {
	    
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_b_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_b_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::up) {
	    
	    right_action_vec_.push_back(quadrotor_distance.d1);
	    r_k_u_action_vec_.push_back(quadrotor_distance.d1);	    
	    r_k_u_action_vec_.push_back(saved_quadrotor_distance_.d1);

	  } else if (action_ == Quadcopter::Action::right and
		     saved_action_ == Quadcopter::Action::down) {
	    
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
      
      LogInfo() << "Mean of Forward knowing left" <<  mtools_.mean(f_k_l_action_vec_);
      LogInfo() << "Mean of Forward knowing right" <<  mtools_.mean(f_k_r_action_vec_);
      LogInfo() << "Mean of Forward knowing up" <<  mtools_.mean(f_k_u_action_vec_);
      LogInfo() << "Mean of Forward knowing down" <<  mtools_.mean(f_k_d_action_vec_);

      LogInfo() << "Mean of backward knowing left" <<  mtools_.mean(b_k_l_action_vec_);
      LogInfo() << "Mean of backward knowing right" <<  mtools_.mean(b_k_r_action_vec_);
      LogInfo() << "Mean of backward knowing up" <<  mtools_.mean(b_k_u_action_vec_);
      LogInfo() << "Mean of backward knowing down" <<  mtools_.mean(b_k_d_action_vec_);

      LogInfo() << "Mean of left knowing forward" <<  mtools_.mean(l_k_f_action_vec_);
      LogInfo() << "Mean of left knowing backward" <<  mtools_.mean(l_k_b_action_vec_);
      LogInfo() << "Mean of left knowing up" <<  mtools_.mean(l_k_u_action_vec_);
      LogInfo() << "Mean of left knowing down" <<  mtools_.mean(l_k_d_action_vec_);

      

      
      
      /*  Calculate the mean value and variance of the above random variable */
      
      
      
    }
    
    /*  Logging */
    
    /*  Landing */
    swarm_.land_specific_quadrotor("l");
    
    /*  Resetting */
    sim_interface_->reset_models();

    LogInfo() << "The quadcopters have been resetted...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}


  
