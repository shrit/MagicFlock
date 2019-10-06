#pragma once

# include "trajectory_noise.hh"

template<class flight_controller_t,
	 class simulator_t>
TrajectoryNoise<flight_controller_t, simulator_t>::
TrajectoryNoise(std::vector<std::shared_ptr<flight_controller_t>> quads,
		std::shared_ptr<simulator_t> sim_interface)
  :sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads))
{
}

template<class flight_controller_t,
	 class simulator_t>
void TrajectoryNoise<flight_controller_t, simulator_t>::
test_trajectory(bool random_action)
{

  /*  Choose a quacopter  */

  Quadcopter::Action action;
  Quadcopter robot;

  if (random_action == true) {
    
    action = robot.random_action_generator();
    saved_action_ = action;
    
  } else {
    action = saved_action_;
  }

  /*  Execute a trajectory for 1 seconds */
  swarm_.one_quad_execute_trajectory("l" ,
				     action,
				     1000);  
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
    bool arm = swarm_.arm();
    if(!arm)
      stop_episode_ = true;
    
    /*  Test with out this sleep */
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(5);
    if (!takeoff)
      stop_episode_ = true;
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed();

     /*  Switch to offboard mode, Allow the control */
    /* Stop the episode is the one quadcopter have fallen to set
       offbaord mode */    
    bool offboard_mode = swarm_.start_offboard_mode();
    if(!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode_) {
            
      lt::positions<lt::position3D<double>> positions_before_action =
	sim_interface_->positions();
      
      LogInfo() << "Positions before action : " << positions_before_action;
      
      if (episode_ % 2) {
	test_trajectory(true);
      } else {
	test_trajectory(false);
      }
      
      /* Get the actual position, test if the triangle is OK */
      lt::positions<lt::position3D<double>> positions_after_action = sim_interface_->positions();
      
      LogInfo() << "Positions After action : " << positions_after_action;
      
      LogInfo() << "Travelled Distance : " <<
	mtools_.travelled_distances(positions_before_action,
				    positions_after_action); 
      /*  Calculate the bias */
      /*  Each time the trajectory is repeated calculate the difference in the coordination */
      /*  These differences are random variable extracted from the above experince */
      /*  Calculate the mean value and variance of the above random variable */          

    }
    /*  Logging */

    /*  Landing */
    swarm_.land();
    
    /*  Resetting */
    sim_interface_->reset_models();

    LogInfo() << "The quadcopters have been resetted...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  
  }
}

  
