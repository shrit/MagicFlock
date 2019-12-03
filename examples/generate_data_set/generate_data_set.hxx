#pragma once

# include "generate_data_set.hh"

template<class flight_controller_t,
	 class simulator_t>
Generator<flight_controller_t, simulator_t>::
Generator(std::vector<std::shared_ptr<flight_controller_t>> quads,
	  const std::vector<Quadrotor<simulator_t>>& quadrotors,
	  std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads)),
   quadrotors_(std::move(quadrotors))
{
  /*  Allow easier access and debugging to all quadrotors state */
  leader_ = quadrotors_.begin();
  follower_1_ = std::next(quadrotors_.begin(), 1);
  follower_2_ = std::next(quadrotors_.begin(), 2);
}

/*  Phase one: Data Set generation */
template <class flight_controller_t,
	  class simulator_t>
void Generator<flight_controller_t, simulator_t>::
generate_trajectory(bool change_leader_action)
{
  Actions action;  
  std::vector<std::thread> threads;
  
  /*  Sample the state at time t */
  follower_1_->sample_state();
  follower_2_->sample_state();
  LogInfo() << "Sampling states at t";
  /*  Create a random action for the leader_s, with the opposed condition */
  if (change_leader_action == true) {
    leader_->current_action(
			   action.random_action_generator_with_only_opposed_condition
			   (leader_->last_action()));
  } else {
    leader_->current_action(leader_->last_action());
  }
    
  /*  Do not allow follower to move, at this time step, 
      block the follower and log not move*/
  follower_1_->current_action(Actions::Action::NoMove);
  follower_2_->current_action(Actions::Action::NoMove);
  
  LogInfo() << "Current action leader" << action.action_to_str(leader_->current_action());
  /*  Threading Quadrotors */
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory(leader_->id(),
								     leader_->current_action(),
								     1,
								     1000);
				}));

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  /* Get the next state at time t + 1  */
  LogInfo() << "Sampling states at t +1";
  follower_1_->sample_state();
  follower_2_->sample_state();

  /*  Do a random action at t+1 for the follower 1 */
  follower_1_->current_action(
			      action.random_action_generator_with_only_opposed_condition
			      (follower_1_->last_action()));
  
  LogInfo() << "Current action follower 1" << action.action_to_str(follower_1_->current_action());
  
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory(follower_1_->id(),
								     follower_1_->current_action(),
								     1,
								     1000);
				}));
  
  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  /* Get the next state at time t + 2  */
  LogInfo() << "Sampling states at t+2";
  follower_1_->sample_state();
  follower_2_->sample_state();
  
  /*  Do a random action at t+2 for the follower 2 */
  follower_2_->current_action(
			    action.random_action_generator_with_only_opposed_condition
			    (follower_2_->last_action()));
  LogInfo() << "Current action follower 2" << action.action_to_str(follower_2_->current_action());
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory(follower_2_->id(),
								     follower_2_->current_action(),
								     1,
								     1000);
				}));
  for(auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the Quadrotors finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  
  /* Get the next state at time t + 3 */
  LogInfo() << "Sampling states at t +3";
  follower_1_->sample_state();
  follower_2_->sample_state();
}

template<class flight_controller_t,
	 class simulator_t>
void Generator<flight_controller_t, simulator_t>::
run()
{
  std::vector<lt::position3D<double>> original_positions = sim_interface_->positions();
  LogInfo() << "Starting positions : " << original_positions;

  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase: an episode start when the quadrotors
     * takeoff and end when they land and reset. Each episode contain
     * a count number (steps). Each count represents one line in the
     * dataset. Several trajectories might be executed in one count.
     * Now. We execute only two trajectories in one count, thus means
     * dist(t-1), a(t-1), dist(t), a(t). Where dist contain distance
     * between the (TF and TL) and (TF and FF)
     */    
    LogInfo() << "Episode : " << episode_ ;

    /* Stop the episode if one of the quad has fallen to arm */    
    stop_episode_ = false;
    bool arm = swarm_.arm();
    if(!arm)
      stop_episode_ = true;
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(15);
    if (!takeoff)
      stop_episode_ = true;
    
    /*  Setting up speed in order to switch the mode */
    swarm_.init_speed();
    
    /*  Switch to offboard mode, Allow the control */
    /* Stop the episode is the one quadcopter have fallen to set
       offbaord mode */    
    bool offboard_mode = swarm_.start_offboard_mode();
    if(!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));
    
    /*  Start the First phase */
    /*  Collect dataset by creating a set of trajectories, each time
	the leader_ and the follower execute their trajectory randomly,
	we check the triangle (whether the follower is too close or
	too far) finally we break the loop after 10 trajectorise of 1
	second each */
    
    if (!stop_episode_) {
      /*  Verify that vectors are clear when starting new episode */
      follower_1_->reset_all_states();
      follower_2_->reset_all_states();
      follower_1_->reset_all_actions();
      follower_2_->reset_all_actions();
      count_ = 0;
      
      while (!stop_episode_) {
	
	std::vector<lt::position3D<double>> positions_before_action =
	  sim_interface_->positions();       
	
	/* Choose one random trajectory for the leader_ in the first
	   count. Then, keep the same action until the end of the
	   episode */	
	if (count_ == 0) {
	  generate_trajectory(true);
	} else {
	  generate_trajectory(false);
	}
	
	std::vector<lt::position3D<double>> positions_after_action =
	  sim_interface_->positions();
		
	follower_1_->register_data_set();
	follower_2_->register_data_set();
	
	
	/*  Clear vectors after each generated line in the dataset */
	follower_1_->reset_all_states();
	follower_2_->reset_all_states();	
	follower_1_->reset_all_actions();
	follower_2_->reset_all_actions();
	
	/*  Chech the geometrical shape */
	bool shape = false;
	for (auto it : quadrotors_) {
	  shape = it.examin_geometric_shape();
	}
	if (!shape) {
	  LogInfo() << "The geometrical is no longer conserved";
	  break;
	}	
	++count_;					
      }
    }
    
    /*  Save a version of the time steps to create a histogram */
    follower_1_->register_histogram(count_);
    follower_2_->register_histogram(count_);
    
    /* Landing is blocking untill touching the ground*/
    swarm_.land();
    
    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();

    LogInfo() << "All quadrotors have been reset...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));

    /*BIAS accelerometer problem after resetting the models*/

    /*  The only possible solution was to change the upper limit value
     * for the bias inside thee code of the firmware directly. The
     * solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they propose
     * to increase the value of COM_ARM_EKF_AB. Note that, the default
     * value is 0.00024 I have increased it to 0.00054 which is very
     * high to the usual stadard. Otherwise there is no way to do the
     * simulation. Remember, the reboot() function in the MAVSDK
     * action class is not implemented at the time of writing this
     * comment, and maybe it will never be implemented as it is quite
     * complicated to reboot the px4 software from the simulator. I
     * understand this choice, we need to leave a big sleep_for after
     * resetting the quadcopters, that is going to helpe resetting the
     * accelerometer values without any problems!
     */

    /*  I have quite tested a lot of different solution. Frankly,if I am going
     * to find a better one, I will replace it directly. */
  }
}
