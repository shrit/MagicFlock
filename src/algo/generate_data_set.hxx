#pragma once

# include "generate_data_set.hh"

template<class flight_controller_t,
	 class simulator_t>
Generator<flight_controller_t, simulator_t>::
Generator (std::vector<std::shared_ptr<flight_controller_t>> quads,
	   std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(sim_interface)),
   swarm_(std::move(quads))
{
  data_set_.init_dataset_directory();
}

/*  Phase one: Data Set generation */
template <class flight_controller_t,
	  class simulator_t>
void Generator<flight_controller_t, simulator_t>::
generate_trajectory(bool random_leader_action)
{
  Quadcopter::Action action_leader;
  Quadcopter robot;  
  std::vector<std::thread> threads;
  if ((count_ not_eq 0) and (n_trajectory_ == 0)) {
    Quadcopter::State<simulator_t> state(sim_interface_);
    states_.push_back(state);

    auto it = states_.rbegin();
    it = std::next(it, 1);
    diff_height_.push_back((states_.back().height() - states_.back().height_f1()));
  }
  
  if (random_leader_action == true) {
    action_leader =
      robot.random_action_generator_with_only_opposed_condition(saved_leader_action_);
    saved_leader_action_ = action_leader;
  } else {
    action_leader = saved_leader_action_;
  }

  action_follower_.push_back
    (robot.random_action_generator_with_only_opposed_condition(saved_follower_action_));
  saved_follower_action_ = action_follower_.back();

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				    swarm_.one_quad_execute_trajectory("l" ,
								       action_leader,
								       1000);
				}));
  threads.push_back(std::thread([&](){
				    swarm_.one_quad_execute_trajectory("f1" ,
								       action_leader,
								       1000);
				}));
  threads.push_back(std::thread([&](){
				    swarm_.one_quad_execute_trajectory("f2",
								       action_follower_.back(),
								       1000);
				}));
  for(auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(300));


  /* Get the next state at time t + 1  */
  Quadcopter::State<simulator_t> nextState(sim_interface_);
  states_.push_back(nextState);
  auto it = states_.rbegin();
  it = std::next(it, 1);
  diff_height_.push_back((states_.back().height() - states_.back().height_f1()));
  return;
}

template<class flight_controller_t,
	 class simulator_t>
void Generator<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  lt::positions<lt::position3D<double>> original_positions = sim_interface_->positions();

  LogInfo() << "Starting positions : " << original_positions;
\
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

    /*  Handle fake takeoff.. */
    if (sim_interface_->positions().f1.z < 6  or sim_interface_->positions().f2.z < 6) {
      stop_episode_ = true;
    }
    
    /*  Start the First phase */
    /*  Collect dataset by creating a set of trajectories, each time
	the leader and the follower execute their trajectory randomly,
	we check the triangle (whether the follower is too close or
	too far) finally we break the loop after 10 trajectorise of 1
	second each */
    
    if (!stop_episode_) {
      /*  Verify that vectors are clear when starting new episode */
      states_.clear();
      action_follower_.clear();
      count_ = 0;

      std::vector<lt::triangle<double>> new_triangle;
      
      Quadcopter::State<simulator_t> initial_state(sim_interface_);
      states_.push_back(initial_state);
      diff_height_.push_back(0);
      while (count_ < 10 and !stop_episode_) {

	Quadcopter::Reward reward = Quadcopter::Reward::Unknown;
			
	lt::positions<lt::position3D<double>> positions_before_action =
	  sim_interface_->positions();
       	
	lt::triangle<double> triangle_before_action =
	  mtools_.triangle_side_3D(positions_before_action);

	LogInfo() << "Distanes before actions : " <<
	  mtools_.triangle_side_3D(positions_before_action);
	
	
	/* Choose one random trajectory for the leader in the first
	   count. Then, keep the same action until the end of the
	   episode */	
	for (n_trajectory_ = 0; n_trajectory_ < 2; ++n_trajectory_) {
	  if (count_ == 0) {
	    generate_trajectory(true);
	  } else {
	    generate_trajectory(false);
	  }
	}

	lt::positions<lt::position3D<double>> positions_after_action =
	  sim_interface_->positions();
	/* Get the distance between the TL TF, and FF TF  at time t*/
	new_triangle.push_back(mtools_.triangle_side_3D(positions_after_action));	
	
	LogInfo() << "Distances after action : " <<
	  mtools_.triangle_side_3D(positions_after_action);
	  
	/* Calculate the error compare to the starting point */	
	/* We have compared the value of the triangle with the one
	   before executing this action. Instead of comparing it to
	   the original one. But why? Why should this comparison
	   gives better learning score than the one before */
	
	Quadcopter robot;
	/*  Keep classification method */
	if (settings.classification()) {
	  /*  Classification */
	  if (count_ == 0 ) {
	    reward = robot.action_evaluator(triangle_before_action,
					    new_triangle.at(count_));
	  } else {
	    reward = robot.action_evaluator(new_triangle.at(count_ -1),
					    new_triangle.at(count_));
	  }
	}
	/*  Save the information generated from the trajectory into a
	    dataset file */	
	if (settings.classification()) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      mtools_.to_one_hot_encoding(reward, 4)
				      );
	}
	if (settings.regression()) {
	  /*  This condition need to be removed */
	  auto states_it = states_.rbegin();
	  states_it = std::next(states_it, 1);
	  auto states_it_2 = std::next(states_it, 1);
	  
	  auto height_it = diff_height_.begin();
	  height_it = std::next(height_it, 1);
	  
	  data_set_.save_csv_data_set(*(states_it_2),
				      diff_height_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.front(), 6),
				      *(states_it),
				      *(height_it),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      diff_height_.back()					
				      );
	}
	
	/*  Clear vectors after each generated line in the dataset */
	// states_.clear();
	action_follower_.clear();
	diff_height_.clear();
	
	/*  Check the triangle we are out of bound break the loop */
	if (mtools_.is_triangle(mtools_.triangle_side_3D
				(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  break;
	}
	++count_;
      }
    }

    /*  Add one count to have the exact number of time steps in the
	histogram since count start with 0*/
    if (!stop_episode_) {
      count_ = count_ + 1;
    }
    /*  Save a version of the time steps to create a histogram */
    mtools_.histogram(count_);
    data_set_.save_histogram(mtools_.get_histogram<int>());

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
