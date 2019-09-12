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
phase_one(bool random_leader_action)
{
  Quadcopter::Action action_leader;
  Quadcopter robot;

  std::vector<std::thread> threads;

  /* Get the state at time t  */
  Quadcopter::State<simulator_t> state(sim_interface_);
  states_.push_back(state);

  if (random_leader_action == true) {

    action_leader = robot.random_action_generator();
    saved_leader_action_ = action_leader;

  } else {
    action_leader = saved_leader_action_;
  }

  action_follower_.push_back(robot.random_action_generator());

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 16; ++i) {
				    swarm_.one_quad_execute_trajectory("l" ,  action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(45));
				  }
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 16; ++i) {
				    swarm_.one_quad_execute_trajectory("f1" , action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(45));
				  }
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 16; ++i) {
				    swarm_.one_quad_execute_trajectory("f2", action_follower_.back());
				    std::this_thread::sleep_for(std::chrono::milliseconds(45));
				  }
				}));
  for(auto& thread : threads) {
    thread.join();
  }

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
    
  /* Get the next state at time t + 1  */
  Quadcopter::State<simulator_t> nextState(sim_interface_);
  states_.push_back(nextState);
  return;
}

template<class flight_controller_t,
	 class simulator_t>
void Generator<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  lt::positions<lt::position3D<double>> original_positions = sim_interface_->positions();

  LogInfo() << "Starting positions : " << original_positions;

  lt::triangle<double> original_triangle = mtools_.triangle_side_3D(original_positions);

  f3_side_.push_back(original_triangle);

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

    /*  Start the First phase */

    if (!stop_episode_) {

      count_ = 0 ;

      std::vector<lt::triangle<double>> new_triangle;

      while (count_ < 10 and !stop_episode_) {

	Quadcopter::Reward reward = Quadcopter::Reward::Unknown;
	
	/* Choose one random trajectory for the leader in the first
	   count. Then, keep the same action until the end of the
	   episode */
	
	/*  At the end of each phase, quadcopters have already
	    finished their trajectories. Thus, we need to test the
	    triangle*/
	
	if (count_ == 0 ) {
	  phase_one(true);
	} else {
	  phase_one(false);
	}

	/* Get the actual position, test if the triangle is OK */
	lt::positions<lt::position3D<double>> new_positions_gazebo = sim_interface_->positions();
	lt::positions<lt::position_GPS<double>> new_positions_sdk = swarm_.positions_GPS();
	  
	LogInfo() << "New positions Gazebo : " << new_positions_gazebo;
	LogInfo() << "New positions SDK : "    << new_positions_sdk;
	
	/* Get the distance between the TL TF, and FF TF  at time t*/
	new_triangle.push_back(mtools_.triangle_side_3D(new_positions_gazebo));

	/* Keep a copy of the new distance between all of them */
	f3_side_.push_back(mtools_.triangle_side_3D(new_positions_gazebo));

	/* Calculate the noise over the entire trainning session
	   This will allow to refine exactly the good action */

	double noise = mtools_.gaussian_noise(f3_side_,
					      drift_f3_);

	LogInfo() << "Noise: " << noise ;

	/*  Handle fake takeoff.. */
	if (sim_interface_->positions().f1.z < 6  or sim_interface_->positions().f2.z < 6) {
	  stop_episode_ = true;
	} 
  
	/* Calculate the error compare to the starting point */
	/* Compare with the original at start */
	/* We have compared the value of the triangle with the one
	   before executing this action. Instead of comparing it to
	   the original one. But why? Why should this comparison
	   gives better learning score than the one before */
	
	Quadcopter robot;
	/*  This problem is formulated as a classification problem, we
	 will try using regression rather than classification to see
	 if the produced result are better */
	if (settings.classification()) {
	  /*  Classification */
	  if (count_ == 0 ) {
	    reward = robot.action_evaluator(original_triangle,
					    new_triangle.at(count_));	    
	  } else {
	    reward = robot.action_evaluator(new_triangle.at(count_ -1),
					    new_triangle.at(count_));
	  }
	}
	
	double score = -1;
	double score_log = -20;
	double score_square = -1;
	double score_square_log = -1;	
	if (settings.regression()) {
	/*  Regression */
	  if (count_ == 0 ) {
	    score = robot.true_score(original_triangle,
				     new_triangle.at(count_));
	    score_log= robot.true_score_log(original_triangle,
					    new_triangle.at(count_));
	    score_square = robot.true_score_square(original_triangle,
						   new_triangle.at(count_));
	    score_square_log = robot.true_score_square_log(original_triangle,
							   new_triangle.at(count_));
	  } else {
	    score = robot.true_score(new_triangle.at(count_ -1),
				     new_triangle.at(count_));
	    score_log = robot.true_score_log(new_triangle.at(count_ -1),
					     new_triangle.at(count_));
	    score_square = robot.true_score_square(new_triangle.at(count_ -1),
						   new_triangle.at(count_));
	    score_square_log = robot.true_score_square_log(new_triangle.at(count_ -1),
							   new_triangle.at(count_));
	  }
	}
	
	if (mtools_.is_triangle(mtools_.triangle_side_3D
				(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  break;
	}
	
	if (settings.classification()) {
	  data_set_.save_csv_data_set(states_.front(),
				    mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				    (states_.back()),
				    mtools_.to_one_hot_encoding(reward, 4)
				    );
	}
	if (settings.regression()) {
	  if (states_.front().height() > 7.5) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      score,
				      score_log,
				      score_square,
				      score_square_log
				      );
	  }
	}
	states_.clear();
	std::this_thread::sleep_for(std::chrono::seconds(1));
	++count_;
      }
    }
    /*  Add one count to have the exact number of time steps */
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

    LogInfo() << "The quadcopters have been resetted...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));

    /*BIAS accelerometer problem after resetting the models*/

    /*  The only possible solution was to change the upper limit value
     * for the bias inside thee code of the firmware directly. The
     * solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they propose
     * to increase the value of COM_ARM_EKF_AB. Note that, the default
     * value is 0.00024 I have increased it to 0.00054 which is very
     * high to the usual stadard. Otherwise there is no way to do the
     * simulation. Remember, the reboot() function in the action class
     * is not implemented at the time of writing this comment, and
     * maybe it will never be implemented as it is quite complicated
     * to reboot the px4 software from the simulator. I understand
     * this choice, we need to leave a big sleep_for after resetting
     * the quadcopters, that is going to helpe resetting the
     * accelerometer values without any problems!
     */

    /*  I have quite tested a lot of different solution. Frankly,if I am going
     * to find a better one, I will replace it directly. */
  }
}
