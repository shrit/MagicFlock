# include "generate_data_set.hh"


template<class flight_controller_t,
	 class simulator_t>
Generator<flight_controller_t, simulator_t>::
Generator (std::vector<std::shared_ptr<flight_controller_t>> quads,
	   std::shared_ptr<simulator_t> sim_interface)
  :count_(0),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   episode_(0),
   generator_(random_dev()),
   max_episode_(10000),  
   quads_(std::move(quads)),
   sim_interface_(std::move(sim_interface))
{}

/*  need to be moved to quadcopter class, think about it */
template <class flight_controller_t,
	  class simulator_t>
typename Quadcopter<simulator_t>::Reward
Generator<flight_controller_t, simulator_t>::
action_evaluator(lt::triangle<double> old_dist,
		 lt::triangle<double> new_dist)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);
  
  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);
  
  typename Quadcopter<simulator_t>::Reward reward =
    Quadcopter<simulator_t>::Reward::very_bad;
  
  if (0.5  > diff_f1 + diff_f2 ) {
    reward = Quadcopter<simulator_t>::Reward::very_good;      
  } else if ( 1.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 0.5 ) {
    reward = Quadcopter<simulator_t>::Reward::good;
  } else if ( 1.5  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.0 ) {
    reward = Quadcopter<simulator_t>::Reward::bad;
  } else if ( 2.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.5 ) {
    reward = Quadcopter<simulator_t>::Reward::very_bad;
  }  
  return reward;
}

template <class flight_controller_t,
	  class simulator_t>
void Generator<flight_controller_t, simulator_t>::
move_action(std::string label,
	    typename Quadcopter<simulator_t>::Action action)
{
  int quad_number = 0;
  
  if (label == "l") {
    quad_number = 0;
  } else if (label == "f1") {
    quad_number = 1;
  } else if (label == "f2") {
    quad_number = 2;
  }
        
  if (action == Quadcopter<simulator_t>::Action::left) {
    quads_.at(quad_number)->left(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::right) {  
    quads_.at(quad_number)->right(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::forward) { 
    quads_.at(quad_number)->forward(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::backward) { 
    quads_.at(quad_number)->backward(speed_);
    
  }  
}

/*  Data Set generation */
template <class flight_controller_t,
	  class simulator_t>
void Generator<flight_controller_t, simulator_t>::
phase_one(bool random_leader_action)
{
  typename Quadcopter<simulator_t>::Action action_leader ;
  
  std::vector<std::thread> threads;
    
  /* Get the state at time t  */
  typename Quadcopter<simulator_t>::State state(sim_interface_);
  states_.push_back(state);
  
  if ( random_leader_action == true) {
    
    action_leader = randomize_action() ;
    saved_leader_action_ = action_leader;
    //    LogInfo() << "Random action chosen: " << action_leader ;    
  } else {
    action_leader = saved_leader_action_;    
  }
  
  action_follower_.push_back(randomize_action());
  
  /*  Threading QuadCopter */    
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(quads_, "l" , speed_, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(quads_, "f1" , speed_, action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action(quads_, "f2", speed_, action_follower_.back());
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  for(auto& thread : threads) {
    thread.join();
  }
  
  /* We need to wait until the quadcopters finish their actions */  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  
  /* Get the next state at time t + 1  */
  typename Quadcopter<simulator_t>::State nextState(sim_interface_);
  states_.push_back(nextState);
  return ;
}


template <class flight_controller_t,
	  class simulator_t>
typename Quadcopter<simulator_t>::Action
Generator<flight_controller_t, simulator_t>::
randomize_action()
{  
  int random_action = distribution_int_(generator_);
  
  LogInfo() << "Random: " << random_action ;

  typename Quadcopter<simulator_t>::Action action
    = Quadcopter<simulator_t>::Action::forward ; 
  
  if( random_action == 0){
    action = Quadcopter<simulator_t>::Action::forward ;
  }
  else if(random_action == 1){
    action = Quadcopter<simulator_t>::Action::backward ;
  }    
  else if(random_action == 2){
    action = Quadcopter<simulator_t>::Action::left ;
  }
  else if (random_action == 3){
    action = Quadcopter<simulator_t>::Action::right ;   
  }
   return action;
}

template<class flight_controller_t,
	 class simulator_t>
void Generator<flight_controller_t, simulator_t>::
run()
{
  std::vector<lt::position<double>> distance;
  
  lt::positions<double> original_positions = sim_interface_.positions();
  
  LogInfo() << "Starting positions : " << original_positions ;
  
  lt::triangle<double> original_triangle = mtools_.triangle_side(original_positions);

  f3_side_.push_back(original_triangle);
  
  for (episode_ = 0; episode_ < max_episode_; ++episode_) { 
    
    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads: 
     * Arm + takeoff + offboard mode + moving + land 
     * Then: repeat each episode.
     */

    LogInfo() << "Episode : " << episode_ ;            

    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */
    
    /*  Think How we can use threads here */
    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool arm;
    bool stop_episode = false;    
    for (auto it : quads_){
      arm = it->arm();
      if(!arm)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool takeoff;
    for (auto it : quads_){
      takeoff = it->takeoff();
      if(!takeoff)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    /*  Setting up speed_ is important to switch the mode */
    for (auto it : quads_){
      it->init_speed();
    }
    
    /*  Switch to offboard mode, Allow the control */
    for(auto it : quads_){
      it->start_offboard_mode();
    }
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));  
    
    /*  Start the First phase, in 3 Steps */
             
    if (!stop_episode) {
      
      count_ = 0 ;
      
      std::vector<lt::triangle<double>> new_triangle;
      
      while (count_ < 3) {
	
        typename Quadcopter<simulator_t>::Reward reward =
	  Quadcopter<simulator_t>::Reward::very_bad;
	
	/*  if the follower has executed a good action we need to
	    re-discover the other action in this loop until we get a 0
	    reward, the other actions are related to exploration and
	    exploitation note that */
	/*  Test if the distances are good, if yes continue with the same
	    action for leader */
	
	if (count_ == 0 ) {
	  phase_one(true);	  
	} else {
	  phase_one(false);      
	}
	
	lt::positions<double> new_positions = sim_interface_.positions();
	
	LogInfo() << "New positions : " << new_positions ;
  	
	/*  Get the distance between the TL TF, and FF TF  at time t*/
	new_triangle.push_back(mtools_.triangle_side(new_positions));
	
	/*  Keep a copy of the new distance between all of them */
	f3_side_.push_back(mtools_.triangle_side(new_positions));
	
	/*Calculate the noise over the entire trainning session
	  This will allow to refine exactly the good action */	
	
	double noise = mtools_.gaussian_noise(f3_side_,
					      drift_f3_);
	
	LogInfo() << "Noise: " << noise ;
	
	/* Calculate the error compare to the starting point */
	/* Compare with the original at start */
	
	if (count_ == 0 ) {
	  reward = action_evaluator(original_triangle,
				    new_triangle.at(0));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(0)) == false) {
	    break;
	  }
	} else if (count_ == 1 ) {
	  reward = action_evaluator(new_triangle.at(0),
				    new_triangle.at(1));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }
	  
	} else if (count_ == 2 ) {
	  reward = action_evaluator(new_triangle.at(1),
				    new_triangle.at(2));
	  
	  /*  move it outside of the while loop */
	  if (mtools_.is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }	  	  
	}
	
	/*  1- problem logging the actions the next action is not
	    registered before being logged */
	
	auto it_state = states_.rbegin();
	auto it_action = action_follower_.rbegin();
	
	it_state = std::next(it_state, 1);
	it_action = std::next(it_action, 1);
	
	typename Quadcopter<simulator_t>::State sp(sim_interface_);
	
	data_set_.save_csv_data_set(sp.create_printer_struct(*it_state),
				    mtools_.to_one_hot_encoding(action_follower_.back(), 4),
				    sp.create_printer_struct(states_.back()),
				    mtools_.to_one_hot_encoding(reward, 4)
				    );	       	
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	++count_;
      }
    }
    
    for (auto it: quads_)
      it->land();
    
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    sim_interface_->reset_models();
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
    
    /*BIAS accelerometer problem after resetting the models*/
    
    /*  The only possible solution was to change the upper limit value
     * for the bias inside thee code of te firmware direclty. The
     * solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they propose
     * to increase the value of COM_ARM_EKF_AB. Note that, the default
     * value is 0.00024 I have increased it to 0.00054 which is very
     * high to the usual stadard. Other wise there is no way to do the
     * simulation. Remember, the reboot() function in the action class
     * is not implemented at the time of writing this comment, and
     * maybe it will never be implemented as it is quite complicated
     * to reboot the px4 software from the simulator. I understand
     * this choice, we need to leave a big sleep_for after resetting
     * the quadcopters, that is going to helpe resetting the
     * accelerometer values without any problems!
     */
    
    /*  I have quite tested a lot of different solution, if I am going
     * to find a better one, I will replace it directly. */      
  }
}
