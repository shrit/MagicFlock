# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed,
		       std::shared_ptr<Gazebo> gzs,
		       DataSet data_set,
		       bool train)
  :count_(0),
   decay_rate_(0.01),
   discount_rate_(0.95),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   episode_(0),
   epsilon_(1.0),
   learning_rate_(0.9),
   lower_threshold_{4, 4, 4},
   max_episode_(10000),
   max_step_(1000),
   min_epsilon_(0.0),
   qtable_{10000, 5, arma::fill::ones},
   rssi_lower_threshold_(1.1),
   rssi_upper_threshold_(0.94),
   upper_threshold_{9, 9, 9}
{

  if(train == true){
    run_episods(iris_x, speed, gzs, data_set);
  }
}

bool Q_learning::action_evaluator(lt::triangle<double> old_dist,
				  lt::triangle<double> new_dist,
				  double noise)
{
  LogInfo() << "F1 differences: " << std::fabs(old_dist.f1 - new_dist.f1);
  LogInfo() << "F2 differences: " << std::fabs(old_dist.f2 - new_dist.f2);
  
  if( std::fabs(old_dist.f1 - new_dist.f1 ) > noise or
      std::fabs(old_dist.f2 - new_dist.f2) > noise ) {
    return  false;    
  } else {
    return true;
  }
  
}

int Q_learning::cantor_pairing(int x, int y)
{
  int unique = 0.5*(x + y)*(x + y + 1) + y;
  return unique;  
}

double Q_learning::deformation_error(lt::triangle<double> old_dist,
				     lt::triangle<double> new_dist)
{
  double error;  
  
  error = std::sqrt(std::pow((old_dist.f1 - new_dist.f1), 2)  +
		    std::pow((old_dist.f2 - new_dist.f2), 2)  +
		    std::pow((old_dist.f3 - new_dist.f3), 2));
    /*  Recalculate the Error between quadcopters  */
  
  return error;   
}

double Q_learning::gaussian_noise(std::vector<lt::triangle<double>> distances)
{
  std::vector<double> ideal_f3;
  std::vector<double> result; 
  
  std::transform(distances.begin(), distances.end(), std::back_inserter(ideal_f3),
		 [](lt::triangle<double> const& t) { return t.f3; });
  
  std::adjacent_difference(ideal_f3.begin(), ideal_f3.end(), std::back_inserter(result));

  /* The difference in distances needs to be in absolute value */
  double (*fabs)(double) = &std::fabs;
  std::transform(result.begin(), result.end(), result.begin(), fabs);
  
  //  LogInfo() << "difference f3: " << result;
  
  // adding one here to remove the first element of adjacent difference
  double noise_average = std::accumulate(result.begin() + 1, 
				   result.end(), 0.0)/result.size();
  return noise_average;

}

lt::positions<double> Q_learning::get_positions(std::shared_ptr<Gazebo> gzs)
{  
  lt::positions<double> pos;
  pos.leader = gzs->get_positions().leader;
  pos.f1 = gzs->get_positions().f1;
  pos.f2 = gzs->get_positions().f2;

  return pos; 
}

bool Q_learning::is_signal_in_limits(std::shared_ptr<Gazebo> gzs)
{
  bool ok = false;
  
  float sum_of_neigh_signal = gzs->rssi().lf1() + gzs->rssi().ff() ;
  
  if ( sum_of_neigh_signal < sum_of_neigh_signal* rssi_upper_threshold_ ) {
    if ( sum_of_neigh_signal > sum_of_neigh_signal* rssi_lower_threshold_) {      
      ok = true;      
    }    
  }
  
  return ok;  
}

bool Q_learning::is_triangle(lt::triangle<double> t)
{
  bool value = false;
  if((t.f1 + t.f2 >  lower_threshold_.at(0))  and
     (t.f1 + t.f2 <  upper_threshold_.at(0))) {
    if ((t.f1 + t.f3 >  lower_threshold_.at(1))  and
	(t.f1 + t.f3 <  upper_threshold_.at(1))) {
      if ((t.f2 + t.f3 >  lower_threshold_.at(2))  and
	  (t.f2 + t.f3 <  upper_threshold_.at(2)))
	value = true;
    }
  }
  return value;   
}

void Q_learning::move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     std::string label,
			     float speed,
			     lt::action<bool> action)
{
  int quad_number = 0;
  
  if (label == "l"){
    quad_number = 0;
  }
  else if (label == "f1"){
    quad_number = 1;
  }
  else if (label == "f2"){
    quad_number = 2;
  }
        
  if(action.left == true){
    iris_x.at(quad_number)->left(speed);
  }
  else if(action.right == true){ 
    iris_x.at(quad_number)->right(speed);
  }
  else if(action.forward == true){ 
    iris_x.at(quad_number)->forward(speed); 
  }
  else if(action.backward == true){ 
    iris_x.at(quad_number)->backward(speed);
  }  
}

void Q_learning::phase_one(std::vector<std::shared_ptr<Px4Device>> iris_x,
			   float speed,
			   std::shared_ptr<Gazebo> gzs,
			   bool random_leader_action)
{
  lt::action<bool> action_leader = {false, false, false, false};
  
  LogInfo() << "RSSI before flying: " << gzs->rssi();
  
  if ( random_leader_action == true){

    action_leader = randomize_action() ;
    saved_leader_action_ = action_leader;
    LogInfo() << "Random action chosen: " << action_leader ;

  } else {
     action_leader = saved_leader_action_;    
  }
  
  for (int i = 0; i < 4; i++){
    move_action(iris_x, "l" , speed, action_leader) ;
    move_action(iris_x, "f1", speed, action_leader) ;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  LogInfo() << "RSSI on Leader action: " << gzs->rssi() ;
  /* Get the state (signal strength) at time t  */
  states_.push_back(gzs->rssi());
  
  /*  Get the random action for follower at time t */
  action_follower_.push_back(randomize_action());
    
  for (int i = 0; i < 4; i++){
    move_action(iris_x, "f2", speed, action_follower_.back());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } 
  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  LogInfo() << "RSSI on T follower action: " << gzs->rssi() ;
  /* Get the state (signal strength) at time t + 1  */
  states_.push_back(gzs->rssi());
    
  return ;  
}

void Q_learning::phase_two()
{
  
}

double Q_learning::qtable_action(arma::mat q_table, arma::uword state)   
{  
  arma::uword index;
  index = arma::index_max(q_table.row(state));        
  return index;
}

arma::uword Q_learning::qtable_state(std::shared_ptr<Gazebo> gzs, bool value)
{
  int unique = cantor_pairing((int)std::round(gzs->rssi().lf1()),
			      (int)std::round(gzs->rssi().lf2()));
  
  unique = cantor_pairing(unique,
			  (int)std::round(gzs->rssi().ff()));

  LogDebug() << "cantor unuqie number" << unique ;
  
  arma::uword index = 0;
  auto it = signal_map_.find(unique);
  if(it == signal_map_.end()){
    LogDebug() << "Signal is not yet aded to the table" ;
    if( value == true){
      signal_map_.insert(it, std::pair<int, int>(unique, episode_));
      index = episode_;
    }
  }
  else {
    index = it->second;      
  }
  
  return index;
}

arma::uword Q_learning::qtable_state_from_map(std::shared_ptr<Gazebo> gzs,
				      std::unordered_map<int, int> map)
{
  int unique = cantor_pairing((int)std::round(gzs->rssi().lf1()),
			      (int)std::round(gzs->rssi().lf2()));
  
  unique = cantor_pairing(unique,
			  (int)std::round(gzs->rssi().ff()));
  arma::uword index = 0;
  LogDebug() << "cantor unuqie" << unique ;
  auto it = map.find(unique);
  if(it == map.end()){
    LogDebug() << "Signal is not yet aded to the table" ;
  }
  else {
    index = it->second;      
  }
  
  return index;
}

double Q_learning::qtable_value(arma::mat q_table, arma::uword state)
{
  double qvalue = q_table.row(state).max();
  return qvalue;
}

lt::triangle<double> Q_learning::triangle_side(lt::positions<double> pos)
{
    lt::position<double> dist, dist2, dist3;
    
    /*  Distance between leader and FF */
    dist.x =  pos.leader.x - pos.f1.x;
    dist.y =  pos.leader.y - pos.f1.y;
    dist.z =  pos.leader.z - pos.f1.z;

    /* Distance between leader and TF */
    dist2.x = pos.leader.x - pos.f2.x;
    dist2.y = pos.leader.y - pos.f2.y;
    dist2.z = pos.leader.z - pos.f2.z;

    /* Distance between TF and FF */
    dist3.x = pos.f1.x - pos.f2.x;
    dist3.y = pos.f1.y - pos.f2.y;
    dist3.z = pos.f1.z - pos.f2.z;

    lt::triangle<double> t;

    t.f3 = std::sqrt(std::pow((dist.x), 2) +
		    std::pow((dist.y), 2));
    
    t.f1 = std::sqrt(std::pow((dist2.x), 2)+
		    std::pow((dist2.y), 2));
    
    t.f2 = std::sqrt(std::pow((dist3.x), 2)+
		    std::pow((dist3.y), 2));
    /*  it return the traingle side */
    return t;
}

lt::action<bool> Q_learning::randomize_action()
{
  lt::action<bool> action= {false, false, false, false};

  int random = distribution_int_(generator_);

  LogInfo() << "Random:  " << random ;
  
  if( random == 0){
    action.forward = true;
  }
  else if(random  == 1){
    action.backward = true;
  }    
  else if(random  == 2){
        action.left = true;	
  }
  else if (random == 3){
        action.right = true;
  }
  
  LogInfo() << "Action: " << action ;
  
  return action;

}

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::shared_ptr<Gazebo> gzs,
			     DataSet data_set)
{

  //  LogDebug() << qtable_ ;
      
  std::vector<lt::position<double>> distance;
  
  lt::positions<double> original_positions = get_positions(gzs);
  
  LogInfo() << "Starting positions : " << original_positions ;
  
  lt::triangle<double> original_triangle = triangle_side(original_positions);

  f3_side_.push_back( original_triangle);
  
  for (episode_ = 0; episode_ < max_episode_; episode_++){
    
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
    
    /*  Arming the Quads */
    for (auto it : iris_x){
      it->arm();
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool takeoff;
    bool stop_episode = false;    
    for (auto it : iris_x){
      takeoff = it->takeoff();
      if(!takeoff)
	stop_episode = true;
    }
                
    std::this_thread::sleep_for(std::chrono::seconds(3));
    /*  Setting up speed important to switch the mode */
   for (auto it : iris_x){
      it->init_speed();
    }
    
    /*  Switch to offboard mode, Allow the control */
    for(auto it : iris_x){
      it->start_offboard_mode();
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(1));  
    /* put the take off functions inside the steps */
    
    // for(int steps = 0; steps < max_step_; steps++) {
                  
      /*  Start the First phase*/
             
    if (!stop_episode) {
      
      count_ = 0 ;
      
      std::vector<lt::triangle<double>> new_triangle;
      
      while (count_ < 3) {

	int  reward = 0;
       
	/*  if the follower is has executed a good action
	    we need to re-discover the other action in this loop 
	    until we get a 0 reward, the other actions are related to exploration 
	    and exploitation note that  */
	/*  Test if the signal is good, if yes continue with the same action
	    for leader */
	
	if (count_ == 0 ) {
	  phase_one(iris_x, speed, gzs, true);	  
	} else {
	  phase_one(iris_x, speed, gzs, false);      
	}
	
	lt::positions<double> new_positions = get_positions(gzs);
	
	LogInfo() << "New positions : " << new_positions ;
  	
	/*  Get the distance between the TL TF, and FF TF  at time t*/
	new_triangle.push_back(triangle_side(new_positions));

	/*  Keep a copy of the new distance between all of them */
	f3_side_.push_back(triangle_side(new_positions));

	/*Calculate the noise over the entire trainning session
	  This will allow to refine exactly the good action */	

	double noise = gaussian_noise(f3_side_);

	LogInfo() << "Noise: " << noise ;
	
	/* Calculate the error compare to the starting point */
	/* Compare with the original at start */
	if (count_ == 0 ) {
	  reward = action_evaluator(original_triangle,
				    new_triangle.at(0),
				    noise);
	  
	  /*  move it outside of the while loop */
	  if (is_triangle(new_triangle.at(0)) == false) {
	    break;
	  }
	  
	} else if (count_ == 1 ) {
	  reward = action_evaluator(new_triangle.at(0),
				    new_triangle.at(1),
				    noise);
	  
	  /*  move it outside of the while loop */
	  if (is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }
	  	  
	} else if (count_ == 2 ) {
	  reward = action_evaluator(new_triangle.at(1),
				    new_triangle.at(2),
				    noise);
	  
	  /*  move it outside of the while loop */
	  if (is_triangle(new_triangle.at(1)) == false) {
	    break;
	  }	  	  
	}
	
	/*  1- problem logging the actions the next action is not
	    registered before being logged */
	
	auto it_state = states_.rbegin();
	auto it_action = action_follower_.rbegin();
	
	it_state = std::next(it_state, 1);
	it_action = std::next(it_action, 1);
	
	data_set.save_csv_data_set(*it_state,
				   action_follower_.back(),
				   states_.back(),
				   reward
				   );
			                
	LogInfo() << "reward: " << reward ;

	//reduce epsilon as we explore more each episode
	epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
	
	LogInfo() << "Epsilon: " << epsilon_ ;
	
	rewards_.push_back(reward);
	
	/*  Save the data set after each step iteration */		
	
	data_set.write_map_file(signal_map_);
	
	data_set.save_qtable(qtable_); 
	
	std::this_thread::sleep_for(std::chrono::seconds(1));
	
	++count_;
      }
    }
        
    for (auto it: iris_x)
      it->land();
    
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    gzs->reset_models();
    
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
