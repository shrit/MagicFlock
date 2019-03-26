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
   lower_threshold_{4.8, 5, 5},
   max_episode_(10000),
   max_step_(1000),
   min_epsilon_(0.0),
   qtable_{10000, 5, arma::fill::ones},
   rssi_lower_threshold_(1.1),
   rssi_upper_threshold_(0.94),
   upper_threshold_{11, 9, 9}
{

  if(train == true){
    run_episods(iris_x, speed, gzs, data_set);
  }
}

int Q_learning::cantor_pairing(int x, int y)
{
  int unique = 0.5*(x + y)*(x + y + 1) + y;
  return unique;  
}

double Q_learning::qtable_action(arma::mat q_table ,arma::uword state)   
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


lt::triangle<double> Q_learning::triangle_side(std::vector<lt::position<double>> pos)
{
    lt::position<double> dist, dist2, dist3;

    /*  change the distance position to struct
	calculate the distance on the 3 access
     */  
    
    dist.x =  pos.at(0).x - pos.at(1).x;
    dist.y =  pos.at(0).y - pos.at(1).y;
    dist.z =  pos.at(0).z - pos.at(1).z;
    
    dist2.x = pos.at(0).x - pos.at(2).x;
    dist2.y = pos.at(0).y - pos.at(2).y;
    dist2.z = pos.at(0).z - pos.at(2).z;
    
    dist3.x = pos.at(1).x - pos.at(2).x;
    dist3.y = pos.at(1).y - pos.at(2).y;
    dist3.z = pos.at(1).z - pos.at(2).z;


    lt::triangle<double> t;

    t.a = std::sqrt(std::pow((dist.x), 2) +
		    std::pow((dist.y), 2));
    
    t.b = std::sqrt(std::pow((dist2.x), 2)+
		    std::pow((dist2.y), 2));
    
    t.c = std::sqrt(std::pow((dist3.x), 2)+
		    std::pow((dist3.y), 2));

    LogInfo() << "A = " << t.a << "B = " << t.b << "C = " << t.c ;

    return t;
  
}

double Q_learning::deformation_error(std::vector<lt::position<double>> pos)
{

  lt::position<double> dist, dist2;
  
  std::vector<lt::position<double>> distance;
  
  std::vector<double> error;
        
  dist.x =  pos.at(0).x - pos.at(1).x;
  dist.y =  pos.at(0).y - pos.at(1).y;
  dist.z =  pos.at(0).z - pos.at(1).z;
  
  distance.push_back(dist);
  
  dist2.x = pos.at(0).x - pos.at(2).x;
  dist2.y = pos.at(0).y - pos.at(2).y;
  dist2.z = pos.at(0).z - pos.at(2).z;
  
  distance.push_back(dist2);       
  
  error.push_back(std::sqrt(std::pow((distance.at(0).x - distance.at(0).x), 2)  +
			    std::pow((distance.at(0).y - distance.at(0).y), 2)));
  
  error.push_back(std::sqrt(std::pow((distance.at(1).x - distance.at(1).x), 2)  +
			    std::pow((distance.at(1).y - distance.at(1).y), 2)));
  
  
  /*  Recalculate the Error between quadcopters  */
  LogInfo() << "Error :" << error;
  
  double sum_of_error = 0;
  
  for (auto& n : error)
    sum_of_error += n; 
  
  
  return sum_of_error;
   
}

bool Q_learning::is_triangle(lt::triangle<double> t)
{
  bool value = false;
  if((t.a + t.b >  lower_threshold_.at(0))  && (t.a + t.b <  upper_threshold_.at(0))) {
    if ((t.a + t.c >  lower_threshold_.at(1))  && (t.a + t.c <  upper_threshold_.at(1))) {
      if ((t.b + t.c >  lower_threshold_.at(2))  && (t.b + t.c <  upper_threshold_.at(2)))
	value = true;
    }
  }
  return value; 
  
}


lt::action<bool> Q_learning::randomize_action()
{
  lt::action<bool> action= {false, false, false, false};

  if(distribution_int_(generator_) == 0){
    action.forward = true;
  }
  else if(distribution_int_(generator_) == 1){
    action.backward = true;
  }    
  else if(distribution_int_(generator_) == 2){
        action.left = true;	
  }
  else if (distribution_int_(generator_) == 3){
        action.right = true;
  }

  return action;

}

void Q_learning::phase_one(std::vector<std::shared_ptr<Px4Device>> iris_x,
			   float speed,
			   std::shared_ptr<Gazebo> gzs,
			   bool random_leader_action)
{
  lt::action<bool> action_leader = {false, false, false, false};
  
  if ( random_leader_action == true){
    action_leader = randomize_action() ;
    saved_leader_action_ = action_leader;
    LogInfo() << "Random action chosen: " << action_leader ;
  }
  else{
     action_leader = saved_leader_action_;    
  }
  
  for (int i = 0; i < 5; i++){
    move_action(iris_x, "l" ,speed, action_leader) ;
    move_action(iris_x, "f1",speed, action_leader);// move the leader 100 CM
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  /* Get the state (signal strength) at time t  */
  states_.push_back(gzs->rssi());
  

  /*  Get the random action for follower at time t */
  action_follower_.push_back(randomize_action());
  
  
  for (int i = 0; i < 3; i++){
    move_action(iris_x, "f2" ,speed, action_follower_.back());
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } 
  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  /* Get the state (signal strength) at time t + 1  */
  states_.push_back(gzs->rssi());
  
  
  return ;
  
}

void Q_learning::phase_two()
{

  
}

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::shared_ptr<Gazebo> gzs,
			     DataSet data_set)
{

  LogDebug() << qtable_ ;
  
  
//   std::vector<lt::position<double>> pos;    
//   std::vector<lt::position<double>> distance;
  
//   pos.push_back(gzs->get_positions().leader);
//   pos.push_back(gzs->get_positions().f1);
//   pos.push_back(gzs->get_positions().f2);
   
//   LogInfo() << "Starting position l : " << pos.at(0) ;
//   LogInfo() << "Starting position f1: " << pos.at(1) ;
//   LogInfo() << "Starting position f2: " << pos.at(2) ;         
    
  for (episode_ = 0; episode_ < max_episode_; episode_++){
    
    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position. Thus,
     * advertise a gazebo topic that allow to control the world. From
     * here we can start automatically the quads: 
     * Arm + takeoff + offboard mode + moving + land 
     * Then: repeat each episode.
     */

    /* This function in enormous, we need to recut it into Several
     * local function. Try to create several files for q_learning
     * instead of only this file. Do better encapsulation and comments
     * the code functions
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
    
    /*  Taking off the quad */
    for (auto it : iris_x){
      it->takeoff();	
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
    
    for(int steps = 0; steps < max_step_; steps++){
                  
      /*  Start the First phase*/
          

      //index_ = qtable_state(gzs, false);

      /*  create a signal strength limits functions that tests whether
       the choosen action is good or not*/
      /*  according to this action we can define if it is good or not
	  by surveying the singal strength, then we define the rewards according 
	  to this scenario */
      
      // double random =  distribution_(generator_);
      
      // LogInfo() <<"Random number chosen: " << random ;
      
      // /*  Epsilon greedy */      
      // if(random > epsilon_){			           

      // }						           
      // else  {

      // }      
                   
      /*  Recalculate the RSSI after moving the quads */
      /*  get the new  state index*/

      // index_ = qtable_state(gzs, true);

    
      // LogInfo() << "state_index: " << index_ ;
        
      // std::vector<lt::position<double>> new_pos;    
             
      // new_pos.push_back(gzs->get_positions().leader);
      // new_pos.push_back(gzs->get_positions().f1);
      // new_pos.push_back(gzs->get_positions().f2);           

      // LogInfo() << "New position l : " << new_pos.at(0) ;
      // LogInfo() << "New position f1: " << new_pos.at(1) ;
      // LogInfo() << "New position f2: " << new_pos.at(2) ;               	      


      // lt::triangle<double> t = triangle_side(new_pos);

       
      /*  Reward as a function of the triangle */

      /*  if error in the follower is big and the traingle is dead
	  reward is 0, reset the state */
      int  reward = 0;
      count_ = 0 ; 
      /*  if the follower is has executed a good action
       we need to rediscover the other action in this loop 
      until we get a 0 reward, the ather actions are related to exploration 
      and exploitation note that  */
      /*  Test if the signal is good, if yes continue with the same action
      for leader */

      phase_one(iris_x, speed, gzs, true);      
      
      if(is_signal_in_limits(gzs) == true) {
      	reward = 1;
      }
      else{
      	count_ = 4;
      }
      
      // Save the generated dataset HERE
      if (action_follower_.size() == 1 ){
	/*  The first case scenario */
	/*  Do not save any thing Yet */
      }
      else{
	auto it_state = states_.rbegin();
	auto it_action = action_follower_.rbegin();
    
	data_set.save_csv_data_set(*(it_state++),
				   *(it_action++),
				   states_.back(),
				   action_follower_.back(),
				   reward
				   );
        
       }                    
             
      while (count_ < 3)  {
      	phase_one(iris_x, speed, gzs, false);
	++count_;
      }    
            
      LogInfo() << "reward: " << reward ;
       
      // qtable_(index_, action_follower) =
      // 	(1 - learning_rate_) * qtable_(index_, action_follower) +
      // 	learning_rate_ * (reward +  discount_rate_ * qtable_action(qtable_,
      // 								   index_));  
      //reduce epsilon as we explore more each episode
      epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
      
      LogInfo() << "Epsilon: " << epsilon_ ;
      
      rewards_.push_back(reward);

      /*  Save the data set after each step iteration */
      
      //      data_set.save_csv_data_set(new_state_, action_follower, reward);
      
      data_set.write_map_file(signal_map_);
      
      data_set.save_qtable(qtable_);
      
      std::this_thread::sleep_for(std::chrono::seconds(1));
      
      //      if (is_triangle(t) == false)
      //	break;
     
    }
    
    for (auto it: iris_x)
      it->land();
      
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    gzs->reset_models();

    std::this_thread::sleep_for(std::chrono::seconds(15));
    
    /*BIAS accelerometer problem after resetting the models*/
      
    /*  The only possible solution was to change the upper limit
     * value for the bias inside thee code of te firmware direclty.
     * The solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they
     * propose to increase the value of COM_ARM_EKF_AB. Note that,
     * the default value is 0.00024 I have increased it to 0.00054
     * which is very high to their stadard. Because other wise
     * there is no way to do the simulation. Remember, the reboot()
     * function in the action class is not implemented at the time
     * of writing this comment, and maybe it will never be
     * implemented as it is quite complicated to reboot the px4
     * software from the simulator.
     * I understand this choice, we need to leave a big sleep_for 
     * after resetting the quadcopters, that is going to helper
     * resetting the accelerometer values without any problems!
     */

    /*  I have quite tested a lot of different solution, if I am going
     * to find a better one, I will replace it directly. */      
   }
 }
