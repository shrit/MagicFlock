# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed,
		       std::shared_ptr<Gazebo> gzs,
		       DataSet data_set,
		       bool train)
  :qtable_{10000, 5, arma::fill::ones},
   max_episode_(10000),
   max_step_(1000),
   epsilon_(1.0),
   min_epsilon_(0.0),
   decay_rate_(0.01),
   learning_rate_(0.9),
   discount_rate_(0.95),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   episode_(0),
   upper_threshold_{11, 10, 10},
   lower_threshold_{4.8, 5, 5}
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
    arma::uword index = it->second;      
  }
  
  return index;
}

void Q_learning::move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     int action,
			     int quad_number)
{
  switch(action){

  case 0:
    iris_x.at(quad_number)->left(speed);
    break;
  case 1:
    iris_x.at(quad_number)->right(speed);
    break;
  case 2:
    iris_x.at(quad_number)->forward(speed); 
    break;
  case 3:
    iris_x.at(quad_number)->backward(speed);
    break;
  default:
    break;
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
    LogInfo() << "Sum of triangle " << t.a + t.b + t.c ;

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

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::shared_ptr<Gazebo> gzs,
			     DataSet data_set)
{

  LogDebug() << qtable_ ;
    
  
  std::vector<lt::position<double>> pos;    
  std::vector<lt::position<double>> distance;
  
  pos.push_back(gzs->get_positions().leader);
  pos.push_back(gzs->get_positions().f1);
  pos.push_back(gzs->get_positions().f2);
   
  LogInfo() << "Starting position l : " << pos.at(0) ;
  LogInfo() << "Starting position f1: " << pos.at(1) ;
  LogInfo() << "Starting position f2: " << pos.at(2) ;         
    
  for (episode_ = 0; episode_ < max_episode_; episode_++){
    
    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position. Thus,
     * advertise a gazebo topic that allow to control the world. From
     * here we can start automatically the quads: 
     * Arm + takeoff + offboard mode + moving + land 
     * Then: repeat eah episode.
     */

    /* This function in enormous, we need to recut it into Several
     * local function. Try to create several files for q_learning
     * instead of only this file. Do better encapsulation and comments
     * the code functions
     */

    LogInfo() << "Episode : " << episode_ ;            
    
    
  /*  use assert C++ 11 here and move this vector to above 
      the epsides  */
  /*  need to verify for error  */
    /*  NEED SOLUTION HERE */
    
    // lt::assert_equal(pos.at(0), gzs->get_positions().leader);    
    // lt::assert_equal(pos.at(1), gzs->get_positions().f1);
    // lt::assert_equal(pos.at(2), gzs->get_positions().f2);
    
    
    //defin episode reward here
    double e_reward = 0;
    
    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */
    
    /* put the take off functions inside the steps */
    
    for(int steps = 0; steps < max_step_; steps++){
      
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
            
      /*  intilization of position of the quads */
    

      std::this_thread::sleep_for(std::chrono::seconds(1));          

      double random =  distribution_(generator_);
    
      LogInfo() <<"Random number chosen: " << random ;

      int action_leader = distribution_int_(generator_);
    
      LogInfo() <<"Random action chosen: " << action_leader ;
            
      for (int i = 0; i < 10; i++){
	move_action(iris_x, speed, action_leader, 0) ;
	move_action(iris_x, speed, action_leader, 1);// move the leader 100 CM
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      // get the new signal of strength difference            
           
      // random number between 0 and 1 TODO::
      

      int action_follower = 0 ;

      index_ = qtable_state(gzs, false);
            
      if(random > epsilon_){			           
	/*  get the action from the qtable. Exploit */
	action_follower = qtable_action(qtable_, index_);
	LogInfo() << "Action From Qtable " << action_follower ;
      }						           
      else  {
	/*  get a random action. Explore */
	action_follower = distribution_int_(generator_);
      }

      /*  moving the followers randomly */
      for (int i = 0; i < 10; i++){
	move_action(iris_x, speed, action_follower, 2);
	std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
       
      /*  Recalculate the RSSI after moving the quads */
      /*  get the new  state index*/

      index_ = qtable_state(gzs, true);
      new_state_ = gzs->rssi();
    
      LogInfo() << "state_index: " << index_ ;
    
      LogInfo() << "RSSI: " << new_state_  ;
    
      std::vector<lt::position<double>> new_pos;    
      
       
      new_pos.push_back(gzs->get_positions().leader);
      new_pos.push_back(gzs->get_positions().f1);
      new_pos.push_back(gzs->get_positions().f2);           

      LogInfo() << "New position l : " << new_pos.at(0) ;
      LogInfo() << "New position f1: " << new_pos.at(1) ;
      LogInfo() << "New position f2: " << new_pos.at(2) ;               	      


      lt::triangle<double> t = triangle_side(new_pos);

       
      /*  Reward as a function of the triangle */

      /*  if error in the follower is big and the traingle is dead
	  reward is 0, reset the state */
      int  reward = 0;

      if (is_triangle(t) == true ) {
	reward = 1 ;	
      }
                                 
      LogInfo() << "reward: " << reward ;
       
      qtable_(index_, action_follower) =
	(1 - learning_rate_) * qtable_(index_, action_follower) +
	learning_rate_ * (reward +  discount_rate_ * qtable_action(qtable_,
								   index_)); 
      //reduce epsilon as we explore more each episode
      epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
      
      LogInfo() << "Epsilon: " << epsilon_ ;
      
      rewards_.push_back(reward);

      for (auto it: iris_x){
	it->land();
      }

      /*  Save the data set after each step iteration */
      
      data_set.save_csv_data_set(new_state_, action_follower, reward);
      
      data_set.write_map_file(signal_map_);
      
      data_set.save_qtable(qtable_);
      
      std::this_thread::sleep_for(std::chrono::seconds(6));
      
      if (reward == 0)
	break;
      
    } 
    
    gzs->reset_models();

    /*BIAS accelerometer problem after resetting the models*/
      
    /*  The only possible solution was to change the upper limit
     * value for the bias inside thee code of te firmware direclty.
     * The solution can be found at this link:
     * https://github.com/PX4/Firmware/issues/10833 Where they
     * propose to increase the value of COM_ARM_EKF_AB. Note that,
     * the default value is 0.00024 I have increased it to 0.00054
     * which is very high to their standard. Because other wise
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
    
    
    std::this_thread::sleep_for(std::chrono::seconds(15));  
  }
}
