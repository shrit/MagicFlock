# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed,
		       std::shared_ptr<Gazebo> gzs,
		       DataSet data_set)
  :qtable_{10000, 5, arma::fill::ones},
   max_episode_(5000),
   max_step_(2),
   epsilon_(1.0),
   min_epsilon_(0.0),
   decay_rate_(0.01),
   learning_rate_(0.9),
   discount_rate_(0.95),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3)
{
  run_episods(iris_x, speed, gzs, data_set);  
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

int Q_learning::qtable_state(std::shared_ptr<Gazebo> gzs)
{
  int unique = cantor_pairing((int)std::round(gzs->rssi().lf1()),
			      (int)std::round(gzs->rssi().lf2()));
  
  unique = cantor_pairing(unique,
			  (int)std::round(gzs->rssi().ff()));
  
  int index = 0;
  auto it = signal_map_.find(unique);
  if(it == signal_map_.end()){
    std::cout << "Signal is not yet aded to the table" << std::endl;
    signal_map_.insert(it, std::pair<int, int>(unique, episode_));
    index = episode_;
  }
  else {
    index = it->second;      
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

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::shared_ptr<Gazebo> gzs,
			     DataSet data_set)
{

  boost::log::sources::severity_logger<level> lg;
  
  std::ofstream file;
  file.open("data_sample");

  /*
   * Needs to review the algorithm, we do not know if it is working yet.
   * This code is complete for the q learning part. However, drone part needs to
   * be tested.
   */
  //to be overloaded


  std::cout << qtable_ << std::endl;
  
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

    std::cout << "Episode : " << episode_ << std::endl;            
    
    
    /* Need to investigate in Reset Model method, it will allow to
     * gain alot of simulation time instead of waiting until the quads
     * return to the home using RTL function.
     * To be tested using integration test. 
     */

    /*  Reset model method descibed in gazebo source code */
        
    std::vector<lt::position<double>> pos;    
    std::vector<lt::position<double>> distance;
    
    pos.push_back(gzs->get_positions().leader);
    pos.push_back(gzs->get_positions().f1);
    pos.push_back(gzs->get_positions().f2);
   
    std::cout << "position l : " << pos.at(0) << std::endl;
    std::cout << "position f1: " << pos.at(1) << std::endl;
    std::cout << "position f2: " << pos.at(2) << std::endl;
   
    lt::position<double> dist, dist2;

    dist.x =  pos.at(0).x - pos.at(1).x;
    dist.y =  pos.at(0).y - pos.at(1).y;
    dist.z =  pos.at(0).z - pos.at(1).z;
    
    distance.push_back(dist);

    dist2.x = pos.at(0).x - pos.at(2).x;
    dist2.y = pos.at(0).y - pos.at(2).y;
    dist2.z = pos.at(0).z - pos.at(2).z;


    distance.push_back(dist2);
    
    /* TODO: Calculate the distance between each quadcopter */

    //defin episode reward here
    double e_reward;
    
    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */
    
    /* put the take off functions inside the steps */
    
    //for(int steps = 0; steps < max_step_; steps++){

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
    std::cout << random << std::endl;

    int action = distribution_int_(generator_);
    std::cout << action << std::endl;
            
    for (int i = 0; i < 10; i++){
      move_action(iris_x, speed, action, 0) ; // move the leader 100 CM
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    // get the new signal of strength difference            
           
    // random number between 0 and 1 TODO::
      

    int action1 = 0 ;

    /*  Create a function that recover the state index 
     *  from the know signal stregnth
     *  move the down map into this function
     */
      
    index_ = qtable_state(gzs);

    /* Start exploitation instead of exploration, 
     * if the condition is valid
     * Look inside the Q_tables to find the best action
     */
            
    if(random > epsilon_){			           
	
      action1 = qtable_action(qtable_, index_);
      std::cout << "action " << action1 << std::endl;
    }						           
    else  {
      action1 = distribution_int_(generator_);
    }

    /*  moving the followers randomly */
    for (int i = 0; i < 10; i++){
      move_action(iris_x, speed, action1, 1);
      move_action(iris_x, speed, action1, 2);
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
      
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
       
    /*  Recalculate the RSSI after moving the quads */
    /*  get the new  state index*/


    index_ = qtable_state(gzs);
    new_state_ = gzs->rssi();
      
    std::cout << "state_index: " << index_ << std::endl;
    
    std::cout << "RSSI: " << new_state_ << std::endl;
    
    std::vector<lt::position<double>> new_pos;    

    std::vector<double> error;
       
    new_pos.push_back(gzs->get_positions().leader);
    new_pos.push_back(gzs->get_positions().f1);
    new_pos.push_back(gzs->get_positions().f2);           

    std::cout << "New position l : " << new_pos.at(0) << std::endl;
    std::cout << "New position f1: " << new_pos.at(1) << std::endl;
    std::cout << "New position f2: " << new_pos.at(2) << std::endl;               	      

          
    lt::position<double> new_dist, new_dist2;

    std::vector<lt::position<double>> new_distance;
       
    new_dist.x =  new_pos.at(0).x - new_pos.at(1).x;
    new_dist.y =  new_pos.at(0).y - new_pos.at(1).y;
    new_dist.z =  new_pos.at(0).z - new_pos.at(1).z;
       
    new_distance.push_back(new_dist);
       
    new_dist2.x = new_pos.at(0).x - new_pos.at(2).x;
    new_dist2.y = new_pos.at(0).y - new_pos.at(2).y;
    new_dist2.z = new_pos.at(0).z - new_pos.at(2).z;
              
    new_distance.push_back(new_dist2);       
       
    error.push_back(std::sqrt(std::pow((distance.at(0).x - new_distance.at(0).x), 2)  +
			      std::pow((distance.at(0).y - new_distance.at(0).y), 2)));
       
    error.push_back(std::sqrt(std::pow((distance.at(1).x - new_distance.at(1).x), 2)  +
			      std::pow((distance.at(1).y - new_distance.at(1).y), 2)));
       
      
    /*  Recalculate the Error between quadcopters  */
    std::cout << "Error :" << error << std::endl;

    double sum_of_error = 0;
      
    for (auto& n : error)
      sum_of_error += n; 
       
    double reward = 0.5 - error.at(0);
    double reward2 = 0.5 - error.at(1);
              
    e_reward = reward + reward2;
       
    std::cout << "reward: " << e_reward << std::endl;
       
    qtable_(index_, action1) =
      (1 - learning_rate_) * qtable_(index_, action1) +
      learning_rate_ * (reward +  discount_rate_ * qtable_action(qtable_,
								 index_)); 
    //reduce epsilon as we explore more each episode
    epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
      
    std::cout << "epsilon: " << epsilon_ << std::endl;
      
    rewards_.push_back(e_reward);

    for (auto it: iris_x){
      it->land();
    }
       
    std::this_thread::sleep_for(std::chrono::seconds(5));

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
      
    data_set.write_csv_data_set_file(file, new_state_, action1, sum_of_error);
      
    std::this_thread::sleep_for(std::chrono::seconds(15));                  
    //      BOOST_LOG_SEV(lg, Msg) << action1 ;
    
    qtable_.save("qtable", arma::raw_ascii);
  }
  /* 
   * Here I save the generated data set in a specifc file.
   * We use boost log to do that.
   * I am intended to use supervised learning in order to keep 
   * the formation as it has been.
   * Further, we need to look at perceptron in Python and in C++
   * use python to get faster results. Then integrate the result 
   * in the simulation
   */

      

}

  

