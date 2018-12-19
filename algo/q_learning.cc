# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed,
		       std::shared_ptr<Gazebo> gzs)
  :qtable_{64 ,std::vector<double>(5,1)}, /*  Init to ones */
   max_episode_(10000),
   max_step_(2),
   epsilon_(1),
   min_epsilon_(0),
   decay_rate_(0.01),
   learning_rate_(0.9),
   discount_rate_(0.95)
{

  run_episods(iris_x, speed, gzs);  
}

int Q_learning::get_action(std::vector<std::vector<double>> q_table , double state)   
{
  
  auto it = std::max_element(q_table.at(state).begin(), q_table.at(state).end());
  
  return *it;
}

double Q_learning::get_state_index(lt::rssi<double> signal, lt::rssi<double> original_signal)
{
  /*  verify if this algorithm is working */
  
  double x = signal.lf1 - (original_signal.lf1 - 2);
  if(x < 0)
    {
      x = 0;
    }
  else if (x > 3)
    {
      x = 3;
    }
  
  double y =  signal.lf2 - (original_signal.lf2 - 2);
  if(y < 0)
    {
      y = 0;
    }
  else if (y > 3)
    {
       y = 3;
    }
  
  double z =  signal.ff - (original_signal.ff - 2);
    if(z < 0)
      {
	z = 0;
      }
    else if (z > 3)
      {
	z = 3;
      }
    return x * 16 + y * 4 + z;
    
}

void Q_learning::move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     int action,
			     int quad_number)
{
  switch(action){

  case 0:
    iris_x.at(quad_number)->goLeft(speed);
    break;
  case 1:
    iris_x.at(quad_number)->goRight(speed);
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


/*  TODO LIST: */
/*
 * 1- Verify the generation of the random number in a different int test 
 * 2- Update and debug the q table
 * 3- Verify the altitude of RTL !?? why it is not working
 * 4- Rest the model using the code, find a way to do it !!! important
 * 5- At the end comment the code, and create small functions
*/


void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::shared_ptr<Gazebo> gzs)
{

  /*
   * Needs to review the algorithm, we do not know if it is working yet.
   * This code is complete for the q learning part. However, dron part needs to
   * be tested.
   */
  //to be overloaded

  // for( int i = 0; i <  qtable_.size(); i++){
  //   std::cout << qtable_.at(i) << std::endl;
  // }
  
  for (int episode = 0; episode < max_episode_; episode++){

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

    std::cout << "Episode : " << episode << std::endl;            


    /* Need to investigate in this method, This method will allow to
     * gain alot of simulation time instead of waiting until the quads
     * return to the home using RTL function.
     * To be tested using integration test. 
     */
    
    // gazebo::transport::NodePtr node(new gazebo::transport::Node());
    
    // /*  start resetting the world  */
    // gazebo::transport::PublisherPtr reset_pub;
    
    // reset_pub=node->Advertise<gazebo::msgs::WorldControl>("~/world_control");       
    
    // gazebo::msgs::WorldControl w_ctrl;
    
    // w_ctrl.mutable_reset()->set_all(true); //I believe this is equal to indicating a world reset
    
    
    std::vector<lt::position<double>> pos;    
    
    pos.push_back(gzs->get_positions().leader);
    pos.push_back(gzs->get_positions().f1);
    pos.push_back(gzs->get_positions().f2);
   
    std::cout << "New position l : " << pos.at(0) << std::endl;
    std::cout << "New position f1: " << pos.at(1) << std::endl;
    std::cout << "New position f2: " << pos.at(2) << std::endl;
    
    // double dif_x = pos.at(0).x - pos.at(1).x; 
    // double dif2_x = pos.at(0).x - pos.at(2).x; 


    
    /* TODO: Calculate the distance between each quadcopter */
        
    /*  put rssi inside a state */
    lt::rssi<double> e_rssi = gzs->rssi();
    
    // put the recoved signls in state struct
    
    original_signal_ = e_rssi;
    states_ = e_rssi;
    
    double state_index = get_state_index(states_,  original_signal_);

    //defin episode reward here
    double e_reward;
    
    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */    
    /*  put the take off functions inside the steps */
    /*  needs to land the drones and then reset using the world reset 
     *  functionality using the gazebo publish system
     * to be tested separatly 
     */
    
    for(int steps = 0; steps < max_step_; steps++){


      // reset_pub->Publish(w_ctrl);
    
    
    iris_x.at(0)->arm();
    iris_x.at(1)->arm();
    iris_x.at(2)->arm();
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    
    iris_x.at(0)->takeoff();
    iris_x.at(1)->takeoff();
    iris_x.at(2)->takeoff();
    
    std::this_thread::sleep_for(std::chrono::seconds(4));


    iris_x.at(0)->init_speed();
    iris_x.at(1)->init_speed();
    iris_x.at(2)->init_speed();

    iris_x.at(0)->start_offboard_mode();
    iris_x.at(1)->start_offboard_mode();
    iris_x.at(2)->start_offboard_mode();
    
    /*  intilization of position of the quads */
    
    std::this_thread::sleep_for(std::chrono::seconds(1));          

      
      int action = std::rand() % 4;

      for (int i = 0; i < 10; i++){
      move_action(iris_x, speed, action, 0) ; // move the leader 100 CM
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      //get the new signal of strength difference
      
      states_ = gzs->rssi();
      state_index = get_state_index(states_, original_signal_);
      
      // random number between 0 and 1 TODO::
      double random =   (double)std::rand()/((double)RAND_MAX+1); 
      std::cout << random << std::endl;

      int action1;
      int action2;
      
      /* Start exploitation instead of exploration, 
       * if the condition is valid
       * Look inside the Q_tables to find the best action
       */

      /*  find a way to look inside this tables */
      
      //////////////////////////////////////////////////////////
      // if(random > epsilon_){				      //
      // 						      //
      // 	action1 = get_action(qtable_, state_index);   //
      // 	action2 = action1;			      //
      // }						      //
      //////////////////////////////////////////////////////////
      // else {
    	action1 = std::rand() % 4;
    	action2 = action1;
	// }

      /*  moving the followers randomly */
      for (int i = 0; i < 10; i++){
       move_action(iris_x, speed, action1, 1);
       move_action(iris_x, speed, action2, 2);
       std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
      
       std::this_thread::sleep_for(std::chrono::seconds(2));
       
      /*  recalculate the RSSI after moveing the quads */
       /*  get the new  state index*/
      
       new_state_ = gzs->rssi();

       std::cout << "rssi values"<< gzs->rssi() << std::endl;
       state_index = get_state_index(new_state_, original_signal_);

       std::cout << "state_index: " << state_index << std::endl;
       
       std::vector<lt::position<double>> new_pos;    

       std::vector<double> error;
       
       new_pos.push_back(gzs->get_positions().leader);
       new_pos.push_back(gzs->get_positions().f1);
       new_pos.push_back(gzs->get_positions().f2);           

       std::cout << "New position l : " << new_pos.at(0) << std::endl;
       std::cout << "New position f1: " << new_pos.at(1) << std::endl;
       std::cout << "New position f2: " << new_pos.at(2) << std::endl;               	      



       std::cout  << "diff in pos1 " << pos.at(1).x - new_pos.at(1).x << std::endl;
       std::cout  << "diff in pos2 " << pos.at(2).x - new_pos.at(2).x << std::endl;
		
       error.push_back(std::sqrt(std::pow(pos.at(1).x - new_pos.at(1).x, 2) -
				 std::pow(pos.at(1).y - new_pos.at(1).y, 2)));
       
       error.push_back(std::sqrt(std::pow(pos.at(2).x - new_pos.at(2).x, 2) -
				 std::pow(pos.at(2).y - new_pos.at(2).y, 2)));
           

    // //   /*  Recalculate the distance between quadcopter  */
       std::cout << "Error :" << error << std::endl;

       
       int reward = 50 - error.at(0);
       int reward2 = 50 - error.at(1);
       
       
       e_reward = reward + reward2;
       
       std::cout << "reward: " << e_reward << std::endl;
       
       qtable_.at(state_index).at(action1) =
       	 (1 - learning_rate_) * qtable_.at(state_index).at(action1) +
       	 learning_rate_ * (reward +  discount_rate_ * get_action(qtable_,
       								 state_index));
       
       // //   //reduce epsilon as we explore more each episode
    //    epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode); 
       
    //    std::cout << "epsilon: " << epsilon_ << std::endl;
       
    //    //std::cout << "Score over time: " << 
       //      rewards_.push_back(e_reward);

       iris_x.at(0)->set_altitude_rtl_max(4.0);
       iris_x.at(1)->set_altitude_rtl_max(4.0);
       iris_x.at(2)->set_altitude_rtl_max(4.0);
           
       
       iris_x.at(0)->return_to_launch();
       iris_x.at(1)->return_to_launch();
       iris_x.at(2)->return_to_launch();
       
       std::this_thread::sleep_for(std::chrono::seconds(10));
       std::this_thread::sleep_for(std::chrono::seconds(15));
     }          
   }
}
  

