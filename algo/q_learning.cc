# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
		       float speed,
		       std::vector<Gazebo> gzs)
  :qtable_{64 ,std::vector<double>(5,1)}, /*  Init to ones */
   rewards_{}
{

  run_episods(iris_x, speed, gzs);  
}

int Q_learning::get_action(std::vector<std::vector<double>> q_table , double state)   
{
   
  auto it = std::max_element(std::begin(q_table.[state]), std::end(q_table.[state]));
  
  return *it;
}

double Q_learning::get_state_index(lt::rssi<double> signal, lt::rssi<double> original_signal)
{  
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

lt::rssi<double> Q_learning::rssi(std::vector<Gazebo> gzs)
{ 
  lt::rssi<double> rssi;
  rssi.lf1 = gzs.at(0).rssi();
  rssi.lf2 = gzs.at(1).rssi();
  rssi.ff  = gzs.at(2).rssi();
  
  return rssi;  
}

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed,
			     std::vector<Gazebo> gzs)
{
  
  for (int episode = 0; episode < q_values::max_episode; episode++){

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
    

    /*  start resetting the world  */
    


    /*  intilization of position of the quads */
    
    std::cout << "Episode : " << episode << std::endl;            
    std::vector<lt::position<float>> pos;    
    
    pos.push_back(iris_x[0].get_position_ned());
    pos.push_back(iris_x[1].get_position_ned());
    pos.push_back(iris_x[2].get_position_ned());
       
    double dif_x = pos.at(0).x - pos.at(1).x; 
    double dif2_x = pos.at(0).x - pos.at(2).x; 
    
    
    /* TODO: Calculate the distance between each quadcopter */
        
    /*  put rssi inside a state */
    lt::rssi<double> e_rssi = rssi(gzs);
    
    // put the recoved signls in state struct

    states_ = e_rssi;
    
    double state_index = get_state_index(states_,  original_signal);

    /* 
     * Step phase, where we update the q table each step
     * Each episode has a fixed step number.
     * Note: at the end of each episode, we re-intilize everything   
     */
    
    
    for(int steps = 0; steps < q_values::max_step; steps++){

      int action = std::rand() % 4;
      
      move_action(iris_x, speed, action, 0) ; // move the leader 100 CM 
      
	   
      //get the new signal of strength difference
      
      states_ = rssi(gzs);

      int random = std::rand() % 10;
      
      int action1;
      int action2;

      
      /* Start exploitation instead of exploration, 
       * if the condition is valid
       * Look inside the Q_tables to find the best action
       */

      /*  find a way to look inside this tables */
      
      if(random > q_values::epsilon){
	
	action1 = get_action(qtable_);
	action2 = action1;
      }
      else {
	action1 = std::rand() % 4;
	action2 = action1;
      }

      /*  moving the followers randomly */
      
      move_action(iris_x, speed, action1, 1);
      move_action(iris_x, speed, action2, 2);
      
      /*  recalculate the RSSI after moveing the quads */
      /*  get the new  */
      new_state_ = rssi(gzs);

      std::vector<lt::position<float>> new_pos;    

      std::vector<float> error;
      
      new_pos.push_back(iris_x.at(0).get_position_ned());
      new_pos.push_back(iris_x.at(1).get_position_ned());
      new_pos.push_back(iris_x.at(2).get_position_ned());
      
      error.push_back(std::sqrt(std::pow(pos.at(0).x - new_pos.at(0).x, 2) -
				std::pow(pos.at(0).y - new_pos.at(0).y, 2)));
      

      error.push_back(std::sqrt(std::pow(pos.at(1).x - new_pos.at(1).x, 2) -
				std::pow(pos.at(1).y - new_pos.at(1).y, 2)));
      
      error.push_back(std::sqrt(std::pow(pos.at(2).x - new_pos.at(2).x, 2) -
				std::pow(pos.at(2).y - new_pos.at(2).y, 2)));
      
      
      /*  Recalculate the distance between quadcopter  */
      int reward = 50 - error.at(0);
      int reward2 = 50 - error.at(1);
      
      qtable_.at(state_index).at(action1) =
	(1 - q_values::learning_rate) * qtable_.at(state_index).at(action1) +
	q_values::learning_rate * (reward +  q_values::discount_rate * get_action(qtable_,
										  state_index));
      
    }          
  }
}
  

