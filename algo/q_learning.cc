# include "q_learning.hh"

algo::Q_learning::Q_learning(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     float speed)
  :qtable_{64 ,std::vector<double>(5,1)}, /*  Init to ones */
   rewards_{}
{


  run_episodes(iris_x, speed);
  
}

int Q_learning::get_action(std::vector<std::vector<double>> q_table , double state)   
{
   
  auto it = max_element(std::begin(q_table.[state]), std::end(q_table.[state]));
   
}

double Q_learning::get_state_index(lt::rssi<double> signal, lt::rssi<double> original_signal)
{  
  double x = signal.lf1 - (original_signal.lf1 - 2)
    if(x < 0)
      x = 0;
    else if (x > 3)
      x = 3;
    
  double y =  signal.lf2 - (original_signal.lf2 - 2)
    if(y < 0)
      y = 0;
    else if (y > 3)
      y = 3;

  
  double z =  signal.ff - (original_signal.ff - 2)
    if(z < 0)
      z = 0;
    else if (z > 3)
      z = 3;
  
  return x * 16 + y * 4 + z;
    
}

void Q_learning::move_action(std::vector<std::shared_ptr<Px4Device>> iris_x,
			     int speed,
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

void Q_learning::run_episods(std::vector<std::shared_ptr<Px4Device>> iris_x, std::vector<Gazebo> gzs)
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
    

    
    std::cout << "Episode : " << episode << std::endl;            
    std::vector<lt::position> positions_;    
    
    positions_.push_back(iris_x[0].async_position_ned);
    positions_.push_back(iris_x[1].async_position_ned);

    double dif_x = positions_.at(0).x - positions_.at(1).x; 
    double dif_x = positions_.at(0).x - positions_.at(2).x; 
    
    
    /* TODO: Calculate the distance between each quadcopter */
        
    /*  put rssi inside a state */
    lt::rssi<double> e_rssi = rssi(gzs);
    
    // put the recoved signls in state tables

    states_ = e_rssi;
    double state_index = get_state_index(state,  original_signal);
    
    for(int steps = 0; steps < q_values::max_step; steps++){

      int action = std::rand() % 4;
      
      move_action(iris_x, action, 0) ; // move the leader 100 CM 

	   
      //get the new signal of strength difference
      
      states = rssi(gzs);

      int random = std::rand() % 10;
      
      int action1;
      int action2;

      /* Start exploitation instead of exploration, 
       * Look at the Q_tables
       */

      /*  find a way to look inside this tables */
      
      if(random > q_values::epsilon){
	
	action1 = get_action(q_table);
	action2 = action1;
      }
      else {
	action1 = std::rand() % 4;
	action2 = action1;
      }

      /*  moving the followers randomly */
      
      move_action(iris_x, action1, 1);
      move_action(iris_x, action2, 2);
      
      /*  recalculate the RSSI after moveing the quads  */
      
      new_state_ = gzs.rssi();                  
      
      /*  Recalculate the distance between quadcopter  */
      int reward = 50 - dist1;
      int reward2 = 50 - dist2;
      
      qtable_.at(state_index).at(action1) = (1 - algo::q_values::learning_rate) * qtable_() + algo::q_values::learning_rate * (reward +  algo::q_values::discount_rate * /*to continue*/);


      
      // qtable_.at(state_index).at(action1) = (1 - algo::q_values::learning_rate) * qtable_() + algo::q_values::learning_rate * (reward +  algo::q_values::discount_rate * /*to continue*/);
      
      
      
    }      
    
  }
  

