# include "q_learning.hh"

Q_learning::Q_learning(std::vector<std::shared_ptr<Controller>> controllers)
  :qtable_{4096,5},
   states_{4096,3},
   rewards_{}
{

  /* Initlize the matrix for the algorithm */
  q_tables_ = MatrixXd::Constant(4096, 5, 0);
  states_   = MatrixXd::Constant(4096, 3, 0);

  run_episodes(controllers);
  
}

void Q_learning::move_quads_followers_action(std::vector<std::shared_ptr<Controller>> controllers,
					     int action)
{
  switch(action){

  case 0:
    controllers.at(1)->goLeft();
    break;
  case 1:
    controllers.at(1)->goRight();
    break;
  case 2:
    controllers.at(1)->forward(); 
    break;
  case 3:
    controllers.at(1)->backward();
    break;
  default:
    continue;
  }  
}

void Q_learning::run_episods(std::vector<std::shared_ptr<Controller>> controllers)
{
  
  for (int episode = 0; episode < q_values::max_episode; episode++){
    
    std::cout << "Episode : " << episode << std::endl;            
    
    lt::position position_ = gazebo_.get_quad_positions();

    /* TODO: Calculate the distance between each quadcopter */
    
    states_.row(episode) = gazebo_.get_quads_rssi();
    
    // put the recoved signls in state tables 
    
    for(int steps = 0; steps < q_values::max_step; steps++){
      
      controllers[0]->forward(); // reiterate to move at
      // least 10 centometers,
					    // or find another
					    // solution
	   
      //get the new signal of strength difference
      
      states_.row(episode) = gazebo_.get_quads_rssi();
      in random = std::rand() % 10;
      
      int action1;
      int action2;
      
      if(random > q_values::epsilon){
	Eigen::VectorXd v, v1;
	v = qtable_.row(episode);
	v1= qtable_
	
	action1 = v.maxCoeff();
	action2 = matrix();
      }
      else {
	action1 = std::rand() % 4;
	action2 = action1;
      }
      
      move_quads_followers_action(controllers, action1);
      move_quads_followers_action(controllers, action2);
      
      /*  recalculate the RSSI after moveing the quads  */
      
      new_states_.row(episode) = gazebo_.get_quads_rssi();                  
      
      
      /*  Recalculate the distance between quadcopter  */
      int reward = 50 - dist1;
      int reward2 = 50 - dist2;
      
      qtable_() = (1 - q_values::learning_rate) * qtable_() + q_values::learning_rate * (reward + discount_rate * /*to continue*/);
      
      
      
      
     
    }      
    
    
}





