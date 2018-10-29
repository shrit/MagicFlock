# include "q_learning.hh"

Q_learning::Q_learning()
  :matrix_{3,3},
     rewards_{}

{

  /* Initlize the matrix for the algorithm */
  
  for (unsigned i = 0; i < m.size1 (); ++ i)
    for (unsigned j = 0; j < m.size2 (); ++ j)
      m (i, j) = 0;
  std::cout << m << std::endl;
      
  //   for (auto episode : q_values::max_episode){
  //     std::cout << "Episode" << episode << std::endl;

  //  }
  
}

void Q_learning::move_quads_followers_action(Controllers controllers, int action)
{
  switch(action){

  case 0:
    controllers.at(1).goLeft(offboard_);
    break;
  case 1:
    controllers.at(1).goRight(offboard_);
    break;
  case 2:
    controllers.at(1).forward(offboard_); 
    break;
  case 3:
    controllers.at(1).backward(offboard_);
    break;
  default:
    continue;
  }  
}

void Q_learning::run_episods(Controllers controllers)
{

  gazebo::msg::Vector3d states;

  
  for (auto episode : q_values::max_episode){
    
    std::cout << "Episode : " << episode << std::endl;   
          
       // declare position to get signal values

    gazebo_.get_quads_positions();
    gazebo_.get_quads_rssi();

    // put the recoved signls in state tables 
      
       for(auto steps : q_values::max_step){
	 controllers.[0].forward(offboard_); // reiterate to move at
					    // least 10 centometers,
					    // or find another
					    // solution

	   
	   //get the new signal of strength difference
	   
	 states = gazebo_.get_quads_rssi();
	 in random = rand() % 10;


	 int action1;
	 int action2;
	 if(random > q_values::epsilon){
	    action1 = matrix(); // see with ublas how to get the highest values
	    action2 = matrix();
	 }
	 else {
	   action1 = rand() % 4;
	   action2 = action1;
	 }
	 move_quads_followers_action(controllers, action1);
	   }      



  
}





