# include "q_learning.hh"

template <class flight_controller_t,
	  class simulator_t>
Q_learning<flight_controller_t, simulator_t>::
Q_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	   std::shared_ptr<simulator_t> gzs)
  :count_(0),
   decay_rate_(0.01),
   discount_rate_(0.95),
   distribution_(0.0, 1.0),
   distribution_int_(0, 3),
   episode_(0),
   epsilon_(1.0),
   generator_(random_dev()),
   learning_rate_(0.3),
   max_episode_(10000),  
   min_epsilon_(0.0),
   iris_x_(std::move(iris_x)),
   sim_interface_(std::move(gzs)),
   speed_(configs_.speed())
{}

template <class flight_controller_t,
	  class simulator_t>
arma::mat Q_learning<flight_controller_t,
		     simulator_t>::
features_extractor(std::vector<typename Quadcopter<simulator_t>::Action> actions)
{
  arma::mat features;  
  auto it_state = states_.rbegin();    
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 4; ++i) {    
    /*  State */
    row << (*it_state).height()
	<< (*it_state).estimated_distances().f1
	<< (*it_state).estimated_distances().f2
	<< (*it_state).estimated_distances().f3
	<< (*it_state).orientation()
      /*  Action  */
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(3)
      /*  nextState */
	<< states_.back().height()
	<< states_.back().estimated_distances().f1
	<< states_.back().estimated_distances().f2
	<< states_.back().estimated_distances().f3
	<< states_.back().orientation() ;
    
    features.insert_rows(0, row);    
  }
  /*  We need to transpose the matrix, since mlpack is column major */  
  features = features.t();
  
  return features;    
}
  
template <class flight_controller_t,
	  class simulator_t>
arma::mat Q_learning<flight_controller_t,
		     simulator_t>::
insert_absolute_features(std::vector<typename Quadcopter<simulator_t>::Action> actions)
{  
  arma::mat features;  
  auto it_state = states_.rbegin();    
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 4; ++i) {
    
    /*  State */
    row << 3 //(*it_state).height()
	<< (*it_state).distances().f1
	<< (*it_state).distances().f2
	<< (*it_state).distances().f3
	<< (*it_state).orientation()
      /*  Action  */
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 4).at(3)
      /*  nextState */
	<< 3 //states_.back().height()
	<< states_.back().distances().f1
	<< states_.back().distances().f2
	<< states_.back().distances().f3
	<< states_.back().orientation() ;
    
    features.insert_rows(0, row);    
  }
  /*  We need to transpose the matrix, since mlpack is column major */  
  features = features.t();
  
  return features;  
}

template <class flight_controller_t,
	  class simulator_t>
typename Quadcopter<simulator_t>::Action Q_learning<flight_controller_t,
						    simulator_t>::
action_follower(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  typename Quadcopter<simulator_t>::Action
    action = Quadcopter<simulator_t>::Action::NoMove;
  
  if (features(index, 5) == 1) {
    action =  Quadcopter<simulator_t>::Action::forward;
  } else if (features(index, 6) == 1) {
    action =  Quadcopter<simulator_t>::Action::backward;
  } else if (features(index, 7) == 1) {
    action =  Quadcopter<simulator_t>::Action::left;
  } else if (features(index, 8) == 1) {
    action =  Quadcopter<simulator_t>::Action::right;
  }
  return action;
}

template <class flight_controller_t,
	  class simulator_t>
int Q_learning<flight_controller_t,
	       simulator_t>::
highest_values(arma::mat matrix)   
{  
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_max(matrix.col(0));
  }
  return value;
}

template <class flight_controller_t,
	  class simulator_t>
void Q_learning<flight_controller_t, simulator_t>::
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
    iris_x_.at(quad_number)->left(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::right) {  
    iris_x_.at(quad_number)->right(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::forward) { 
    iris_x_.at(quad_number)->forward(speed_);
    
  } else if (action == Quadcopter<simulator_t>::Action::backward) { 
    iris_x_.at(quad_number)->backward(speed_);
    
  }  
}

template <class flight_controller_t,
	  class simulator_t>
void Q_learning<flight_controller_t,
		simulator_t>::
phase_two(bool random_leader_action)
{    
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> model;
  
  mlpack::data::Load("model.xml", "model", model);
  
  /*  we need to pass State ,and nextState, and try possible a */
  typename Quadcopter<simulator_t>::Action action_leader ;
  
  std::vector<std::thread> threads;
    
  /* Get the state at time t  */
  typename Quadcopter<simulator_t>::State state(sim_interface_);
  states_.push_back(state);
  
  if (random_leader_action == true) {    
    action_leader = robot_.randomize_action() ;
    saved_leader_action_ = action_leader;
  } else {
    action_leader = saved_leader_action_;    
  }
  
  /*  Threading QuadCopter */    
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action("l", action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action("f1", action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  /* We need to wait until the quadcopters finish their actions */  
  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  
  /* Get the next state at time t + 1  */
  typename Quadcopter<simulator_t>::State nextState(sim_interface_);
  states_.push_back(nextState);
    
  /*  we need to predict the action for the follower using h(S)*/

  /*  Extract state and push it into the model with several H */
  /*  take the highest value for the highest reward 
      given back by the model */
   
  std::vector<typename Quadcopter<simulator_t>::Action> possible_action  =
    robot_.possible_actions() ;

  /*  Now we need to estimate the features using fgeature extractor */
  
  //  arma::mat features = features_extractor(possible_action);

  /*  Test the trained model using the absolute feature */
  arma::mat features = insert_absolute_features(possible_action);
  
  arma::mat label;
  
  model.Predict(features, label);
  
  /* Transpose to the original format */
  
  features = features.t();
  label = label.t();

  LogInfo() << "True 2D distance: " << states_.back().distances();
  
  LogInfo() << "Size of features: " << arma::size(features);
  LogInfo() << features;
  LogInfo() << label;
    
  int values = highest_values(label);

  LogInfo() << values;
    
  /*  Get the follower action now !! */    
  typename Quadcopter<simulator_t>::Action action_for_follower =
    action_follower(features, values);
  
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action("f2", action_for_follower);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  for (auto& thread : threads) {
    thread.join();
  }


  /*  Get error of deformation to improve persicion later and to
      verify the model accuracy */
  typename Quadcopter<simulator_t>::State ObserverState(sim_interface_);
  robot_.calculate_save_error(original_dist_, ObserverState.distances());        
  
  return;    
}

template <class flight_controller_t,
	  class simulator_t>
void Q_learning<flight_controller_t, simulator_t>::
run()
{

  robot_.init();
  
  typename Quadcopter<simulator_t>::State ObserverState(sim_interface_);
  original_dist_ = ObserverState.distances();

  for (episode_ = 0; episode_ < max_episode_; ++episode_) {
    
    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads: 
     * Arm + takeoff + offboard mode + moving + land 
     * Then: repeat each episode.
     */

    LogInfo() << "Episode : " << episode_ ;            
    
    /*  Think How we can use threads here */

    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool arm;
    bool stop_episode = false;    
    for (auto it : iris_x_){
      arm = it->arm();
      if(!arm)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    /* Stop the episode if one of the quad has fallen to takoff */
    /*  Replace it by a template function  */
    bool takeoff;
    for (auto it : iris_x_) {
      takeoff = it->takeoff();
      if(!takeoff)
	stop_episode = true;
    }
    
    std::this_thread::sleep_for(std::chrono::seconds(3));
    /*  Setting up speed_ is important to switch the mode */
    for (auto it : iris_x_) {
      it->init_speed();
    }
    
    /*  Switch to offboard mode, Allow the control */
    for (auto it : iris_x_) {
      it->start_offboard_mode();
    }
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));  
    
  
    if (!stop_episode) {
      
      count_ = 0 ;
      
      while (count_ < 150) {
	
	if (count_ == 0 ) {
	  phase_two(true);	  
	} else {
	  phase_two(false);      
	}
	
	std::this_thread::sleep_for(std::chrono::seconds(1));

	/*  Need to verify that the controller is working, 
	 use the triangle test to figure out after each iteration*/
	if (mtools_.is_triangle(mtools_.triangle_side(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  robot_.save_controller_count(count_);
	  break;
	}
	
	//reduce epsilon as we explore more each episode
	//epsilon_ = min_epsilon_ + (0.5 - min_epsilon_) * std::exp( -decay_rate_/5 * episode_); 
	
	++count_;
	
      }
    }
        
    for (auto it: iris_x_)
      it->land();
    
    std::this_thread::sleep_for(std::chrono::seconds(6));
    
    sim_interface_->reset_models();
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}
