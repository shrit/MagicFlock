# include "supervised_learning.hh"

template <class flight_controller_t,
	  class simulator_t>
Supervised_learning<flight_controller_t, simulator_t>::
Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
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
{
  data_set_.init_dataset_directory();
}

template <class flight_controller_t,
	  class simulator_t>
arma::mat Supervised_learning<flight_controller_t,
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
arma::mat Supervised_learning<flight_controller_t,
		     simulator_t>::
insert_absolute_features(std::vector<typename Quadcopter<simulator_t>::Action> actions)
{  
  arma::mat features;  
  auto it_state = states_.rbegin();    
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 4; ++i) {
    
    /*  State */
    row << (*it_state).height()
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
	<< states_.back().height()
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
typename Quadcopter<simulator_t>::Action Supervised_learning<flight_controller_t,
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
int Supervised_learning<flight_controller_t,
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
void Supervised_learning<flight_controller_t, simulator_t>::
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
void Supervised_learning<flight_controller_t,
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

  LogInfo() << "Z height: "<< sim_interface_->positions().f1.z;
  
  if (sim_interface_->positions().f1.z < 6 or
      sim_interface_->positions().f2.z < 6 ) {    
    return ;
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
    
  /*  Get the follower action now !! and store it direclty */    
  action_follower_.push_back(action_follower(features, values));
  
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    move_action("f2", action_follower_.back());
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }				  
				}));
  
  for (auto& thread : threads) {
    thread.join();
  }
  
  /*  Get error of deformation to improve persicion later and to
      verify the model accuracy */
  typename Quadcopter<simulator_t>::State ObserverState(sim_interface_);
  states_.push_back(ObserverState);
  
  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, ObserverState.distances()));        
  
  return;    
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
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

    /*  Increase the heights of the drones to decrease the change of
	touching the ground */
    
    for(int i = 0; i < 10; ++i) {
      for (auto it : iris_x_) {
	it->up(10);
      }
    }
  
    if (!stop_episode) {
      
      count_ = 0 ;

      std::vector<lt::triangle<double>> new_triangle;
      
      while (count_ < 150) {

	/*  Do online learning... */
        typename Quadcopter<simulator_t>::Reward reward =
	  Quadcopter<simulator_t>::Reward::very_bad;		
	
	if (count_ == 0 ) {
	  phase_two(true);
	  /*  Change each 10 times the direction of the leader */		  
	} else if (count_ % 10 == 0) {
	  phase_two(true);      
	} else {
	  phase_two(false);      
	}
       	
	lt::positions<double> new_positions = sim_interface_->positions();
	
	LogInfo() << "New positions : " << new_positions ;
	
	new_triangle.push_back(mtools_.triangle_side(new_positions));
				
	if (count_ == 0 ) {
	  reward = robot_.action_evaluator(original_dist_,
					  new_triangle.at(count_));
	  	
	} else  {
	  reward = robot_.action_evaluator(new_triangle.at(count_ -1),
					  new_triangle.at(count_));        
	}
	
	LogInfo() << "Z height: "<< new_positions.f1.z;
	
	if (new_positions.f1.z < 6 or
	    new_positions.f2.z < 6 ) {	  

	  robot_.save_controller_count(count_);
	  break;
	}
	
	/*  Need to verify that the controller is working, 
	 use the triangle test to figure out after each iteration*/
	
	if (mtools_.is_triangle(mtools_.triangle_side(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  robot_.save_controller_count(count_);
	  break;
	}

	/* Log online dataset */
	auto it_state = states_.rbegin();
	it_state = std::next(it_state, 1);
	
	/*  Save the generated data during the testing to improve the model later*/
	typename Quadcopter<simulator_t>::State sp(sim_interface_);
	
	data_set_.save_csv_data_set(sp.create_printer_struct(*it_state),
				    mtools_.to_one_hot_encoding(action_follower_.back(), 4),
				    sp.create_printer_struct(states_.back()),
				    mtools_.to_one_hot_encoding(reward, 4)
				    );

	/*  Get the fligtht error as the mean of the step error */
	double mean_error = std::accumulate(step_errors_.begin() , step_errors_.end(),
					    0.0)/step_errors_.size();
	flight_errors_.push_back(mean_error);
	step_errors_.clear();
	
	/* Check why we need this sleep ! ??*/	
	std::this_thread::sleep_for(std::chrono::seconds(1));       
	++count_;	
      }
    }
        
    for (auto it: iris_x_)
      it->land();
    
    std::this_thread::sleep_for(std::chrono::seconds(8));
    
    sim_interface_->reset_models();
    
    std::this_thread::sleep_for(std::chrono::seconds(17));
  }
}
