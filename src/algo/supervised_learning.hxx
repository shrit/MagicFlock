# include "supervised_learning.hh"

template <class flight_controller_t,
	  class simulator_t>
Supervised_learning<flight_controller_t, simulator_t>::
Supervised_learning(std::vector<std::shared_ptr<flight_controller_t>> iris_x,
	   std::shared_ptr<simulator_t> gzs)
  :count_(0),
   episode_(0),
   max_episode_(10000),
   sim_interface_(std::move(gzs)),
   swarm_(std::move(iris_x))
{
  data_set_.init_dataset_directory();
}

template <class flight_controller_t,
	  class simulator_t>
arma::mat Supervised_learning<flight_controller_t,
		     simulator_t>::
features_extractor(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  auto it_state = states_.rbegin();
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 6; ++i) {
    /*  State */
    row << (*it_state).height()
	<< (*it_state).estimated_distances().f1
	<< (*it_state).estimated_distances().f2
	<< (*it_state).estimated_distances().f3
	<< (*it_state).orientation()
      /*  Action  */
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(3)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(4)
      	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(5)
      /*  nextState */
	<< states_.back().height()
	<< states_.back().estimated_distances().f1
	<< states_.back().estimated_distances().f2
	<< states_.back().estimated_distances().f3
	<< states_.back().orientation();

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
insert_absolute_features(std::vector<Quadcopter::Action> actions)
{
  arma::mat features;
  auto it_state = states_.rbegin();
  it_state = std::next(it_state, 1);

  arma::rowvec row;
  for (int i = 0; i < 6; ++i) {

    /*  State */
    row << (*it_state).height()
	<< (*it_state).distances_3D().f1
	<< (*it_state).distances_3D().f2
	<< (*it_state).distances_3D().f3
	<< (*it_state).orientation()
      /*  Action encoded as 1, and 0, add 6 times to represent 6 actions */
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(0)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(1)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(2)
	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(3)
      	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(4)
      	<< mtools_.to_one_hot_encoding(actions.at(i), 6).at(5)
      /*  nextState */
	<< states_.back().height()
	<< states_.back().distances_3D().f1
	<< states_.back().distances_3D().f2
	<< states_.back().distances_3D().f3
	<< states_.back().orientation();
    /*  Create a matrix of several rows, each one is added to on the top */
    features.insert_rows(0, row);
  }
  /*  We need to transpose the matrix, since mlpack is column major */
  features = features.t();
  /*  The return features need to be used in the model in order to
      give back the best action with highest score */
  return features;
}

/* Get the best action from the model according to the best values */
template <class flight_controller_t,
	  class simulator_t>
Quadcopter::Action Supervised_learning<flight_controller_t,
				       simulator_t>::
action_follower(arma::mat features, arma::uword index)
{
  /*  just a HACK, need to find a dynamic solution later */
  Quadcopter::Action action = Quadcopter::Action::NoMove;
  /*  Access matrix values according to a given index  */
  /*  Only one action exist that equal 1 in each row of 
   a matrix */
  
  if (features(index, 5) == 1) {
    action =  Quadcopter::Action::forward;
  } else if (features(index, 6) == 1) {
    action =  Quadcopter::Action::backward;
  } else if (features(index, 7) == 1) {
    action =  Quadcopter::Action::left;
  } else if (features(index, 8) == 1) {
    action =  Quadcopter::Action::right;
  } else if (features(index, 9) == 1) {
    action =  Quadcopter::Action::up;
  } else if (features(index, 10) == 1) {
    action =  Quadcopter::Action::down;
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
void Supervised_learning<flight_controller_t,
		simulator_t>::
phase_two(bool random_leader_action)
{
  mlpack::ann::FFN<mlpack::ann::SigmoidCrossEntropyError<>,
		   mlpack::ann::RandomInitialization> model;

  mlpack::data::Load("model.txt", "model", model, true);

  /*  we need to pass State ,and nextState, and try possible a */
  Quadcopter::Action action_leader ;

  std::vector<std::thread> threads;

  /* Get the state at time t  */
  Quadcopter::State<simulator_t> state(sim_interface_);
  states_.push_back(state);

  if (random_leader_action == true) {
    action_leader = robot_.random_action_generator() ;
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
				    swarm_.one_quad_execute_trajectory("l", action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }
				}));
  threads.push_back(std::thread([&](){
				  for (int i = 0; i < 4; ++i) {
				    swarm_.one_quad_execute_trajectory("f1", action_leader);
				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
				  }
				}));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  /* Get the next state at time t + 1  */
  Quadcopter::State<simulator_t> nextState(sim_interface_);
  states_.push_back(nextState);

  /*  we need to predict the action for the follower using h(S)*/

  /*  Extract state and push it into the model with several H */
  /*  take the highest value for the highest reward
      given back by the model */

  std::vector<Quadcopter::Action> possible_action  =
    robot_.possible_actions();

  /*  Now we need to estimate the features using propagation model */

  //  arma::mat features = features_extractor(possible_action);

  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = insert_absolute_features(possible_action);

  arma::mat label;

  model.Predict(features, label);

  /* Transpose to the original format */

  features = features.t();
  label = label.t();

  LogInfo() << "True 3D distance: " << states_.back().distances_3D();

  LogInfo() << "Size of features: " << arma::size(features);
  LogInfo() << features;
  LogInfo() << label;

  int values = highest_values(label);

  LogInfo() << values;

  /*  Get the follower action now !! and store it directly */
  action_follower_.push_back(action_follower(features, values));

  threads.push_back(std::thread([&](){
  				  for (int i = 0; i < 4; ++i) {
  				    swarm_.one_quad_execute_trajectory("f2", action_follower_.back());
  				    std::this_thread::sleep_for(std::chrono::milliseconds(35));
  				  }
  				}));
  
  for (auto& thread : threads) {
    thread.join();
  }

  /*  Get error of deformation to improve persicion later and to
      verify the model accuracy */
  Quadcopter::State<simulator_t> ObserverState(sim_interface_);
  states_.push_back(ObserverState);

  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, ObserverState.distances_3D()));

  return;
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
run()
{

  robot_.init();

  Quadcopter::State<simulator_t> ObserverState(sim_interface_);
  original_dist_ = ObserverState.distances_3D();

  for (episode_ = 0; episode_ < max_episode_; ++episode_) {

    /* Intilization phase, in each episode we should reset the
     * position of each quadcopter to the initial position.
     * From here we can start automatically the quads:
     * Arm + takeoff + offboard mode + moving + land
     * Then: repeat each episode.
     */

    LogInfo() << "Episode : " << episode_ ;

    /*  Think How we can use threads here */

    /* Stop the episode if one of the quad has fallen to arm */   
    bool stop_episode = false;
    bool arm = swarm_.arm();
    if(!arm)
      stop_episode = true;
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(5);
    if (!takeoff)
      stop_episode = true;    
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed();

    /*  Switch to offboard mode, Allow the control */
    bool offboard_mode = swarm_.start_offboard_mode();
    if(!offboard_mode)
      stop_episode = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode) {

      count_ = 0;

      std::vector<lt::triangle<double>> new_triangle;

      while (count_ < 300) {

	/*  Do online learning... */
        Quadcopter::Reward reward =
	  Quadcopter::Reward::very_bad;

	if (count_ == 0 ) {
	  phase_two(true);
	  //Change each 10 times the direction of the leader
	} else if (count_ % 10 == 0) {
	  phase_two(true);
	} else {
	  phase_two(false);
	}

	lt::positions<lt::position3D<double>> new_positions = sim_interface_->positions();

	LogInfo() << "New positions : " << new_positions ;

	new_triangle.push_back(mtools_.triangle_side_3D(new_positions));

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

	if (mtools_.is_triangle(mtools_.triangle_side_3D(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  robot_.save_controller_count(count_);
	  break;
	}

	/* Log online dataset */
	auto it_state = states_.rbegin();
	it_state = std::next(it_state, 1);

	/*  Save the generated data during the testing to improve the model later*/
	Quadcopter::State<simulator_t> sp(sim_interface_);

	data_set_.save_csv_data_set(sp.create_printer_struct(*it_state),
				    mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				    sp.create_printer_struct(states_.back()),
				    mtools_.to_one_hot_encoding(reward, 4)
				    );
	time_step_vector_.push_back(count_);

	/* Check why we need this sleep ! ??*/
	std::this_thread::sleep_for(std::chrono::seconds(1));
	++count_;
      }
    }
    /*  Add 1 to adjust count to one instead of starting by zero */
    count_ = count_ + 1;
    /*  Save a version of the time steps to create a histogram */
    mtools_.histogram(count_);
    data_set_.save_histogram(mtools_.get_histogram<int>());

    /*  Get the flight error as the mean of the step error */
    double mean_error = mtools_.mean(step_errors_);

    if(mean_error != -1) {
      data_set_.save_error_file(mean_error);
      flight_errors_.push_back(mean_error);
    }

    step_errors_.clear();

    swarm_.land();
    
    /* Resetting the entire swarm after the end of each episode*/
    sim_interface_->reset_models();
    
    LogInfo() << "The quadcopters have been resetted...";
    
    std::this_thread::sleep_for(std::chrono::seconds(15));
  }
}
