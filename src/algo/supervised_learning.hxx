#pragma once

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

/* Estimate the features (distances) using propagation model from RSSI */
template <class flight_controller_t,
	  class simulator_t>
arma::mat Supervised_learning<flight_controller_t,
		     simulator_t>::
insert_estimated_features(std::vector<Quadcopter::Action> actions)
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
  arma::rowvec row;
  
  for (int i = 0; i < 6; ++i) {
    /*  State */
    row << states_.front().height()
	<< states_.front().distances_3D().f1
	<< states_.front().distances_3D().f2
	<< states_.front().distances_3D().f3
	<< states_.front().orientation()
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

template <class flight_controller_t,
	  class simulator_t>
int Supervised_learning<flight_controller_t,
	       simulator_t>::
index_of_best_action_classification(arma::mat matrix)
{
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_max(matrix.col(0));
  }
  return value;
}

template <class flight_controller_t,
	  class simulator_t>
int Supervised_learning<flight_controller_t,
	       simulator_t>::
index_of_best_action_regression(arma::mat matrix)
{
  int value = 0;
  for (arma::uword i = 0; i < matrix.n_rows; ++i) {
    value = arma::index_min(matrix.col(0));
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
		   mlpack::ann::RandomInitialization> classification_model;

    mlpack::ann::FFN<mlpack::ann::MeanSquaredError<>,
		     mlpack::ann::RandomInitialization> regression_model;
    
    if (classification_) {  
      mlpack::data::Load("model.txt", "model", classification_model, true);
    } else if(regression_) {
      mlpack::data::Load("model.txt", "model", regression_model, true);
    }
    
  /* We need to pass State, nextState, and all possible action */
  Quadcopter::Action action_leader ;
  std::vector<std::thread> threads;
  
  if (random_leader_action == true) {
    action_leader = robot_.random_action_generator();
    saved_leader_action_ = action_leader;
  } else {
    action_leader = saved_leader_action_;
  }

  /*  Threading QuadCopter */
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("l",
								     action_leader,
								     1000);
				}));
  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f1",
								     action_leader,
								     1000);
				  
				}));

  /* We need to wait until the quadcopters finish their actions */
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  /* Get the next state at time t + 1  */
  Quadcopter::State<simulator_t> state(sim_interface_);
  states_.push_back(state);

  /*  We need to predict the action for the follower using h(S)*/
  /*  Extract state and push it into the model with several actions */
  /*  Take the action index for the highest class
      given back by the model */

  std::vector<Quadcopter::Action> possible_action  =
    robot_.possible_actions();

  /*  Test the trained model using the absolute gazebo distance feature */
  arma::mat features = insert_absolute_features(possible_action);
  arma::mat label;
  
  if (classification_) {
    classification_model.Predict(features, label);  
  } else if (regression_) {
    regression_model.Predict(features, label);
  }
  /* Log the controler prediction to improve accuracy*/
  arma::rowvec vec  = label.row(0);
  controller_predictions_ = mtools_.to_std_vector(vec);
  
  /* Transpose to the original format */
  features = features.t();
  label = label.t();

  LogInfo() << "True 3D distance: " << states_.back().distances_3D();
  LogInfo() << "Size of features: " << arma::size(features);
  LogInfo() << features;
  LogInfo() << label;

  int values = index_of_best_action_regression(label);
  LogInfo() << values;

  /*  Get the follower action now !! and store it directly */
  action_follower_.push_back(robot_.action_follower(features, values));

  threads.push_back(std::thread([&](){
				  swarm_.one_quad_execute_trajectory("f2",
								     action_follower_.back(),
								     1000);
  				   
  				}));
  
  for (auto& thread : threads) {
    thread.join();
  }

  /*  Get error of deformation to improve percision later and to
      verify the model accuracy */
  Quadcopter::State<simulator_t> nextState(sim_interface_);
  states_.push_back(nextState);

  step_errors_.push_back(mtools_.deformation_error_one_follower
			(original_dist_, nextState.distances_3D()));
  return;
}

template <class flight_controller_t,
	  class simulator_t>
void Supervised_learning<flight_controller_t, simulator_t>::
run(const Settings& settings)
{
  robot_.init();
  bool classification_ = settings.classification();
  bool regression_ = settings.regression();
  
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

    /* Stop the episode if one of the quad has fallen to arm */   
    stop_episode_ = false;
    bool arm = swarm_.arm();
    if (!arm)
      stop_episode_ = true;
    
    std::this_thread::sleep_for(std::chrono::seconds(2));

    /* Stop the episode if one of the quad has fallen to takoff */
    bool takeoff = swarm_.takeoff(5);
    if (!takeoff)
      stop_episode_ = true;    
    
    /*  Setting up speed_ is important to switch the mode */
    swarm_.init_speed();

    /*  Switch to offboard mode, Allow the control */
    bool offboard_mode = swarm_.start_offboard_mode();
    if (!offboard_mode)
      stop_episode_ = true;
    
    /*  Wait to complete the take off process */
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (!stop_episode_) {

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

	LogInfo() << "New positions : " << new_positions;
		/*  Handle fake takeoff.. */

	if (sim_interface_->positions().f1.z < 6
	    or sim_interface_->positions().f2.z < 6) {
	  stop_episode_ = true;
	} 
		
	new_triangle.push_back(mtools_.triangle_side_3D(new_positions));
	
	if (classification_) {
	  if (count_ == 0 ) {
	    reward = robot_.action_evaluator(original_dist_,
					     new_triangle.at(count_));
	    
	  } else {
	    reward = robot_.action_evaluator(new_triangle.at(count_ -1),
					     new_triangle.at(count_));
	  }
	}

	double score = -1;
	if (regression_) {
	  /*  Regression */
	  if (count_ == 0 ) {
	    score = robot_.true_score_square(original_dist_,
				     new_triangle.at(count_));	  
	  } else {
	    score = robot_.true_score_square(new_triangle.at(count_ -1),
				     new_triangle.at(count_));
	  }
	}
	
	/*  Need to verify that the controller is working,
	 use the triangle test to figure out after each iteration*/
	if (mtools_.is_triangle(mtools_.triangle_side_3D(sim_interface_->positions())) == false) {
	  LogInfo() << "The triangle is no longer conserved";
	  robot_.save_controller_count(count_);
	  break;
	}

	/* Log online dataset */	
	if (classification_) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      controller_predictions_,
				      mtools_.to_one_hot_encoding(reward, 4)
				      );
	}
	
	if (settings.regression()) {
	  data_set_.save_csv_data_set(states_.front(),
				      mtools_.to_one_hot_encoding(action_follower_.back(), 6),
				      states_.back(),
				      controller_predictions_,
				      score
				      );
	}
	controller_predictions_.clear();
	states_.clear();	
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

    if (mean_error != -1) {
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
