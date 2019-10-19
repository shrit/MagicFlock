/*
 * @author: Omar Shrit
 *
 * This file implement the state class for quadcopters
 *
 *
*/
#pragma once

# include <random>
# include <utility>

/*  Armadillo includes  */
# include <armadillo>

# include "../data_set.hh"
# include "../global.hh"
# include "../math_tools.hh"
# include "propagation_model.hh"

namespace lt = local_types;

class Quadcopter {

public:

  Quadcopter();
  
  template <class simulator_t>
  class State {

  public:

    State(std::shared_ptr<simulator_t> sim_interface);
    double height_f1 () const;
    double height_f2 () const;
    double height_difference() const;
    lt::rssi<double> signal_strength () const;
    lt::triangle<double> distances_2D () const;
    lt::triangle<double> distances_3D () const;
    lt::triangle<double> estimated_distances () const;
    double orientation () const;

  private:

    Math_tools mtools_;
    lt::rssi<double> rssi_ ;
    double height_f2_;
    double height_f1_;
    double z_orinetation_  ;
    lt::triangle<double> dists_2D_ ;
    lt::triangle<double> dists_3D_ ;
    lt::triangle<double> e_dists_;
    /*  Create a shared pointer to a simulator interface The interface
	need to have all the required data about the quadcopter*/
    std::shared_ptr<simulator_t> sim_interface_;
    Propagation_model<simulator_t, double> pmodel_;
  };

  enum class Action
    {
     forward,
     backward,
     left,
     right,
     up,
     down,
     NoMove,
     Unknown,
    };

  enum class Reward
    {
     very_good,
     good,
     bad,
     very_bad,
     Unknown,
    };
  
  void init();

  Reward action_evaluator(const lt::triangle<double>& old_dist,
			  const lt::triangle<double>& new_dist);
        
  Action
  action_follower(arma::mat features, arma::uword index);
  
  double true_score(const lt::triangle<double>& old_dist,
		    const lt::triangle<double>& new_dist);
  
  double true_score_log(const lt::triangle<double>& old_dist,
			const lt::triangle<double>& new_dist);
  
  double true_score_square(const lt::triangle<double>& old_dist,
			   const lt::triangle<double>& new_dist);
  
  double true_score_square_log(const lt::triangle<double>& old_dist,
			       const lt::triangle<double>& new_dist);

  int evaluation_score(const lt::triangle<double>& old_dist,
		       const lt::triangle<double>& new_dist,
		       const double altitude,
		       const double old_altitude);
  void
  save_controller_count(double value);

  std::vector<Action> possible_actions() const;

  Action random_action_generator();

  Action 
  random_action_generator_with_only_opposed_condition(Action action);

  Action
  random_action_generator_with_all_conditions(Action action);

private:

  std::vector<Action> possible_actions_ = { Action::forward,
					    Action::backward,
					    Action::left,
					    Action::right,
					    Action::up,
					    Action::down};
  
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;
  std::mt19937 generator_;
  DataSet data_set_;

};

# include "quadcopter.hxx"
