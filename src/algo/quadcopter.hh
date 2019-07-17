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

# include "../data_set.hh"
# include "../global.hh"
# include "../math_tools.hh"
# include "propagation_model.hh"


namespace lt = local_types;

struct state_printer {
  lt::rssi<double> rssi   ;
  double  height   ;
  lt::triangle<double>  distances  ;
  double  orientation   ;
};

inline std::ostream& operator<< (std::ostream& out, const state_printer& s)
{
  // << s.rssi <<","
  out << s.height << "," 
      << s.distances << ","
      << s.orientation ;  
  return out;
}

template <class simulator_t>
class Quadcopter {

public:

  Quadcopter();
  
  class State {
        
  public:
    
    State(std::shared_ptr<simulator_t> sim_interface);
    
    double height () const;
    lt::rssi<double> signal_strength () const;
    lt::triangle<double> distances () const;
    lt::triangle<double> estimated_distances () const;
    double orientation () const;

    state_printer
    create_printer_struct(Quadcopter<simulator_t>::State state);
          
  private:
    
    Math_tools mtools_;
    lt::rssi<double> rssi_ ;
    double height_;
    double z_orinetation_  ;
    lt::triangle<double> dists_ ;
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
     NoMove,
    };

  enum class Reward
    {
     very_good,
     good,
     bad,
     very_bad,     
    };

  void init();
  
  Reward action_evaluator(lt::triangle<double> old_dist,
			  lt::triangle<double> new_dist);

  void
  calculate_save_error(lt::triangle<double> old_dist,
		       lt::triangle<double> new_dist);

  void 
  save_controller_count(double value);
    
  std::vector<Action> possible_actions() const;
  
  Action randomize_action();
    
private:
  
  std::vector<Action> possible_actions_ = { Action::forward,
					    Action::backward,
					    Action::left,   					   
					    Action::right };
  
  std::uniform_real_distribution<> distribution_;
  std::uniform_int_distribution<> distribution_int_;
  std::random_device random_dev;  
  std::mt19937 generator_;
  DataSet data_set_;
   
};

# include "quadcopter.hxx"
