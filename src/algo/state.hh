#pragma once

/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
*/

/*  Local includes */
# include "../global.hh"
# include "../math_tools.hh"
# include "propagation_model.hh"

namespace lt = local_types;

template <class simulator_t>
class State {

public:

  State(std::shared_ptr<simulator_t> sim_interface,
	unsigned int id,
	std::vector<unsigned int> nearest_neighbors);
  double height_f1() const;
  double height_f2() const;
  double height_difference() const;
  lt::rssi<double> signal_strength() const;
  lt::triangle<double> distances_2D() const;
  lt::triangle<double> distances_3D() const;
  lt::triangle<double> estimated_distances() const;
  double orientation() const;

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

# include "state.hxx"
