#pragma once

/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
*/

/*  Local includes */
# include "global.hh"
# include "math_tools.hh"

namespace lt = local_types;

template <class simulator_t>
class State {

public:

  State();
  State(std::shared_ptr<simulator_t> sim_interface,
	unsigned int id,
	std::vector<unsigned int> nearest_neighbors);

  State(std::vector<double> distances, double altitude_diff);

  double height_difference() const;
  double rt_height_difference();
  double rt_height();

  std::vector<double> distances_3D() const;
  std::vector<double> estimated_distances() const;

  std::vector<State> StateConstructor(arma::mat values);

private:

  double altitude_;
  double alti_diff_;
  Math_tools mtools_;
  std::vector<double> dists_3D_;
  std::vector<double> estimated_dists_3D_;
  /*  Create a shared pointer to a simulator interface The interface
      need to have all the required data about the quadcopter*/
  std::shared_ptr<simulator_t> sim_interface_;

  std::vector<lt::position3D<double>>::iterator leader_;
  std::vector<lt::position3D<double>>::iterator follower_;
};

# include "state.hxx"
