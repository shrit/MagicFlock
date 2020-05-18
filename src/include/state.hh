#pragma once
/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
 */

#include <memory>

/*  Local includes */
#include "math_tools.hh"

template<class FilterType, class ContainerType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  State(unsigned int id,
        ContainerType nearest_neighbors,
        FilterType noise);

  State(unsigned int id,
        ContainerType nearest_neighbors);

  arma::colvec Data();

  arma::colvec& Data();

  arma::colvec RSSI() const;

  arma::colvec& RSSI();

  arma::colvec TOAs() const;
  
  arma::colvec& TOAs();

  const arma::colvec& Encode() const;

  static constexpr size_t dimension = 3;

private:
  int id_;
  Math_tools mtools_;

  arma::colvec data_;
  arma::colvec rssi_data_;  
  arma::colvec toa_data_;
};

#include "state.hxx"
