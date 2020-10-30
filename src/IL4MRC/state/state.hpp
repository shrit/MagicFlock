#pragma once
/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
 */

#include <memory>
#include <type_traits>

/*  Local includes */
#include <IL4MRC/util/arma_helper.hpp>

#include "state_types.hpp"

template<class FilterType,
         class NoiseType,
         class StateType,
         class ContainerType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  State(unsigned int id,
        int num_neighbors,
        int num_of_antenna_src,
        const ContainerType& container,
        FilterType filter);

  arma::colvec Data() const;

  arma::colvec& Data();

  arma::colvec RSSI() const;

  arma::colvec& RSSI();

  arma::colvec TOAs() const;

  arma::colvec& TOAs();

  const arma::colvec& Encode() const;

  // Need to find a solution for this one
  static constexpr size_t dimension = 14;

private:
  unsigned int id_;
  int num_neighbors_;
  ArmaHelper arma;
  arma::colvec data_;
  arma::colvec rssi_data_;
  arma::colvec toa_data_;
};

#include "state_impl.hpp"
