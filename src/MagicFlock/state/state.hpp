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
#include <MagicFlock/util/arma_helper.hpp>

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
        ContainerType container,
        FilterType filter);

  arma::colvec Data() const;

  arma::colvec& Data();

  arma::colvec leader_data() const;

  arma::colvec& leader_data();

  arma::colvec followers_data() const;

  arma::colvec& followers_data();

  arma::colvec ReducedData() const;

  arma::colvec& ReducedData();

  // Need to find a solution for this one
  static constexpr size_t dimension = 18;

private:
  unsigned int id_;
  int num_neighbors_;
  ArmaHelper arma;
  arma::colvec data_;
  arma::colvec leader_data_, followers_data_;
  arma::colvec reduced_data_;
};

#include "state_impl.hpp"
