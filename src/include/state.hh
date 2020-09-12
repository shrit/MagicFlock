#pragma once
/*
 * @author: Omar Shrit
 *
 * This file implement the state of a quadrotor
 *
 */

#include <memory>

/*  Local includes */
#include "arma_helper.hh"

template<class FilterType, class ContainerType>
class State
{

public:
  State();

  State(const arma::colvec& data);

  State(unsigned int id,
        const ContainerType& container,
        FilterType noise);

  State(unsigned int id,
        const ContainerType& container);

  arma::colvec Data() const;

  arma::colvec& Data();

  arma::colvec RSSI() const;

  arma::colvec& RSSI();

  arma::colvec TOAs() const;
  
  arma::colvec& TOAs();

  const arma::colvec& Encode() const;

  static constexpr size_t dimension = 3;

private:
  int id_;
  ArmaHelper arma;
  arma::colvec data_;
  arma::colvec rssi_data_;  
  arma::colvec toa_data_;
};

#include "state.hpp"
