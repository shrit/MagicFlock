/*
 * @author: Omar Shrit
 *
 * This file implement the actions of a quadrotor
 *
 *
 */
#pragma once

#include <random>
#include <utility>

#include "action.hh"
#include "quadrotor.hh"

template<class simulator_t>
class ActionGenerator : public Actions
{

public:
  ActionGenerator(typename std::vector<Quadrotor<simulator_t>>::iterator quad);

  Action generate_random_action();

  Action generate_persistant_action(int n_time_step);

  Action generate_random_action_no_opposed(Action action);

  Action generate_random_action_no_opposed_no_same(Action action);

  Action generate_action_from_oracle();

private:
  typename std::vector<Quadrotor<simulator_t>>::iterator quad_;
};

#include "action_generator.hxx"
