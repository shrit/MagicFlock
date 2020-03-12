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

template<class QuadrotorType>
class ActionGenerator : public Actions
{

public:
  ActionGenerator(typename std::vector<QuadrotorType>::iterator quad);

  Action generate_random_action();

  Action generate_persistant_action(int for_n_time_step, int timesteps);

  Action generate_random_action_no_opposed(Action action);

  Action generate_random_action_no_opposed_no_same(Action action);

  Action generate_action_from_oracle();

private:
  typename std::vector<QuadrotorType>::iterator quad_;
};

#include "action_generator.hxx"
