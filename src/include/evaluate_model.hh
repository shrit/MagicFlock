#pragma once

#include "action.hh"
#include "data_set.hh"
#include <vector>

struct Evaluation
{
  double percent_same_action;
  double percent_same_b_a;
  double percent_same_c_a;
  double percent_not_same_b_a;
  double percent_not_same_c_a;
  double percent_both_not_same;
  double total_count;
};

template<class simulator_t>
class EvaluateModel
{

public:
  EvaluateModel();

  void input(Actions::Action leader_action,
             Actions::Action follower_1_action,
             Actions::Action follower_2_action);

  std::tuple<int, int> evaluate_not_similar_actions(
    Actions::Action leader_action,
    Actions::Action follower_1_action);

  Evaluation output();
  void register_evaluation();

private:
  int count_both_actions_;
  int count_follower_1_;
  int count_follower_2_;
  int count_bad_action_f1_;
  int count_bad_action_f2_;
  int count_both_bad_actions_;
  double global_count_;
  Evaluation evaluate_;
  DataSet<simulator_t> dataset_;
};

#include "evaluate_model.hxx"
