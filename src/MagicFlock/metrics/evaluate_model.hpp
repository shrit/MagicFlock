#pragma once

#include <MagicFlock/actions/discret_actions.hpp>
#include <MagicFlock/data/dataset.hpp>

/* Standard library includes*/
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

class EvaluateModel
{

public:
  EvaluateModel();

  void input(DiscretActions leader_action,
             DiscretActions follower_1_action,
             DiscretActions follower_2_action);

  Evaluation evaluate();
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
  DataSet dataset_;
};

