#pragma once

#include "discret_actions.hh"
#include "dataset.hh"
#include "gazebo.hh"
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

template
class EvaluateModel
{

public:
  EvaluateModel();

  void input(DiscretActions::Action leader_action,
             DiscretActions::Action follower_1_action,
             DiscretActions::Action follower_2_action);

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

