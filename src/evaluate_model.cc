#pragma once

EvaluateModel::EvaluateModel()
  : count_both_actions_(0)
  , count_follower_1_(0)
  , count_follower_2_(0)
  , count_bad_action_f1_(0)
  , count_bad_action_f2_(0)
  , count_both_bad_actions_(0)
  , global_count_(0.0)
{
  dataset_.init_dataset_directory();
}

void
EvaluateModel::input(DiscretActions::Action leader_action,
                     DiscretActions::Action follower_1_action,
                     DiscretActions::Action follower_2_action)
{
  global_count_++;

  if (leader_action == follower_1_action) {
    count_follower_1_++;
  }

  if (leader_action == follower_2_action) {
    count_follower_2_++;
  }

  if (leader_action == follower_1_action and
      leader_action == follower_2_action) {
    count_both_actions_++;
  }

  if (leader_action != follower_1_action) {
    count_bad_action_f1_++;
  }

  if (leader_action != follower_2_action) {
    count_bad_action_f2_++;
  }

  if (leader_action != follower_1_action and
      leader_action != follower_2_action) {
    count_both_bad_actions_++;
  }
}

Evaluation
EvaluateModel::evaluate()
{
  if (global_count_ != 0) {
    evaluate_.percent_same_action = count_both_actions_ / global_count_;
    evaluate_.percent_same_c_a = count_follower_1_ / global_count_;
    evaluate_.percent_same_b_a = count_follower_2_ / global_count_;
    evaluate_.percent_not_same_c_a = count_bad_action_f1_ / global_count_;
    evaluate_.percent_not_same_b_a = count_bad_action_f2_ / global_count_;
    evaluate_.percent_both_not_same = count_both_bad_actions_ / global_count_;
    evaluate_.total_count = global_count_;
  }
  return evaluate_;
}

void
EvaluateModel::register_evaluation()
{
  evaluate();
  dataset_.save_evaluation(*this);
}

std::ostream&
operator<<(std::ostream& out, EvaluateModel& m_evaluate)
{
  out << "====================================================================="
         "================================"
      << std::endl
      << "                                            Real time model "
         "Evaluation:        "
      << std::endl
      << "====================================================================="
         "================================"
      << std::endl
      << "Percentage of timesteps that all of quadrotors execute "
         "the same action:"
      << m_evaluate.evaluate().percent_same_action << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Bob and Alice quadrotors execute the "
         "same action:     "
      << m_evaluate.evaluate().percent_same_b_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Charlie and Alice quadrotors execute "
         "the same action: "
      << m_evaluate.evaluate().percent_same_c_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Bob and Alice execute different action: "
         "              "
      << m_evaluate.evaluate().percent_not_same_b_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Chalrie and Alice execute different "
         "action:           "
      << m_evaluate.evaluate().percent_not_same_c_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that all of them execute different actions   "
         "                "
      << m_evaluate.evaluate().percent_both_not_same << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Number of total timesteps executed from the start of the "
         "simulation:              "
      << m_evaluate.evaluate().total_count << std::endl
      << "====================================================================="
         "================================"
      << std::endl;

  return out;
}
