#pragma once

template<class simulator_t>
EvaluateModel<simulator_t>::EvaluateModel<simulator_t>()
  : count_both_actions_(0)
  , count_follower_1_(0)
  , count_follower_2_(0)
  , count_bad_action_f1_(0)
  , count_bad_action_f2_(0)
  , count_both_bad_actions_(0)
  , global_count_(0.0)
{}

template<class simulator_t>
void
EvaluateModel<simulator_t>::input(Actions::Action leader_action,
                                  Actions::Action follower_1_action,
                                  Actions::Action follower_2_action)
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

template<class simulator_t>
Evaluation
EvaluateModel<simulator_t>::output()
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

template<class simulator_t>
void
EvaluateModel<simulator_t>::register_evaluation()
{
  dataset_.save_evaluation(*this);
}

template<class simulator_t>
std::ostream&
operator<<(std::ostream& out, EvaluateModel<simulator_t>& m_evaluate)
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
      << "Percentage of timesteps that all of quadrotors in the swarm execute "
         "the same action:"
      << m_evaluate.output().percent_same_action << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Bob and Alice quadrotors execute the "
         "same action:     "
      << m_evaluate.output().percent_same_b_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Charlie and Alice quadrotors execute "
         "the same action: "
      << m_evaluate.output().percent_same_c_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Bob and Alice execute different action: "
         "              "
      << m_evaluate.output().percent_not_same_b_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that Chalrie and Alice execute different "
         "action:           "
      << m_evaluate.output().percent_not_same_b_a << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Percentage of timesteps that all of them execute different actions   "
         "            "
      << m_evaluate.output().percent_both_not_same << std::endl
      << "---------------------------------------------------------------------"
         "--------------------------------"
      << std::endl
      << "Number of totat timesteps executed from the start of the "
         "simualtion:              "
      << m_evaluate.output().total_count << std::endl
      << "====================================================================="
         "================================"
      << std::endl;

  return out;
}
