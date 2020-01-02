# include "include/evaluate_model.hh"

EvaluateModel::EvaluateModel()
	:count_both_actions_(0),
	 count_follower_1_(0),
	 count_follower_2_(0),
	 count_bad_action_f1_(0),
	 count_bad_action_f2_(0),
	 count_not_bad_f1_(0),
	 count_not_bad_f2_(0),
	 global_count_(0.0)
{}

void EvaluateModel::input(Actions::Action leader_action, 
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
		int not_bad_f1;
		int bad_f1;
		std::tie(not_bad_f1, bad_f1) = evaluate_not_similar_actions(leader_action, follower_1_action);
		count_not_bad_f1_ = count_not_bad_f1_ + not_bad_f1;
		count_bad_action_f1_ = count_bad_action_f1_ + bad_f1;
	} 

	if (leader_action != follower_2_action) {
		int not_bad_f2;
		int bad_f2;
		std::tie(not_bad_f2, bad_f2) = evaluate_not_similar_actions(leader_action, follower_2_action);
		count_not_bad_f2_ = count_not_bad_f2_ + not_bad_f2;
		count_bad_action_f2_ = count_bad_action_f2_ + bad_f2;
		}
}

std::tuple<int, int> EvaluateModel::
evaluate_not_similar_actions(Actions::Action leader_action, 
														 Actions::Action follower_action)
{
	int count_not_bad = 0;
	int count_bad_action = 0;
	if ((leader_action == Actions::Action::forward and
	    follower_action == Actions::Action::left)
	    or  (leader_action == Actions::Action::left and
	    follower_action == Actions::Action::forward)) {
			count_not_bad++;
	} else if ((leader_action == Actions::Action::backward and
	    follower_action == Actions::Action::right)
	    or  (leader_action == Actions::Action::right and
	    follower_action == Actions::Action::backward)) {
			count_not_bad++;
	}	else {
		count_bad_action++;
	}
	return std::make_tuple(count_not_bad, count_bad_action);
}

std::vector<double> EvaluateModel::output()
{
	std::vector<double> output(8);
	if (global_count_ != 0) {
		output.at(0) = count_both_actions_ / global_count_;
		output.at(1) = count_follower_1_ / global_count_;
		output.at(2) = count_follower_2_ / global_count_ ;
		output.at(3) = count_bad_action_f1_ / global_count_;
		output.at(4) = count_bad_action_f2_ / global_count_;
		output.at(5) = count_not_bad_f1_ / global_count_;
		output.at(6) = count_not_bad_f2_ / global_count_;
		output.at(7) = global_count_;
	}
	return output;
}
