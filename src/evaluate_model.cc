# include "../evaluate_model.hh"


EvaluateModel::EvaluateModel()
	:count_both_actions_(0),
	 count_follower_1_(0),
	 count_follower_2_(0),
	 count_bad_action_f1_(0),
	 count_bad_action_f2_(0),
	 count_not_bad_f1_(0),
	 count_bad_action_f2_(0)
{}

void EvaluateModel::input(Actions::Action leader_action, 
												 Actions::Action follower_1_action,
												 Actions::Action follower_2_action);
{
	if (leader_action == follower_1_action) {
		count_follower_1_++;
	} else if (leader_action == follower_2_action) {
		count_follower_2_++;
	}

	if (leader_action == follower_1_action and
			leader_action == follower_2_action) {
				count_both_action_++;
	}
	if (leader_action != follower_1_action) {
		std::tie(count_not_bad_f1_, count_bad         ) = evaluate_not_similar_actions(leader_action, follower_1_action);
	} else if (leader_action != follower_2_action) {
		std::tie () = evaluate_not_similar_actions(leader_action, follower_2_action);		
	}
}

std::tuple<int, int> EvaluateModel::
evaluate_not_similar_actions(Actions::Action leader_action, 
														 Actions::Action follower_action);
{
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
	return std::make_tuple<int, int>(count_not_bad, count_bad_action);
}

std::vector<double> EvaluateModel::output()
{
	std::vector<double> output(7);
	output.at(0) = count_both_actions_;
	output.at(1) = count_follower_1_;
	output.at(2) = count_follower_2_;
	output.at(3) = count_bad_action_f1_;
	output.at(4) = count_bad_action_f2_;
	output.at(5) = count_not_bad_f1_;
	output.at(6) = count_not_bad_f2_;
	return output;
}
