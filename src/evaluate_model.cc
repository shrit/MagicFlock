# include "../evaluate_model.hh"


EvaluateModel::EvaluateModel(){}



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
		evaluate_not_similar_actions(leader_action, follower_1_action)
	} else if () {

		
	} 
}

void EvaluateModel::evaluate_not_similar_actions(Actions::Action leader_action, 
																								 Actions::Action follower_action);
{
	if ((leader_action == Actions::Action::forward and
	    follower_action == Actions::Action::left)
	    or  (leader_action == Actions::Action::left and
	    follower_action == Actions::Action::forward)) {
			count_not_bad_f1++;
	} else if ((leader_action == Actions::Action::backward and
	    follower_action == Actions::Action::right)
	    or  (leader_action == Actions::Action::right and
	    follower_action == Actions::Action::backward)) {
			count_not_bad_f1++;
	}	else {
		count_bad_action++;
	}

}

std::vector<double> EvaluateModel::output()
{







}
