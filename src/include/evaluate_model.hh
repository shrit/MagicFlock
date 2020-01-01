#pragma once 

# include <vector>

# include "action.hh"


class EvaluateModel 
{

public:
	EvaluateModel(); 

	void input(Actions::Action leader_action, 
						 Actions::Action follower_1_action,
						 Actions::Action follower_2_action);

	std::tuple<int, int> evaluate_not_similar_actions(Actions::Action leader_action, 
																	 Actions::Action follower_1_action,
																	 Actions::Action follower_2_action);

  std::vector<double> output();

private:

	int count_both_actions_;
	int count_follower_1_;
	int count_follower_2_;
	int count_bad_action_f1_;
	int count_bad_action_f2_;
	int count_not_bad_f1_;
	int count_not_bad_f2_;
};




