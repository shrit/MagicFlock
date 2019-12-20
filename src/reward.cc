# include "include/reward.hh"

Rewards::Reward Rewards::
evaluate_current_state(const lt::triangle<double>& old_dist,
			const lt::triangle<double>& new_dist)
{
  logger::logger_->info("F1 differences: {}",std::fabs(old_dist.f1 - new_dist.f1));
  logger::logger_->info("F2 differences: {}",std::fabs(old_dist.f2 - new_dist.f2));

  double diff_f1 = std::fabs(old_dist.f1 - new_dist.f1);
  double diff_f2 = std::fabs(old_dist.f2 - new_dist.f2);

  Reward reward = Reward::Unknown;

  if (0.5  > diff_f1 + diff_f2 ) {
    reward = Reward::very_good;
  } else if ( 1.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 0.5 ) {
    reward = Reward::good;
  } else if ( 1.5  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.0 ) {
    reward = Reward::bad;
  } else if ( 2.0  > diff_f1 + diff_f2 and
	      diff_f1 + diff_f2  > 1.5 ) {
    reward = Reward::very_bad;
  }
  return reward;
}
