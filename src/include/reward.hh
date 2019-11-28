#pragma once

# include "global.hh"

namespace lt = local_types;

class Rewards {

public:

    enum class Reward
    {
     very_good,
     good,
     bad,
     very_bad,
     Unknown,
    };
  
  Reward evaluate_current_state(const lt::triangle<double>& old_dist,
				const lt::triangle<double>& new_dist);
    
};
