#pragma once

#include "global.hh"
#include "logger.hh"

namespace lt = local_types;
using namespace ILMR;

class Rewards
{

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
