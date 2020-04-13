#pragma once

#include "logger.hh"

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

  Reward evaluate_current_state();
};
