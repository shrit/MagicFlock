#pragma once

#include <random>

template<class ContainerType>
class EmptyNoise
{
public:
  EmptyNoise();
  EmptyNoise(double mean, double standard_deviation);

  ContainerType ApplyNoise(ContainerType container);

private:
  double mean_ = 0;
  double standard_deviation_ = 0.19;
};

#include "empty_noise_impl.hpp"
