#pragma once

#include <random>

template<class ContainerType>
class GaussianNoise
{
public:
  GaussianNoise()
  {
   // Nothing to do here
  }

  GaussianNoise(double mean, double standard_deviation);

  double& mean() { return mean_; }
  double& standard_deviation() { return standard_deviation_; }

  ContainerType apply_noise(ContainerType container);

private:
  double mean_ = 0;
  double standard_deviation_ = 0.19;
  std::random_device random_dev;
  std::mt19937 generator_;

};

#include "gaussian_noise_impl.hpp"
