#pragma once

#include <random>

template<class ContainerType>
class GaussianNoise
{
public:
  GaussianNoise(double mean, double standard_deviation);

  ContainerType ApplyNoise(ContainerType container);

private:
  std::normal_distribution<double> normal_distribution_;
  std::random_device random_dev;
  std::mt19937 generator_;

};

#include "gaussian_noise.hxx"
