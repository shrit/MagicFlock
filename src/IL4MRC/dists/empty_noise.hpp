#pragma once

template<class ContainerType>
class EmptyNoise
{
public:
  EmptyNoise() {};
  EmptyNoise(double mean, double standard_deviation)
    : mean_(mean)
    , standard_deviation_(standard_deviation)
  {}

  ContainerType apply_noise(ContainerType container) { return container; }

  double& mean() { return mean_; }
  double& standard_deviation() { return standard_deviation_; }

private:
  double mean_ = 0;
  double standard_deviation_ = 0;
};

