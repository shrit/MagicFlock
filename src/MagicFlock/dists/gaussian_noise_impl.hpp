#pragma once

template<class ContainerType>
GaussianNoise<ContainerType>::GaussianNoise(double mean,
                                            double standard_deviation)
  : mean_(mean)
  , standard_deviation_(standard_deviation)
  , generator_(random_dev())
{}

template<class ContainerType>
ContainerType
GaussianNoise<ContainerType>::apply_noise(ContainerType container)
{
  std::normal_distribution<double> normal_distribution(mean_,
                                                       standard_deviation_);
  for (int i = 0; i < container.size(); ++i) {
    double random = normal_distribution(generator_);
    container[i] = container[i] + random;
  }
  return container;
}
