#pragma once

template<class ContainerType>
GaussianNoise<ContainerType>::GaussianNoise(double mean,
                                            double standard_deviation)
  : normal_distribution_(mean, standard_deviation)
  , generator_(random_dev())
{}

template<class ContainerType>
ContainerType
GaussianNoise<ContainerType>::ApplyNoise(ContainerType container)
{
  for (int i = 0; i < container.size(); ++i) {
    double random = normal_distribution_(generator_);
    container[i] = container[i] + random;
  }
  return container;
}
