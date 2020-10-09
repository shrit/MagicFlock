#pragma once

template<class ContainerType>
EmptyNoise<ContainerType>::EmptyNoise(double mean, double standard_deviation)
  : mean_(mean)
  , standard_deviation_(standard_deviation)
{}

template<class ContainerType>
ContainerType
EmptyNoise<ContainerType>::ApplyNoise(ContainerType container)
{
  return container;
}
