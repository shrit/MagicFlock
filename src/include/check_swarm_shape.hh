#pragma once

#include <algorithm>
#include <vector>

#include <ignition/math6/ignition/math/Vector3.hh>

#include "Vector.hh"
#include "compute_distance.hh"

using namespace ILMR;

template<class QuadrotorType>
class CheckShape
{
public:
  CheckShape() {}
  CheckShape(double lower_threshold, double upper_threshold);

  double& lower_threshold();
  double lower_threshold() const;

  double& upper_threshold();
  double upper_threshold() const;

  bool is_good_shape(const std::vector<QuadrotorType>& quads);

private:
  double lower_threshold_ =
    0; /*  0 meters minimum distance between quadrotors */
  double upper_threshold_ =
    20; /*  20 meters maximum distance between quadrotors */

  VectorHelper vec_;
  ComputeDistance dist_;
};

#include "check_swarm_shape.hxx"
