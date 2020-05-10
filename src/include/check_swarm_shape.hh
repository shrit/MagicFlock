#pragma once

#include <algorithm>
#include <vector>

#include <ignition/math6/ignition/math/Vector3.hh>

#include "compute_distance.hh"
#include "Vector.hh"

using namespace ILMR;

template<class QuadrotorType>
class CheckShape
{
public:
  CheckShape() {}
  bool is_good_shape(std::vector<QuadrotorType> quads);


private:

  double lower_threshold_ =
    1; /*  1 meters minimum distance between quadrotors */
  double upper_threshold_ =
    8; /*  8 meters maximum distance between quadrotors */

  VectorHelper vec_;
  ComputeDistance dist_;
};
