#pragma once

#include <algorithm>
#include <vector>

#include <ignition/math6/ignition/math/Vector3.hh>

#include "global.hh"
#include "compute_distance.hh"
#include "Vector.hh"

namespace lt = local_types;

class CheckShape
{
public:
  CheckShape() {}

  bool is_good_shape(unsigned int id,
                     std::vector<unsigned int> nearest_neighbors,
                     std::vector<ignition::math::Vector3d> positions);
private:

  double lower_threshold_ =
    1; /*  1 meters minimum distance between quadrotors */
  double upper_threshold_ =
    8; /*  8 meters maximum distance between quadrotors */

  VectorHelper vec_;
  ComputeDistance dist_;
};
