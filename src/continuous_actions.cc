#include "include/continuous_actions.hh"

ContinuousActions::ContinuousActions() {}

ignition::math::Vector3d
ContinuousActions::Data() const
{
  return velocity_vector_;
}

ignition::math::Vector3d&
ContinuousActions::Data()
{
  return velocity_vector_;
}
