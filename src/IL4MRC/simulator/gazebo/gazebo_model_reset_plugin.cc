#include "gazebo_model_reset_plugin.hh"

namespace gazebo {

GZ_REGISTER_WORLD_PLUGIN(ResetPlugin);
ResetPlugin::ResetPlugin()
  : WorldPlugin()
  , distribution_real_(-2, +2)
  , generator_(random_dev())
{}

void
ResetPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr)
{
  world_ = _parent;
  node_ = transport::NodePtr(new transport::Node());
  node_->Init();
  std::string topicName = "/gazebo/default/model_reset_plugin";
  sub_ = node_->Subscribe(topicName, &ResetPlugin::OnMsg, this);
}

void
ResetPlugin::OnMsg(ConstResetModelPtr& _msg)
{
  world_->SetPaused(true);

  int i = 0;
  for (physics::ModelPtr m : world_->Models()) {
    std::string name = m->GetName();
    if (!name.empty()) {
      name.pop_back();
    }
    if (name == "iris_") {
      if (_msg->reset()) {
        m->Reset();
        m->ResetPhysicsStates();
      }
      i = i + 1;
    }
  }

  std::vector<ignition::math::Vector3d> positions = RandomPoseGenerator(i);
  //  std::cout << "All positions are generated" << std::endl;
  int j = 0;

  for (physics::ModelPtr m : world_->Models()) {
    std::string name = m->GetName();
    if (!name.empty()) {
      name.pop_back();
    }
    if (name == "iris_") {
      if (j < i) {
        m->SetRelativePose(ignition::math::Pose3d(
          positions.at(j).X(), positions.at(j).Y(), 0, 0, 0, 0));
        j = j + 1;
      }
    }
  }
  world_->SetPaused(false);
}

std::vector<ignition::math::Vector3d>
ResetPlugin::RandomPoseGenerator(int quad_number)
{
  /* This piece of code relay on circle packing algorithm
   *  We consider the place occupied by a quadrotor as a circle
   *  then the diameter should be 40 centimeters.
   *  We do not want these quadrotors to overlap at any price
   *  this is the reason for using circle packing
   */
  std::vector<ignition::math::Vector3d> RandomPosition(quad_number);
  double radius = 0.7;

  for (int i = 0; i < quad_number; ++i) {
    ignition::math::Vector3d quadRegion;
    quadRegion.X() = distribution_real_(generator_);
    quadRegion.Y() = distribution_real_(generator_);
    quadRegion.Z() = 0;
    std::vector<bool> overlapping(quad_number, false);
    for (int j = 0; j < i; ++j) {
      double d = quadRegion.Distance(RandomPosition.at(j));
      std::cout << "Distance between quads on generating: " << d << std::endl;
      if (d < radius) {
        overlapping.at(j) = true;
      }
    }
    if (std::all_of(
          overlapping.begin(), overlapping.end(), [](const bool& overlap) {
            std::cout << "Overlapping: " << overlap << std::endl;
            return overlap == false;
          })) {
      std::cout << "Added Not overlapping" << std::endl;
      RandomPosition.at(i) = quadRegion;
    }
  }
  for (int j = 0; j < quad_number; ++j) {
    for (int i = 0; i < quad_number; ++i) {
      std::cout << "Used final distance:  "
                << RandomPosition.at(j).Distance(RandomPosition.at(i))
                << std::endl;
    }
  }

  return RandomPosition;
}
}
