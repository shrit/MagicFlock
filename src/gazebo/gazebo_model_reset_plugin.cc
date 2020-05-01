#include "gazebo_model_reset_plugin.hh"

namespace gazebo {
GZ_REGISTER_WORLD_PLUGIN(ResetPlugin)

ResetPlugin::ResetPlugin()
  : WorldPlugin()
  , distribution_real_(-5, +5)
  , generator_(random_dev())
{}

void
ResetPlugin::Load(physics::WorldPtr _parent, sdf::ElementPtr)
{
  world_ = _parent;
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->Name());
  std::string topicName = "~/" + this->model->GetName() + "/model_reset_plugin";
  this->sub = this->node->Subscribe(topicName, &ResetPlugin::OnMsg, this);
}

void
ResetPlugin::OnMsg(ConstResetModelPtr& _msg)
{
  world_->SetPaused(true);

  int i = 0;
  for (physics::ModelPtr m : world_->Models()) {
    if (_msg->reset()) {
      m->Reset();
      m->ResetPhysicsStates();
    }
    i = i + 1;
  }

  std::vector<ignition::math::Vector3d> positions = RandomPoseGenerator(i);

  for (physics::ModelPtr m : world_->Models()) {
    m->SetRelativePose(ignition::math::Pose3d(
      positions.at(i).X(), positions.at(i).Y(), 0, 0, 0, 0));
  }

  world_->SetPaused(false);
}

std::vector<ignition::math::Vector3d>
ResetPlugin::RandomPoseGenerator(int quad_number)
{
  std::vector<ignition::math::Vector3d> RandomPosition(quad_number);
  for (std::size_t i = 0; i < RandomPosition.size(); ++i) {
    RandomPosition.at(i).X() = distribution_real_(generator_);
    RandomPosition.at(i).Y() = distribution_real_(generator_);
    RandomPosition.at(i).Z() = 0;

    for (std::size_t j = 0; j < i; ++j) {
      if ((RandomPosition.at(i).X() - RandomPosition.at(j).X()) < 1) {
        RandomPosition.at(i).X() = distribution_real_(generator_);
      }
      if ((RandomPosition.at(i).Y() - RandomPosition.at(j).Y()) < 1) {
        RandomPosition.at(i).Y() = distribution_real_(generator_);
      }
    }
  }
  return RandomPosition;
}
}
