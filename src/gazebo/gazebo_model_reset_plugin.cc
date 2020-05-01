#include "gazebo_model_reset_plugin.hh"

namespace gazebo {
GZ_REGISTER_MODEL_PLUGIN(ResetPlugin)

ResetPlugin::ResetPlugin()
  : ModelPlugin()
  , distribution_real_(-7, +7)
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
  std::vector<ignition::math::Pose3d> positions = RandomPoseGenerator();

  for (physics::ModelPtr m : world->Models())
  {
    m->SetRelativePose(
      ignition::math::Pose3d(v.pos().x(), v.pos().y(), 0, 0, 0, 0));

    if (_msg->reset()) {
      m->Reset();
      m->ResetPhysicsStates();
    }
  }
}

std::vector<ignition::math::Pose3d>
ResetPlugin::RandomPoseGenerator(int quad_number)
{
  std::vector<ignition::math::Pose3d> RandomPosition(quad_nubmer);
  for (int i = 0; i < RandomPosition.size(); ++i) {
    RandomPosition.at(i).pos().x() = distribution_real_(generator_);
    RandomPosition.at(i).pos().y() = distribution_real_(generator_);
    RandomPosition.at(i).pos().z() = 0;

    for (int j = 0; j < i; ++j) {
      if ((RandomPosition.at(i).pos().x() - RandomPosition.at(j).pos().x()) <
          1) {
        RandomPosition.at(i).pos().x() = distribution_real_(generator_);
      }
      if ((RandomPosition.at(i).pos().y() - RandomPosition.at(j).pos().y()) <
          1) {
        RandomPosition.at(i).pos().y() = distribution_real_(generator_);
      }
    }
  }
}
