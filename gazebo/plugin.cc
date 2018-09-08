# include <plugin.hh>


void gazebo::Iris_position::Load(gazebo::physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Store the pointer to the model
  this->model = _parent;
  //this->state_ = 
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = gazebo::event::Events::ConnectWorldUpdateBegin(
									  std::bind(&Iris_position::get_position, this));
  
}

ignition::math::Pose3d gazebo::Iris_position::get_position()
{

  return  state_.Pose();
  
}
