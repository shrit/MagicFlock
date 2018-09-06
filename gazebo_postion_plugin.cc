# include <gazebo_poition_plugin.hh>


void Iris_position::Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
{
  // Store the pointer to the model
  this->model = _parent;
  
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
								  std::bind(&ModelPush::get_position, this));
  
}

gazebo::math::Pose3d get_position()
{

  return  this->model->Pose();
  
}
