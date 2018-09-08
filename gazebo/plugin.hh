#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class Iris_position : public ModelPlugin
  {
  public:
    //Iris_position();
    
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    ignition::math::Pose3d get_position();
    
  private:
    
    // Pointer to the model
    physics::ModelPtr model;

    physics::ModelState state_;
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    
  };
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(Iris_position)

}
