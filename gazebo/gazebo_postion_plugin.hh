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
    void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);

    gazebo::math::Pose3d get_position();
    
  private:
    
    // Pointer to the model
    physics::ModelPtr model;
    
    // Pointer to the update event connection
    event::ConnectionPtr updateConnection;
    
  };
}
