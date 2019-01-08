#ifndef _MODEL_RESET_HH_
#define _MODEL_RESET_HH_

# include <gazebo/gazebo.hh>
# include <gazebo/physics/physics.hh>
# include <gazebo/common/common.hh>
# include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ResetPlugin : public ModelPlugin
  {

  public: ResetPlugin(){}
    
  public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
            
      // Store the model pointer for convenience.
      this->model = _parent;

      // Create the node
      this->node = transport::NodePtr(new transport::Node());
  
      this->node->Init(this->model->GetWorld()->Name());
  
      // Create a topic name
      std::string topicName = "~/" + this->model->GetName() + "/model_reset";

      // Subscribe to the topic, and register a callback
      this->sub = this->node->Subscribe(topicName,
					&ResetPlugin::OnMsg, this);
    }
    
    // Called by the world update start event
  public: void OnMsg(ConstVector2dPtr &_msg)
    {
      // Apply a small linear velocity to the model.
      if (_msg->x() == 1){
	this->model->Reset();	
      }
    }

    // Pointer to the model
  private: physics::ModelPtr model;
    
    /// \brief A node used for transport
  private: transport::NodePtr node;
    
  private: transport::SubscriberPtr sub;
        
    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ResetPlugin)
}

#endif
