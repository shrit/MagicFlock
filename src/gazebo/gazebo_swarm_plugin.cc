/*  This code is borrowed from Tansa dansing platforme, with some
 * modifications*/

/*
   Gazebo plugin loaded into an empty world to make it easy add in many
   distinguishable drone models.

   The mavlink interface in sitl_gazebo listens for messages on a random
   port and sends messages to the given config port (default 14560). Likewise
   the PX4 simulator module be default listens for messages on 14560
*/

#include "gazebo/common/Plugin.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "spawn.pb.h"
#include <ignition/math6/ignition/math/Pose3.hh>
#include <sdf/sdf.hh>

#include <fstream>
#include <signal.h>
#include <streambuf>
#include <string>
#include <sys/wait.h>
#include <unistd.h>

#include <boost/shared_ptr.hpp>

#include <iostream>
#include <string>

typedef const boost::shared_ptr<const tansa::msgs::SpawnRequest>
  SpawnRequestPtr;
typedef const boost::shared_ptr<const gazebo::msgs::Pose> PosePtr;


class SwarmPlugin : public WorldPlugin
{
public:
  void Load(physics::WorldPtr _parent, sdf::ElementPtr)
  {
    node = transport::NodePtr(new transport::Node());
    world = _parent;

    node->Init(world->Name());
    spawnSub = node->Subscribe("~/spawn", &SwarmPlugin::spawn_callback, this);
    requestPub = node->Advertise<gazebo::msgs::Request>("~/request");
  }

  void spawn_callback(SpawnRequestPtr& msg)
  {
    world->SetPaused(true);
    // Figure out what to do with existing models in the world
    int nexist = 0;
    for (physics::ModelPtr m : world->Models()) {
      std::string name = m->GetName();
      if (strncmp(name.c_str(), "iris_", 8) == 0) {

        int num = atoi(name.c_str() + 8);
        if (num < msg->vehicles_size()) { // Reuse it
          const tansa::msgs::SpawnRequest_Vehicle& v = msg->vehicles(num);
          m->SetRelativePose(ignition::math::Pose3d(v.pos().x(),
                                                    v.pos().y(),
                                                    v.pos().z(),
                                                    v.orient().x(),
                                                    v.orient().y(),
                                                    v.orient().z()));
          nexist++;
        } else { // Delete it
          msgs::Request* msg =
            gazebo::msgs::CreateRequest("entity_delete", name);
          requestPub->Publish(*msg, true);
        }
      }
    }

    std::ifstream t(msg->sdf_file());
    std::string str((std::istreambuf_iterator<char>(t)),
                    std::istreambuf_iterator<char>());

    sdf::SDF file;
    file.SetFromString(str);

    sdf::ElementPtr root = file.Root();
    sdf::ElementPtr model = root->GetFirstElement();
    sdf::ElementPtr pose = model->AddElement("pose");

    // Finding a reference to the plugin which handles mavlink
    sdf::ElementPtr plugin = model->GetElement("plugin");
    while (plugin->GetAttribute("name")->GetAsString() != "mavlink_interface") {
      plugin = plugin->GetNextElement("plugin");
    }

    sdf::ElementPtr port = plugin->GetElement("mavlink_udp_port");

    for (int i = nexist; i < msg->vehicles_size(); i++) {
      const tansa::msgs::SpawnRequest_Vehicle& v = msg->vehicles(i);

      model->GetAttribute("name")->Set("iris_" + std::to_string(i));

      std::string p =
        std::to_string(v.pos().x()) + " " + std::to_string(v.pos().y()) + " " +
        std::to_string(v.pos().z()) + " " + std::to_string(v.orient().x()) +
        " " + std::to_string(v.orient().y()) + " " +
        std::to_string(v.orient().z());

      pose->Set<std::string>(p);
    }
  }



private:
  transport::NodePtr node;
  transport::SubscriberPtr spawnSub;
  transport::PublisherPtr requestPub;
  physics::WorldPtr world;

  transport::SubscriberPtr world_sub;


};

