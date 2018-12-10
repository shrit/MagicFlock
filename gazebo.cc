# include "gazebo.hh"

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

Gazebo::Gazebo(int argc, char* argv[])
  :  node_(new gazebo::transport::Node())
{
  
  gazebo::client::setup(argc, argv);
  
  node_->Init();
  
}

//template <typename MsgHandler>
void Gazebo::subscriber(lt::topic_name name)
{
  SubPtr sub = node_->Subscribe(name, &Gazebo::Parse_rssi_msg, this);
  //  subs_.push_back(sub); 
}

void Gazebo::Parse_rssi_msg(ConstVector2dPtr& msg)
{
  rssi_.push_back(msg->x());
  if (rssi_.size() == 3){
    signal_.lf1 = rssi_.at(0);
    signal_.lf2 =  rssi_.at(1);
    signal_.ff = rssi_.at(2);
    std::cout << rssi_ << std::endl;
    rssi_.clear();
  }
}

void Gazebo::Parse_position_msg(ConstPosesStampedPtr& posesStamped)
{

  for (int i =0; i < posesStamped->pose_size(); ++i)
    {
      const ::gazebo::msgs::Pose &pose = posesStamped->pose(i);
      std::string name = pose.name();
      if (name == std::string("iris"))
        {
          const ::gazebo::msgs::Vector3d& position = pose.position();
	  
          position_.x = position.x();
          position_.y = position.y();
          position_.z = position.z();

	}
    }  
}

//quad_positions
lt::position Gazebo::get_quads_positions() const
{ return position_;}

//quads_rssi
std::vector<double> Gazebo::get_quads_rssi() const
{return rssi_;}


