# include "settings.hh"

/*  Print the out the possible keyboard input */

void usage(std::ostream& out)
{
  
  out<< "Usage : " << std ::endl   
     << "To control the quadcopter using keyboard use : " << std::endl
     << " a : to arm" << std::endl
     << " t : to takoff" << std::endl
     << " l : to land" << std::endl
     << " s : to activate offboard mode" << std::endl
     << " key up    : to go forward" << std::endl
     << " key right : to go right" << std::endl
     << " key left  : to go left" << std::endl
     << " key down  : to go backward" << std::endl
     << " + : to turn clock wise" << std::endl
     << " - : to turn counter clock wise" << std::endl;
}

/*  Constructor for program options */

Settings::Settings(int argc, char* argv[])
{

  namespace po = boost::program_options;
  po::options_description option("Allowed:");

  train_ = false;
  
  option.add_options()
    ("help,h", "Print this help message and exit" )				
    ("version,v", "Print the current version")
    ("Versbose,V", "Be more verbose")
    ("read-from-file,r", po::value<std::string>(&file_name_),"Read program option from a file")
    ("speed,s", po::value<float>(&speed_), "Enter the speed of the quad")
    ("number,n", po::value<int>(&number_of_quads_),
	 "Number of quadcopters to create")
    ("Port, p", po::value<lt::port_type>(&port_), "Enter the port number for the Leader")
    ("connection-type,c", po::value<std::string>(&socket_),
     "Enter the connection type: udp or tcp")
    ("ports-for-followers, P", po::value< std::vector<lt::port_type> >(&ports_)->multitoken(),
     "Insert a vector of  ports of followers")
    ("train,t");
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, option), vm);
  po::notify(vm);

  //if(vm.count())

  
  if(vm.count("help")){
    usage(std::cout);
    std::cout << option << std::endl;
    exit(0);        
  }
    
  if(vm.count("version")){
    std::cout << "0.2v ";
    exit(0);        
  }

  if(vm.count("train")){
    train_ = true;    
  }

}

std::string Settings::get_file_name() const
{
  return file_name_;
}

int Settings::quad_number() const
{
  return number_of_quads_;
}

std::vector<lt::port_type> Settings::quads_ports() const
{
  return ports_;
}


float Settings::speed() const
{
  return speed_;
}

bool Settings::train() const
{
  return train_;
}
