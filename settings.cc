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
  po::options_description option("Allowed");
  
  option.add_options()
    ("help,h", "Print this help message and exit" )				
    ("version,v", "Print the current version")
    ("Versbose,V", "Be more verbose")
    ("r", po::value<std::string>(&file_name_),"Read program option from a file")
    
    ("n", po::value<int>(&number_of_quads_)->default_value(1),
     "Number of quadcopters to create")
    ("udp",
     po::value<std::string>(&connection_url_)->default_value("udp://:14540"),
     "Connection URL format should be: udp://[bind_host][:bind_port] \n, For example to connect to simulator use --udp udp://:14540")
    ("tcp",
     po::value<std::string>(&connection_url_),"Connection URL format should be: tcp://[server_host][:server_port]")
    ("serial",
      po::value<std::string>(&connection_url_),"Connection URL format should be: serial:///path/to/serial/dev[:baudrate]");
 
  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, option), vm);
  po::notify(vm);
  
  if(vm.count("help")){
    usage(std::cout);
    std::cout << option << std::endl;
    exit(0);        
  }
    
  if(vm.count("version")){
    std::cout << "0.2v ";
    exit(0);        
  }

}
/*  read the program options from a json file */
void Settings::read_json(const std::string& file_path)
{
  boost::property_tree::ptree  input; 
  boost::property_tree::read_json(file_path, input);
  input.get<std::string>("quad_L", );
  input.get<std::vector<port_type>>("port_number", );
   
  
}

std::string Settings::get_connection_url() const
{
  return connection_url_;
}

std::string Settings::get_file_name() const
{
  return file_name_;
}
