# include "settings.hh"

/*  Print the out the possible keyboard input */

void usage(std::ostream& out)
{
  
  out<< "Usage : " << std ::endl   
     << "To control the quadcopter using keyboard use : " << std::endl
     << " a : arm" << std::endl
     << " t : takoff" << std::endl
     << " l : land" << std::endl
     << " s : activate offboard mode" << std::endl
     << " key up    : to go forward" << std::endl
     << " key right : to go right" << std::endl
     << " key left  : to go left" << std::endl
     << " key down  : to go backward" << std::endl
     << " + : to turn clock wise" << std::endl
     << " - : to turn counter clock wise" << std::endl
     << "To control the quadcopter using Xbox joystick use : " << std::endl
     << " A : to arm" << std::endl
     << " X : takeoff "<< std::endl
     << " Y : activate offboard mode" << std::endl
     << " B : land" << std::endl
     << " Right Stick up    : to go forward" << std::endl
     << " Right Stick right : to go right" << std::endl
     << " Right Stick left : to go left" << std::endl
     << " Right Stick down : to go backword" << std::endl
     << " Left Stick up    : to go up" << std::endl
     << " Left Stick right : to go down" << std::endl;
  
}

/*  Constructor for program options */

Settings::Settings(int argc, char* argv[])
{

  namespace po = boost::program_options;
  po::options_description option("Allowed:");
  
  option.add_options()
    ("help,h", "Print this help message and exit" )				
    ("version,v", "Print the current version")
    ("Versbose,V", "Be more verbose")
    ("ini-file,n", po::value<std::string>(&ini_file_),"Specify the name of the ini file")
    ("output-file,o", po::value<std::string>(&outputfile_),"Specify the file name to be used to logging");
    
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, option), vm);
  po::notify(vm);
    
  if(vm.count("help")){
    usage(std::cout);
    std::cout << option << std::endl;
    exit(0);        
  }
    
  if(vm.count("version")){
    std::cout << "0.9v ";
    exit(0);        
  }
      
  if(vm.count("Verbose")){
    std::cout << "Debug level increased ";
  }    

}
