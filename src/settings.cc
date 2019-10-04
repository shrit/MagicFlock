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
  po::options_description global("Allowed:");

  global.add_options()
    ("help,h", "Print this help message and exit" )
    ("version,v", "Print the current version")
    ("Versbose,V", "Be more verbose")
    ("ini-file,n", po::value<std::string>(&ini_file_)->value_name("ini_file"),"Specify the name of the ini file")
    ("generate-dataset","Generate data set by doing generating random trajectories")
    ("training", po::value<std::string>(&dataset_file_)->value_name("dataset"), "Train a dataset file using a neural network, 10% of the dataset is used for testing")
    ("classification", "Treat the problem as classification problem, labels are classed into classes ")
    ("regression", "Treat the problem as regression problem ")
    ("testing", "Test an aleardy trained controller on the followers")
    ("trajectory-noise", "Create an estimator to estimate the noise in trajectory");

  po::positional_options_description pos;
  pos.add("training", 2);

  po::variables_map vm;

  po::parsed_options parsed = po::command_line_parser(argc, argv).
    options(global).
    positional(pos).
    allow_unregistered().
    run();

  po::store(parsed, vm);

  po::notify(vm);

  if (vm.count("help")) {
    usage(std::cout);
    std::cout << global << std::endl;
    exit(0);
  }

  if (vm.count("generate-dataset")) {
    generate_ = true;
  }

  if (vm.count("training")) {    
    training_ = true;        
  }

  if (vm.count("testing")) {
    testing_ = true;
  }

  if (vm.count("classification")) {
    classification_ = true;
  }

  if (vm.count("regression")) {
    regression_ = true;
  }

  if (vm.count("trajectory-noise")) {
    trajectory_noise_ = true;
  }

  if (vm.count("version")) {
    std::cout << "0.9v ";
    exit(0);
  }

  if(vm.count("Verbose")) {
    std::cout << "Debug level increased ";
  }
}

bool Settings::generate() const
{ return generate_; }

bool Settings::training() const
{ return training_; }

bool Settings::testing() const
{ return testing_; }

bool Settings::classification() const
{ return classification_; }

bool Settings::regression() const
{ return regression_; }

bool Settings::trajectory() const
{ return trajectory_noise_; }

std::string Settings::dataset() const
{ return dataset_file_; }



