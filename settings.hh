# include <iostream>

# include <boost/program_options.hpp>
# include <boost/property_tree/ptree.hpp>
# include <boost/property_tree/json_parser.hpp>

/**
 * Settings for the controller code
 * it parses the command line to run the program 
 * TODO: make the configuration readable from a file
*/

class Settings {

public:
   /// Type of a drone's IP
    using ip_type   = std::string;
    /// Type of a drone's port
    using port_type = uint16_t;


  Settings(int argc, char* argv[]);
  
  std::string get_connection_url() const;

  void read_json(const std::string& file_path);
  
private:
  
  int number_of_quads_;
  port_type	port_;
  
  /// Drone on which to connect to.
  ip_type	drone_ip_;

  // number of udp ports to connects to quads
  std::vector<port_type> ports_;
  std::string file_name_;
  std::string connection_url_;


};
