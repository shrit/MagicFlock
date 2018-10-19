# include <iostream>

/**
 * Settings for the controller code
 * it parses the command line to run the program 
 * TODO: make the configuration readable foroma file

*/

class Settings {

public:
   /// Type of a drone's IP
    using ip_type   = std::string;
    /// Type of a drone's port
    using port_type = uint16_t;


  Settings(int argc, char* argv[]);
  
  std::string get_connection_url() const;

private:
  
  /// port for TCP connection from/to the drones.
  port_type	port_;
  
  /// Drone on which to connect to.
  ip_type	drone_ip_;
  
  
  std::string connection_url_;
  
};
