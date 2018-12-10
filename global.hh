#ifndef GLOBAL_HH_
#define GLOBAL_HH_




/*  Global name space for the simulation
    It contain a namespace with several 
    defintion used in all files 
*/




namespace local_types {
  
  struct position {
    
    double x;
    double y;
    double z;
    
  };

  struct rssi {
    double lf1;
    double lf2;
    double ff;
  };
  
  /// Type of a drone's IP
  using ip_type   = std::string;
  /// Type of a drone's port
  using port_type = uint16_t;
  //Type of drone's conncetion socket
  using connection_type = std::string;


  using topic_name               = std::string;

  using quads_positions          = std::vector<position>;

  using quads_rssi               = std::vector<double>;
}




#endif 
