#ifndef GLOBAL_HH_
#define GLOBAL_HH_

# include <iostream>
# include <fstream>
# include <vector>
# include <iterator>

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ", "));
    out << "\b\b]";
  }
  return out;
}

/*  Global name space for the simulation
    It contain a namespace with several 
    defintion used in all files 
*/

namespace local_types {

  
  template <typename T> 
  struct position {
    
    T x;
    T y;
    T z;
    
  };
  
  template <typename T> 
  struct rssi {
    T lf1;
    T lf2;
    T ff;
  };
  
  
  /// Type of a drone's IP
  using ip_type   = std::string;
  /// Type of a drone's port
  using port_type = uint16_t;
  //Type of drone's conncetion socket
  using connection_type = std::string;

  using topic_name               = std::string;

  using quads_positions          = std::vector<position<double>>;

  using quads_rssi               = std::vector<double>;
}

template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::position<T> p)
{
  out << "[ "<< p.x <<", " << p.y <<", " << p.z <<"]"<<"\n";
  return out;
}
  
template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::rssi<T> r)
{
  out << "[ "<< r.lf1 <<", " << r.lf2 <<", " << r.ff <<"]"<<"\n";
  return out;
}





#endif 
