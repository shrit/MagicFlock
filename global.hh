#ifndef GLOBAL_HH_
#define GLOBAL_HH_

# include <iostream>
# include <fstream>
# include <vector>
# include <iterator>

# include "log.hh"

template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<T>& v) {
  if ( !v.empty() ) {
    //    out << '[';
    std::copy (v.begin(), v.end(), std::ostream_iterator<T>(out, ","));
    // out << "\b\b]";
  }
  return out;
}


template <typename T>
std::ostream& operator<< (std::ostream& out, const std::vector<std::vector<T>>& v) {
  if ( !v.empty() ) {
    out << '[';
    for (int i=0; i < v.size(); i++ )
      std::copy (v.at(i).begin(), v.at(i).end(), std::ostream_iterator<T>(out, ","));
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
  class rssi {
    
  public:
    
    rssi<T>(){}
	    
    
    const T lf1() const
    { return lf1_;}
    const T lf2() const
    { return lf2_;}
    const T ff() const
    {return ff_;}      

    void lf1(T value) 
    {  lf1_ =  value;}
    void lf2(T value) 
    {  lf2_ = value;}
    void ff(T value) 
    { ff_ = value;}      
    
    
  private:
    
    T lf1_;
    T lf2_;
    T ff_;
  };

  template<typename T>
  struct triangle 
  {
    T a;
    T b;
    T c;      
  };
    
  template <typename T> 
  struct error {
    T x;
    T y;
  };

  // bool assert_equal(double one, double two)
  // {
  //   bool ok = (std::fabs(one - two) < std::numeric_limits<double>::epsilon());
  //   if(ok != true){
  //     LogErr() << "Err: Can not assert values";
  //     exit(0);
  //   }
  //   return ok;
  // }
  
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
std::ostream& operator<< (std::ostream& out, const local_types::position<T>& p)
{
  out << "["<< p.x <<", " << p.y <<", " << p.z <<"]";
  return out;
}


template <typename T>
std::ostream& operator<< (std::ostream& out, const local_types::rssi<T>& r)
{
  //  out << "["<< r.lf1 <<", " << r.lf2 <<", " << r.ff <<"]";
  out << r.lf1() <<"," << r.lf2() <<"," << r.ff();
  return out;
}

#endif 
