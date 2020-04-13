#ifndef GLOBAL_HH_
#define GLOBAL_HH_

#include <fstream>
#include <iostream>
#include <iterator>
#include <vector>

#include "logger.hh"

using namespace ILMR;
/*  Global name space for the simulation
    It contain a namespace with several
    defintion used in all files
*/
namespace local_types {

template<class T>
struct position_GPS
{
  T latitude_deg;
  T longitude_deg;
  T absolute_altitude_m;
  T relative_altitude_m;
};

template<typename T>
class rssi
{

public:
  rssi<T>() {}

  const T f1() const { return f1_; }
  const T f2() const { return f2_; }
  const T f3() const { return f3_; }

  void f1(T value) { f1_ = value; }
  void f2(T value) { f2_ = value; }
  void f3(T value) { f3_ = value; }

private:
  T f1_;
  T f2_;
  T f3_;
};

using port_type = uint16_t;
using connection_type = std::string;
using topic_name = std::string;
}

/*  Overloading the << operator to print local structs, vectors and classes */
template<typename T>
std::ostream&
operator<<(std::ostream& out, const local_types::position_GPS<T>& p)
{
  out << "[" << p.latitude_deg << ", " << p.longitude_deg << ", "
      << p.absolute_altitude_m << ", " << p.relative_altitude_m << "]";
  return out;
}

template<typename T>
std::ostream&
operator<<(std::ostream& out, const local_types::rssi<T>& r)
{ /*  Print in order  f1, f2 ,f3, according to TF FF TL */
  out << r.f1() << "," << r.f2() << "," << r.f3();
  return out;
}

template<typename T1, typename T2>
std::ostream&
operator<<(std::ostream& out, const std::map<T1, T2>& m)
{
  for (auto it : m) {
    out <<"[" <<it.first << ":" << it.second << "]"; 
  }
  return out;
}

#endif
