#include "log.hh"


std::ostream& operator<< (std::ostream& os, level level)
{
    static const char* strings[] =
      {
       "Msg",
       "Dbg",
       "Wrn",
       "Err"                
      };

    if (static_cast< std::size_t >(level) < sizeof(strings) / sizeof(*strings))
        os << strings[level];
    else
        os << static_cast< int >(level);

    return os;
}


void init()
{
   logging::add_file_log
    (
        keywords::file_name = "Data_set_%N.log",
        // This makes the sink to write log records that look like this:
        // 1: [Msg] A normal severity message
        // 2: [Err] An error severity message
        keywords::format =
        (
	 expr::stream
	 << "["
	 << expr::format_date_time< boost::posix_time::ptime >("TimeStamp", "%H:%M:%S")
	 << "]"
	 << "["
	 << expr::attr<level, severity_tag>("Severity")
	 << "] " << expr::smessage
	 ),
	keywords::rotation_size = 10 * 1024 * 1024,    /*10 MB files*/                                 
	keywords::time_based_rotation = sinks::file::rotation_at_time_point(12, 0, 0)  
	  
    );
   
}

// int main(int, char*[])
// {
//   init();
  
//   logging::add_common_attributes();

  
//   boost::log::sources::severity_logger<level> lg;

//   //  lg.add_attribute();
    
//   BOOST_LOG_SEV(lg, Msg) << "A trace severity message";
//   BOOST_LOG_SEV(lg, Dbg) << "A debug severity message";
//   BOOST_LOG_SEV(lg, Err) << "An informational severity message";
//   BOOST_LOG(lg) << "An informational severity message";
//   BOOST_LOG(lg) << "An informational severity message";
//   BOOST_LOG(lg) << "A warning severity message";
//   BOOST_LOG(lg) << "An error severity message";
//   BOOST_LOG(lg) << "A fatal severity message";
  
//   return 0;
// }
