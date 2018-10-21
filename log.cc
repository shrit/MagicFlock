#include "log.hh"


namespace log {

  void init()
  { /* Modify it to be iterable file with %N 
     * Add several log files according to forum
     * Call the log inside each files instead of cout
     */
      
    logging::add_file_log(keywords::file_name = "./position.log",
			  keywords::auto_flush = true,
			  keywords::format = "[%TimeStamp%]: %Message%");

    logging::core::get()->set_filter
      (logging::trivial::severity >= logging::trivial::debug);
  }

} // namespace log
