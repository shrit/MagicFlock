#pragma once

# include <sstream>


/*  Speed log include  */
#include <spdlog/spdlog.h>
#include <spdlog/logger.h>
#include <spdlog/sinks/stdout_color_sinks.h> 
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/fmt/ostr.h> 

/*  Boost and file system includes */
# include <experimental/filesystem>
# include <boost/date_time/posix_time/posix_time.hpp>

/*  Init the logging system */

namespace ILMR
{  
  std::shared_ptr<spdlog::logger> setup_logger(std::vector<spdlog::sink_ptr> sinks);  
  std::shared_ptr<spdlog::logger> init();  
}



