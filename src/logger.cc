# include "include/logger.hh"

namespace ILMR
{
  static const std::string logger_name = "";
  
  std::shared_ptr<spdlog::logger> setup_logger(std::vector<spdlog::sink_ptr> sinks)
  {
    auto logger = spdlog::get(logger_name);
    if (not logger) {
      if (sinks.size() > 0) {
        
	logger = std::make_shared<spdlog::logger>(logger_name,
						  std::begin(sinks),
						  std::end(sinks));
	spdlog::register_logger(logger);
      } else {
	logger = spdlog::stdout_color_mt(logger_name);
      }
    }    
    return logger;
  }

  std::shared_ptr<spdlog::logger> init()
  {        
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();   
    date_stream_ << now.date();
    time_stream_ << now.time_of_day();
    
    std::experimental::filesystem::create_directory("../log");
    std::experimental::filesystem::create_directory("../log/" + date_stream_.str());
    log_file_name = "../log/" + date_stream_.str() + "/" + time_stream_.str();
    
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::info);
    
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_name, true);
    file_sink->set_level(spdlog::level::trace);
    
    spdlog::sinks_init_list sink_list = {file_sink, console_sink};    
    auto logger = setup_logger(sink_list);
    
    return logger;
  }
}