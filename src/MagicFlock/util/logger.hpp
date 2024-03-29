#pragma once

#include <memory>
#include <sstream>
#include <thread>
#include <filesystem>

/*  Speed log include  */
#include <spdlog/spdlog.h>
#include <spdlog/logger.h>
#include <spdlog/async.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/fmt/ostr.h>

/*  Boost and file system includes */
#include <boost/date_time/posix_time/posix_time.hpp>

/*  Init the logging system */
namespace ILMR
{
  class logger
  {
  public:
    static std::shared_ptr<spdlog::logger> setup_logger(std::vector<spdlog::sink_ptr> sinks)
    {
      spdlog::init_thread_pool(8192, 1);
      auto logger = std::make_shared<spdlog::async_logger>(
        "IL",
        sinks.begin(),
        sinks.end(),
        spdlog::thread_pool(),
        spdlog::async_overflow_policy::block);
        spdlog::register_logger(logger);

      return logger;
    }

    static std::shared_ptr<spdlog::logger> init()
    {
      boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
      std::stringstream date, time;
      date << now.date();
      time << now.time_of_day();

      std::filesystem::create_directory("../log");
      std::filesystem::create_directory("../log/" + date.str());
      std::string log_file_name = "../log/" + date.str() + "/" + time.str();

      auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
      console_sink->set_level(spdlog::level::info);

      auto rotating_sink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
        log_file_name, 1024 * 1024 * 10, 3);
      rotating_sink->set_level(spdlog::level::trace);

      std::vector<spdlog::sink_ptr> sinks{ rotating_sink, console_sink };
      auto logger = setup_logger(sinks);
      return logger;
    }

    static void create_library_logger(std::shared_ptr<spdlog::logger> logger)
    {
      logger_ = logger;
    }

    inline static std::shared_ptr<spdlog::logger> logger_;
  };
}


