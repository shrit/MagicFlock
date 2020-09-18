#include <IL4MRC/util/logger.hpp>

namespace ILMR {
std::shared_ptr<spdlog::logger> logger::logger_;
std::shared_ptr<spdlog::logger>
logger::setup_logger(std::vector<spdlog::sink_ptr> sinks)
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

std::shared_ptr<spdlog::logger>
logger::init()
{
  boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
  std::stringstream date, time;
  date << now.date();
  time << now.time_of_day();

  std::experimental::filesystem::create_directory("../log");
  std::experimental::filesystem::create_directory("../log/" + date.str());
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

void
logger::create_library_logger(std::shared_ptr<spdlog::logger> logger)
{
  logger_ = logger;
}
}
