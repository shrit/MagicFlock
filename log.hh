
# include <boost/log/core.hpp>
# include <boost/log/trivial.hpp>
# include <boost/log/expressions.hpp>
# include <boost/log/sinks/text_file_backend.hpp>
# include <boost/log/utility/setup/file.hpp>
# include <boost/log/utility/setup/common_attributes.hpp>
# include <boost/log/sources/severity_logger.hpp>
# include <boost/log/sources/record_ostream.hpp>


namespace log {

  namespace logging  = boost::log;
  namespace src      = boost::log::sources;
  namespace sinks    = boost::log::sinks;
  namespace keywords = boost::log::keywords;

  /// Initialize the log system
  void	init();

  using severity_level = logging::trivial::severity_level;
  using logger_type    = logging::sources::severity_logger<severity_level>;

} // namespace log
