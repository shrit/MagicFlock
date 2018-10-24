#include "log.hh"


namespace log {

  void init_logging()
  {
    std::shared_ptr< logging::core > core = logging::core::get();

    std::shared_ptr< sinks::text_multifile_backend > backend =
      std::make_shared< sinks::text_multifile_backend >();

    // Set up the file naming pattern
    backend->set_file_name_composer
      (
       sinks::file::as_file_name_composer
       (expr::stream << "logs/" << expr::attr< std::string >("") << ".log")
       );

    // Wrap it into the frontend and register in the core.
    // The backend requires synchronization in the frontend.
    using sink_t =  sinks::synchronous_sink< sinks::text_multifile_backend >;
    std::shared_ptr<sink_t> sink(new sink_t(backend));

    // Set the formatter
    sink->set_formatter
      (
       expr::stream
       << "[: " << expr::attr< std::string >("RequestID")
       << "] " << expr::smessage
       );

    core->add_sink(sink);
  }


} // namespace log
