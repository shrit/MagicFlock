# include <connection.hh>


TCPClient(socket_type&&				socket,
	  const boost::optional<endpoint_type>&	ep = boost::none)
  : socket_(std::move(socket)),
   endpoint_(ep)
{}


void TCPClient::connect()
{
  
  auto self(this->shared_from_this());
  if (endpoint_)
    socket_.async_connect(*endpoint_,
			  [this, self](boost::system::error_code ec) {
			    if (ec) {
			      if (ec == boost::asio::error::operation_aborted)
				return;
			      // Some error occurred on connection
			      // attempt.  Close the socket and
			      // schedule a later connection attempt.
			      std::cout <<
				"Session: could not connect to endpoint: " <<
				ec.message() << '.' << std::endl;
			      socket_.close();
			      //do_connect_later();
			      return;
			    }
			    // Connection succeeded.  Schedule a
			    // reception as soon as input data is
			    // available.
			    start();
			  });
}
