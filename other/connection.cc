# include <connection.hh>


TCPClient::TCPClient(socket_type&&				socket,
		     const boost::optional<endpoint_type>&	ep = boost::none)
  : socket_(std::move(socket)),
    endpoint_(ep)
{}


void TCPClient::connect(Controller& controller)
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
			    start(Controller& controller);
			  });
}

void TCPClient::start(Controller& controller)
{  
  position_sender_.start(Controller& controller);
  do_read_msg();
}
  


void TCPClient::do_read_msg()
{


  std::vector<double> msg;
  auto	self(this->shared_from_this());
  boost::asio::
    async_read(socket_,
	       boost::asio::buffer(msg, msg.size()),
	       [this, self](boost::system::error_code	ec,
			    std::size_t		/* length */) {
		 if (not ec) {
		   
		   // msg is a rssi value received from ns3
		   parse_rssi_msg(msg);
		 }
		 else {
		   std::cerr << "Error in do_read_body: "
			     << ec.message() << std::endl;
		   handle_error(ec);
		 }
	       });    
  
}


void TCPClient::parse_rssi_msg(std::vector<double> msg)
{
  
  /*  to fill according to the message sent by ns3 */
  
}



/* move this part of the code to the NS3 */

// TCPServer::Server(boost::asio::io_service&	io_service,
// 		  uint16_t			port)   
//   :io_service_(io_service),
//    acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
//    socket_(io_service) {
//   do_accept();
// }


// void TCPServer::do_accept() {
//   acceptor_.async_accept(socket_,
// 			 [this](boost::system::error_code ec) {
// 			   if (not ec) {
// 			     sessions_.push_back
// 			       (std::make_shared<session_type>
// 				(std::move(socket_), flight_controller_));
// 			     sessions_.back()->start();
// 			   }
			   
// 			   do_accept();
// 			 });
// }
