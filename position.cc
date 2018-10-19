# include "position.hh"
# include <vector>

PositionSender::PositionSender(socket_type& socket)
  : socket_{socket},
    send_timer_{socket.get_io_service()}
{}
  

void PositionSender::start(Controller& controller)
{
  send_position(Controller& controller);
}


void PositionSender::send_position(Controller& controller)
{
  //recover and create the message for the position
  // use vector as message type

  std::vector<double> msg;
  
  msg.push_back(controller.get_position_ned().position.down_m);
  msg.push_back(controller.get_position_ned().position.east_m);
  msg.push_back(controller.get_position_ned().position.north_m);
    
     async_write(socket_,
		  boost::asio::buffer(msg,
				      msg.size(),
		  [this](error_code	ec,
			 std::size_t	/* length */) {
		    if (not ec) {
		      // Wait period milliseconds before sending the next
		      // status report
		      auto timeout = boost::posix_time::milliseconds(period);
		      send_timer_.expires_from_now(timeout);
		      send_timer_.async_wait([this](error_code ec) {
			  if (not ec)
			    this->send_position();
			});
		    } else {
		      std::cerr << "Error in send_status: "
				<< ec.message() << std::endl;
		      // Connection has been lost, try to reconnect
		      session_.connect();
		      }
		    });
		  
}
