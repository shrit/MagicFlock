# include <boost/asio.hpp>
# include "controller.hh"

class PositionSender {
  
  using error_code   = boost::system::error_code;
  using timer_type   = boost::asio::deadline_timer;    
  
  
public:
  using io_service_type = boost::asio::io_service;
  using endpoint_type   = boost::asio::ip::tcp::endpoint;
  using socket_type     = boost::asio::ip::tcp::socket;
  
  PositionSender(socket_type&		socket);
  
  void start(Controller& controller);

private:
 void	send_position(Controller& controller);
  
  socket_type&	socket_;
  
  timer_type		send_timer_;
  
};
