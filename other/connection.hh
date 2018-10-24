# include <boost/asio.hpp>
# include <boost/optional.hpp>
# include "position.hh"
# include "controller.hh"

class TCPClient
  :public std::enable_shared_from_this<TCPClient> {
  
  using tcp                = boost::asio::ip::tcp;
  using socket_type        = tcp::socket;
  using endpoint_type      = boost::asio::ip::tcp::endpoint;

  
public:
  TCPClient(socket_type&&				socket,
	    const boost::optional<endpoint_type>&	ep = boost::none);	        

  void connect(Controller& controller);

  void start(Controller& controller);
  void do_read_msg();   /*  do read signal stregth */

  void parse_rssi_msg();
  
private:
  PositionSender position_sender_;
  socket_type socket_;
  boost::optional<endpoint_type>	endpoint_;
};


////////////////
// TCP Server //
////////////////

/*  move this part of the code to ns3 */


class TCPServer {
public:
  
  TCPServer(boost::asio::io_service&	io_service,
	 uint16_t			port);

private:
  void do_accept();
  
  boost::asio::io_service&			io_service_;
  tcp::acceptor				acceptor_;
  tcp::socket					socket_;
  
  /* to see how to make a list of connections if needed*/ 
  //  std::list<std::shared_ptr<session_type>>	sessions_;
};
