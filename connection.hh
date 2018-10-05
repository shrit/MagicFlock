# include <boost/asio.hpp>
# include <boost/optional.hpp>



class TCPClient
  :public std::enable_shared_from_this<TCPClient> {
  
  using tcp                = boost::asio::ip::tcp;
  using socket_type        = tcp::socket;
  using endpoint_type      = boost::asio::ip::tcp::endpoint;

  
public:
  TCPClient(socket_type&&				socket,
	    const boost::optional<endpoint_type>&	ep = boost::none);	        

  void connect();

  void start() {
    /*  Postion sender to add here, Create a class for the position
     otherwise use the controlller class directley */
    //    status_sender_.start();
    //  do_read_header();
  }
  
  
  
private:
 
  socket_type socket_;
  boost::optional<endpoint_type>	endpoint_;
};


////////////////
// TCP Server //
////////////////


// TODO: reimplement the server 


class TCPServer {

  using tcp = boost::asio::ip::tcp;
  
public:
  
  struct TCPConnection : std::enable_shared_from_this<TCPConnection> {
    TCPServer* server_;
    tcp::socket socket_;
    bool write_pending_;

    // streambuf to send messages.

    //std::ostream ostream_;

    TCPConnection(TCPServer* server, tcp::socket&& socket)
      : server_{server}, socket_{std::move(socket)},
	write_pending_{false} {

    }
    // Send a string that must not contain a new line.
    template<class object>
    void send(object data) {
      std::vector<object> msg;
      msg.push_back(data);
      boost::asio::buffer(msg);
      if (write_pending_)
	return;

      auto shared = shared_from_this();
      boost::asio::async_write(socket_, boost::asio::buffer(msg),
	[shared](const boost::system::error_code& e,
			  std::size_t) {
	  shared->write_pending_ = false;
	  if (e) {
	    boost::system::error_code ignored;
	    std::cerr << "Error sending to "
		      << shared->socket_.remote_endpoint(ignored)
		      << ": " << e << ", closing...\n";
	    //	    shared->server_->destroy(shared);
	  }
	});
    //   for(const auto& it : msg){
    // 	if(it == end(msg)){
    // 	  assert(!"no thing to send, empty vector");
    // 	  return;
    // 	}
    // 	msg.erase(it);
    //   // }
      
       write_pending_ = true;
     }

  };
  
  using connection_pointer = std::shared_ptr<TCPConnection>;
  
  TCPServer()
    :service_{},
     acceptor_{service_},
     socket_to_accept_{service_},
     connection_{}  {
     }


  void start(const tcp::endpoint& endpoint)
  {
    acceptor_.open(endpoint.protocol());
    acceptor_.bind(endpoint);
    acceptor_.listen();
    accept();
    
  }

  void accept()
  {
    
    acceptor_.async_accept(socket_to_accept_,
      [this](boost::system::error_code ec) {
	if (!ec)
	  connection_ = std::make_shared<TCPConnection>(this, std::move(socket_to_accept_));	
	accept();
      });
  }

  //  void destroy(){}

  
  
  template<class object>
  void send(object data)
  {
    service_.post([this, data](){
		    connection_->send(data);      
		  });   
  }
  
  void poll() {
    service_.run();
  }
  void stop() {
    service_.stop();
  }
      
private:
  
  boost::asio::io_service service_;
  tcp::acceptor acceptor_;
  tcp::socket socket_to_accept_;
  connection_pointer connection_;

  
};
