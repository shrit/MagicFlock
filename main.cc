/**
 * @file main.cc
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-07-21
 */
extern "C"
{
  # include <curses.h>         
}

# include <string>
# include <sstream>
# include <iostream>
# include <future>
# include <chrono>
# include <vector>
# include <boost/asio/posix/stream_descriptor.hpp>
# include <boost/program_options.hpp>
# include <boost/asio.hpp>

# include "controller.hh"

using namespace dronecode_sdk;
using std::this_thread::sleep_for;
using std::chrono::milliseconds;
using std::chrono::seconds;

/**
 * TODO: Manage the error exit
 * TODO: Comment the code 
 * TODO: Recover all the info of the telemetry
 * TODO: print well all the output using ncurses
 * TODO: use boost log to create a log
 * TODO: also understand the normal log provided by the library
 * TODO: implement camera receive video, start, and stop, take photo, etc..
 * TODO: Add a timer for data that are sent, also use timer in the client side
 * to ask for data each 50 ms
 * TODO: Create asyncronous clin
*/



// inline void action_error_exit(ActionResult result, const std::string &message)
// {
//     if (result != ActionResult::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << action_result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }

// // Handles Offboard's result
// inline void offboard_error_exit(Offboard::Result result, const std::string &message)
// {
//     if (result != Offboard::Result::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << Offboard::result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }

// // Handles connection result
// inline void connection_error_exit(ConnectionResult result, const std::string &message)
// {
//     if (result != ConnectionResult::SUCCESS) {
//         std::cerr << ERROR_CONSOLE_TEXT << message << connection_result_str(result)
//                   << NORMAL_CONSOLE_TEXT << std::endl;
//         exit(EXIT_FAILURE);
//     }
// }


template <class Handler>
void async_wait(boost::asio::posix::stream_descriptor& in, Handler&& handler) {
  in.async_read_some(boost::asio::null_buffers(),
		     [&](boost::system::error_code ec,
			 size_t /* bytes_transferred */) {
		       if (not ec)
			 handler();
		       else
			   std::cerr << "Error: " << ec.message() << std::endl;
		       
			 async_wait(in, std::forward<Handler>(handler));
		     });
}


class TCPClient
{
  
  using tcp = boost::asio::ip::tcp;
 
public:
  
  
private:
  
};


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
      for(const auto& it : msg){
	if(it == end(msg)){
	  assert(!"no thing to send, empty vector");
	  return;
	}
	msg.erase(it);
      }
      
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


void usage()
{
  
  std::cout << "Usage : " << std ::endl
    
	    << "-h  print out this help message" << std::endl
	    << "-v  print out the version" << std::endl
	    << "--verbose be more verbose" << std::endl
	    << "To control the quadcopter using keyboard use : " << std::endl
	    << " a : to arm" << std::endl
	    << " t : to takoff" << std::endl
	    << " l : to land" << std::endl
	    << " s : to activate offboard mode" << std::endl
	    << " key up    : to go forward" << std::endl
	    << " key right : to go right" << std::endl
	    << " key left  : to go left" << std::endl
	    << " key down  : to go backward" << std::endl
	    << " + : to turn clock wise" << std::endl
	    << " - : to turn counter clock wise" << std::endl;
  
}

int main(int argc, char** argv)
{

  std::string connection_url;
  std::stringstream ss;
  
  namespace po = boost::program_options;
  po::options_description option("Allowed");
  
  option.add_options()
    ("help,h", "Print this help message and exit" )				
    ("version,v", "Print the current version")
    ("Versbose,", "Be more verbose")
    ("udp",
     po::value<std::string>(&connection_url)->default_value("udp://:14540"),
     "Connection URL format should be: udp://[bind_host][:bind_port] \n, For example to connect to simulator use --udp udp://:14540")
    ("tcp",
     po::value<std::string>(&connection_url),"Connection URL format should be: tcp://[server_host][:server_port]")
    ("serial",
      po::value<std::string>(&connection_url),"Connection URL format should be: serial:///path/to/serial/dev[:baudrate]");

  
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, option), vm);
  po::notify(vm);
  
  if(vm.count("help")){
    std::cout << option << std::endl;
    exit(0);        
  }
    
  if(vm.count("version")){
    std::cout << "0.1v ";
    exit(0);        
  }
      
  boost::asio::io_service	io_service;

  TCPServer server;

  Controller controller;

  DronecodeSDK dc_;

  dronecode_sdk::Telemetry::PositionNED position;
  
  controller.connect_to_quad(dc_, connection_url);

  controller.discover_system(dc_);  
      
  
  System& system = dc_.system();
  
  auto telemetry_ = std::make_shared<dronecode_sdk::Telemetry>(system);
  auto offboard_  = std::make_shared<dronecode_sdk::Offboard>(system);
  auto action_    = std::make_shared<dronecode_sdk::Action>(system);
    
  controller.set_rate_result(telemetry_);
  controller.get_position(telemetry_);
  controller.quad_health(telemetry_);
  controller.get_position_ned(telemetry_);
  
  setlocale(LC_ALL, "");
  
  initscr();
  
  halfdelay(3);
  
  
  // Suppress automatic echoing
  noecho();
  
  // Do not translate the return key into newline
  nonl();
  
  // Capture special keystrokes (including the four arrow keys)
  keypad(stdscr, TRUE);
  refresh();
  
  int ch;
  
  boost::asio::posix::stream_descriptor in{io_service, 0};  

  server.start({boost::asio::ip::tcp::v4(), 8800});
  
  auto future = std::async(std::launch::async, [&server]() {
						 server.poll();
					       });  



  server.send(position);
  
    
  auto lambda = [&](){
		  
		  ch = getch(); 
		  		  
		  switch (ch) {
		  case KEY_UP:
		    printw("key_up");
		    controller.forward(offboard_);
		    break;
		  case KEY_DOWN:
		    printw("key_down");
		    controller.backward(offboard_);
		    break;
		  case KEY_LEFT:
		    printw("key_left");
		    controller.goLeft(offboard_);
		    break;
		  case KEY_RIGHT:
		    printw("key_right");
		    controller.goRight(offboard_);
		    break;
		  case 'u':    
		    printw("goUp");
		    controller.goUp(offboard_);
		    break;
		  case 'd':
		    printw("goDown");
		    controller.goDown(offboard_);
		    break;		    
		  case 't':
		    printw("take_off");
		    controller.takeoff(action_);
		    break;
		  case 'l':
		    printw("land");
		    controller.land(action_);
		    break;
		  case 'a':
		    printw("arming...");
		    controller.arm(action_);
		    sleep_for(seconds(2));
		    break;
		  case '+':
		    printw("turn to right");
		    controller.turnToRight(offboard_);		    
		    break;
		  case '-':
		    printw("turn to left");
		    controller.turnToLeft(offboard_);		    
		    break;		    
		  case 's':
		    controller.init_speed(offboard_);
		    controller.start_offboard_mode(offboard_);
		    break;
		    
		  default:		    
		    printw("key_NOT DEFINED: %c", ch);
		    endwin();
		  }		  

		};
    
  async_wait(in, lambda);

  io_service.run();
             
}
