#pragma once

# include <iostream>
# include <sstream>
# include <fstream>
# include <experimental/filesystem>

# include <boost/date_time/posix_time/posix_time.hpp>


// For release builds, don't show debug printfs, and just discard it into the NullStream.
class NullStream {
public:
    template<typename TPrintable> NullStream &operator<<(TPrintable const &)
    {
        /* no-op */
        static NullStream nothing;
        return nothing;
    }
};

enum class Color { RED, GREEN, YELLOW, BLUE, GRAY, RESET };

void set_color(Color color);

class Log {
  
public:
  Log() :
    s_()
  {}
  
  void init ();
  
  template<typename T> Log &operator<<(const T &x)
  {
    /*  Print to the standard output */
    s_ << x;    
    return *this;
    }
  
    virtual ~Log()
    {
      switch (_log_level) {
      case LogLevel::Debug:
	set_color(Color::GREEN);
	break;
      case LogLevel::Info:
	set_color(Color::BLUE);
	break;
      case LogLevel::Warn:
	set_color(Color::YELLOW);
	break;
      case LogLevel::Err:
	set_color(Color::RED);
	break;
      }
      
      boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
      
      std::cout << "[" << now.time_of_day();

        switch (_log_level) {
	case LogLevel::Debug:
	  std::cout << "|Debug] ";
	  break;
	case LogLevel::Info:
	  std::cout << "|Info ] ";
	  break;
	case LogLevel::Warn:
	  std::cout << "|Warn ] ";
	  break;
	case LogLevel::Err:
	  std::cout << "|Error] ";
	  break;
        }
	
        set_color(Color::RESET);
	
	std::cout << s_.str() << std::endl;
	
	/*  Also make a copy of the standard output to a file */
	
	file_.open(file_name_,  std::ios::out | std::ios::app );
	file_ << s_.rdbuf();
	file_.flush();
	file_.close();	

    }

    Log(const Log &) = delete;
    void operator=(const Log &) = delete;

protected:
    enum LogLevel { Debug, Info, Warn, Err } _log_level = LogLevel::Debug;  
private:
  std::ofstream file_;
  std::string file_name_;
  std::stringstream s_;
  std::stringstream date_stream_, time_stream_; 
  
};

class LogDebug : public Log {
public:
    LogDebug() : Log()
    {
        _log_level = LogLevel::Debug;

    }
};

class LogInfo : public Log {
public:
    LogInfo() : Log()
    {
        _log_level = LogLevel::Info;
    }
};

class LogWarn : public Log {
public:
    LogWarn() : Log()
    {
        _log_level = LogLevel::Warn;
    }
};

class LogErr : public Log {
public:
    LogErr() : Log()
    {
        _log_level = LogLevel::Err;
    }
};
