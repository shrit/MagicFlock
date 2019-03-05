#pragma once

# include <iostream>
# include <sstream>

# include <boost/filesystem.hpp>
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
  {
    
    boost::posix_time::ptime now = boost::posix_time::second_clock::local_time();
    
    std::stringstream date_stream, time_stream; 
    
    date_stream << now.date();
    time_stream << now.time_of_day();
    
    boost::filesystem::create_directory("../log/" + date_stream.str());
    file_.open("../log/" + date_stream.str() + "/" + time_stream.str(),
		 boost::filesystem::ofstream::app|boost::filesystem::ofstream::out);
    
  }
  
  template<typename T> Log &operator<<(const T &x)
  {
    /*  Print to the standard output */
    s_ << x;
    
    /*  Also make a copy of the standard output to a file */
    
    file_ << x;
    file_.flush();
    
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
	file_.close();
    }

    Log(const Log &) = delete;
    void operator=(const Log &) = delete;

protected:
    enum LogLevel { Debug, Info, Warn, Err } _log_level = LogLevel::Debug;

private:
  boost::filesystem::ofstream file_;
  std::stringstream s_;
  
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

