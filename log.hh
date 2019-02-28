#pragma once

#include <sstream>
#include <iostream>
#include <ctime>

#define __FILENAME__ __FILE__

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

#if DEBUG
#define LogDebug() LogDebugDetailed(__FILENAME__, __LINE__)
#else
#define LogDebug() NullStream()
#endif

#define LogInfo() LogInfoDetailed(__FILENAME__, __LINE__)
#define LogWarn() LogWarnDetailed(__FILENAME__, __LINE__)
#define LogErr() LogErrDetailed(__FILENAME__, __LINE__)


enum class Color { RED, GREEN, YELLOW, BLUE, GRAY, RESET };

void set_color(Color color);

class LogDetailed {
public:
    LogDetailed(const char *filename, int filenumber) :
        _s(),
        _caller_filename(filename),
        _caller_filenumber(filenumber)
    {}

    template<typename T> LogDetailed &operator<<(const T &x)
    {
        _s << x;
        return *this;
    }

    virtual ~LogDetailed()
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

        // Time output taken from:
        // https://stackoverflow.com/questions/16357999#answer-16358264
        time_t rawtime;
        time(&rawtime);
        struct tm *timeinfo = localtime(&rawtime);
        char time_buffer[10]{}; // We need 8 characters + \0
        strftime(time_buffer, sizeof(time_buffer), "%I:%M:%S", timeinfo);
        std::cout << "[" << time_buffer;

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

        std::cout << _s.str();
        std::cout << " (" << _caller_filename << ":" << _caller_filenumber << ")";

        std::cout << std::endl;
    }

    LogDetailed(const dronecode_sdk::LogDetailed &) = delete;
    void operator=(const dronecode_sdk::LogDetailed &) = delete;

protected:
    enum LogLevel { Debug, Info, Warn, Err } _log_level = LogLevel::Debug;

private:
    std::stringstream _s;
    const char *_caller_filename;
    int _caller_filenumber;
};

class LogDebugDetailed : public LogDetailed {
public:
    LogDebugDetailed(const char *filename, int filenumber) : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Debug;
    }
};

class LogInfoDetailed : public LogDetailed {
public:
    LogInfoDetailed(const char *filename, int filenumber) : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Info;
    }
};

class LogWarnDetailed : public LogDetailed {
public:
    LogWarnDetailed(const char *filename, int filenumber) : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Warn;
    }
};

class LogErrDetailed : public LogDetailed {
public:
    LogErrDetailed(const char *filename, int filenumber) : LogDetailed(filename, filenumber)
    {
        _log_level = LogLevel::Err;
    }
};

