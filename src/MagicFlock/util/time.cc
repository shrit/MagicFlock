#include "time.hpp"

#include <iostream>
#include <math.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

/*
 * Part of this code is borrowed from tansa.
 * Major modification is still required to be adapted to this project
 */

using namespace std;

static Clock defaultClock;
/**
 * Gets the current time
 * if since_epoch is set to true, then the current calendar time/time
 * since epoch will be used. this time is less precise
 * otherwise, the best monotonic clock source will be used
 * Credit to https://gist.github.com/jbenet/1087739 for some of this
 * function
*/
void
current_time(struct timespec* ts, bool since_epoch = false)
{
  clock_gettime(since_epoch ? CLOCK_REALTIME : CLOCK_MONOTONIC, ts);
}

void
timespec_add(struct timespec* result,
             const struct timespec* a,
             const struct timespec* b)
{
  result->tv_sec =
    a->tv_sec + b->tv_sec + (a->tv_nsec + b->tv_nsec >= 1000000000 ? 1 : 0);
  result->tv_nsec = (a->tv_nsec + b->tv_nsec) % 1000000000;
}

// computes stop - start
void
timespec_subtract(struct timespec* result,
                  const struct timespec* start,
                  const struct timespec* stop)
{
  if ((stop->tv_nsec - start->tv_nsec) < 0) {
    result->tv_sec = stop->tv_sec - start->tv_sec - 1;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec + 1000000000;
  } else {
    result->tv_sec = stop->tv_sec - start->tv_sec;
    result->tv_nsec = stop->tv_nsec - start->tv_nsec;
  }
}

void
time_init()
{
  defaultClock.simTimeValid = false;
  // This should force the usage of the wall clock
  defaultClock.starttime = Time::realNow();
}

Time::Time(int secs, int nsecs)
{
  this->val.tv_sec = secs;
  this->val.tv_nsec = nsecs;
}

Time::Time(double seconds)
{
  double intpart;
  double frac = modf(seconds, &intpart);

  this->val.tv_sec = intpart;
  this->val.tv_nsec = frac * 1000000000;
}

Time
Time::now()
{
  Time t = Time::realNow();

  if (defaultClock.simTimeValid) {
    Time dt = t.since(defaultClock.simRefTime);
    dt = dt.scale(defaultClock.simFactor);

    Time stime;
    timespec_add(&stime.val, &dt.val, &defaultClock.simTime.val);
    return stime;
  }

  return t;
}

Time
Time::realNow()
{
  struct timespec t;
  current_time(&t, true);

  Time ot;
  ot.val = t;
  return ot;
}

void
Time::setTime(const Time& t, double factor)
{
  defaultClock.simTimeValid = true;
  defaultClock.simTime = t;
  defaultClock.simRefTime = Time::realNow();
  defaultClock.simFactor = factor;
}

Time
Time::since(const Time& other) const
{
  Time diff;
  timespec_subtract(&diff.val, &other.val, &this->val);
  return diff;
}

Time
Time::sinceStart()
{
  return since(defaultClock.starttime);
}

Time
Time::add(const Time& rhs)
{
  Time sum;
  timespec_add(&sum.val, &this->val, &rhs.val);
  return sum;
}

Time
Time::subtract(const Time& rhs)
{
  Time diff;
  timespec_subtract(&diff.val, &rhs.val, &this->val);
  return diff;
}

uint64_t
Time::nanos() const
{
  return (uint64_t)val.tv_nsec + ((uint64_t)val.tv_sec * 1000000000);
}

uint64_t
Time::micros() const
{
  return ((uint64_t)val.tv_nsec / 1000) + ((uint64_t)val.tv_sec * 1000000);
}

uint64_t
Time::millis() const
{
  return ((uint64_t)val.tv_nsec / 1000000) + ((uint64_t)val.tv_sec * 1000);
}

double
Time::seconds() const
{
  return ((double)val.tv_sec) + (((double)val.tv_nsec) / 1000000000.0);
}

std::string
Time::dateString() const
{
  struct tm t;

  char buf[64];

  tzset();
  if (localtime_r(&(val.tv_sec), &t) == NULL)
    return "Unknown-" + std::to_string(val.tv_sec);

  strftime(buf, sizeof(buf), "%Y%m%d-%H_%M_%S", &t);

  return std::string(buf);
}

Rate::Rate(unsigned int hz)
  : lasttime(0, 0)
{
  this->hz = hz;
  this->lasttime = Time::now();
  this->uperiod = 1000000 / hz;
}

void
Rate::sleep()
{

  uint64_t done = Time::now().since(lasttime).micros();

  // Number of cycles completed since last time
  unsigned cycles = done / uperiod;

  // Sleep if we are still undertime
  if (cycles == 0) {
    usleep(uperiod - done);
    cycles++;
  } else {
    // cerr << "Loop too slow" << endl;
  }

  Time dt = Time(0, 1000 * uperiod).scale(cycles);
  lasttime = lasttime.add(dt);
}

Timer::Timer()
{
  // Nothing to do here
}

void
Timer::start()
{
  starting_time_ = std::chrono::steady_clock::now();
}

int
Timer::stop()
{
  std::chrono::steady_clock::time_point end_time =
    std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                              starting_time_);

  return elapsed_time.count();
}

std::string
Timer::stop_and_get_time()
{
  std::chrono::steady_clock::time_point end_time =
    std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed_time =
    std::chrono::duration_cast<std::chrono::duration<double>>(end_time -
                                                              starting_time_);
  int total = elapsed_time.count();
  int minutes = total / 60;
  int seconds = total % 60;
  int hours = minutes / 60;
  minutes = minutes % minutes;

  std::string time = std::to_string(hours) + ":" + std::to_string(minutes) +
                     ":" + std::to_string(seconds);

  return time;
}
