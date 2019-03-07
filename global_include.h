#pragma once

#define UNUSED(x) (void)(x)

#include <chrono>
#include <thread>

// Instead of using the constant from math.h or cmath we define it ourselves. This way
// we don't import all the other C math functions and make sure to use the C++ functions
// from the standard library (e.g. std::abs() instead of abs()).
#ifndef M_PI
constexpr double M_PI = 3.14159265358979323846;
#endif

#ifndef M_PI_F
constexpr float M_PI_F = float(M_PI);
#endif


typedef std::chrono::time_point<std::chrono::steady_clock> dl_time_t;

class Time {
public:
    Time();
    virtual ~Time();

    virtual dl_time_t steady_time();
    double elapsed_s();
    double elapsed_since_s(const dl_time_t &since);
    dl_time_t steady_time_in_future(double duration_s);
    void shift_steady_time_by(dl_time_t &time, double offset_s);

    virtual void sleep_for(std::chrono::hours h);
    virtual void sleep_for(std::chrono::minutes m);
    virtual void sleep_for(std::chrono::seconds s);
    virtual void sleep_for(std::chrono::milliseconds ms);
    virtual void sleep_for(std::chrono::microseconds us);
    virtual void sleep_for(std::chrono::nanoseconds ns);
};

class FakeTime : public Time {
public:
    FakeTime();

    virtual ~FakeTime();
    virtual dl_time_t steady_time() override;
    virtual void sleep_for(std::chrono::hours h) override;
    virtual void sleep_for(std::chrono::minutes m) override;
    virtual void sleep_for(std::chrono::seconds s) override;
    virtual void sleep_for(std::chrono::milliseconds ms) override;
    virtual void sleep_for(std::chrono::microseconds us) override;
    virtual void sleep_for(std::chrono::nanoseconds ns) override;

private:
    std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds> _current{};
    void add_overhead();
};

double to_rad_from_deg(double deg);
double to_deg_from_rad(double rad);

float to_rad_from_deg(float deg);
float to_deg_from_rad(float rad);



