#ifndef TIMER_H
#define TIMER_H
#include <chrono>


class Timer
{
private:
    std::chrono::high_resolution_clock::time_point tp1_;
public:
    Timer()
    {

    }
    void start()
    {
        tp1_ = std::chrono::high_resolution_clock::now();
    }
    double end()
    {
        using namespace std::chrono;
        high_resolution_clock::time_point tp2 = high_resolution_clock::now();
        duration<double> dt = duration_cast<duration<double>>(tp2 - tp1_);
        return dt.count();
    }
};

#endif // TIMER_H
