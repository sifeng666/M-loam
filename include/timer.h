//
// Created by ziv on 2020/10/8.
//

#ifndef MLOAM_TIMER_H
#define MLOAM_TIMER_H

#include <iostream>
#include <chrono>
#include <string>
#include <atomic>

static std::atomic_int16_t timerCount{ 0 };

// use for function timer, total time consume
//[[maybe_unused]] auto timeConsumeFunc = [](auto&& func, auto&&... params) {
//    using clock_type = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;
//    clock_type tp1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
//    auto time1 = tp1.time_since_epoch().count();
//
//    std::forward<decltype(func)>(func)(std::forward<decltype(params)>(params)...);
//
//    clock_type tp2 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
//    auto time2 = tp2.time_since_epoch().count();
//    std::cout << "Function Consume time:" << time2 - time1 << "ms" << std::endl;
//};


class TimeCounter {
private:
    using clock_type = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;
    long long int startTime;
public:
    explicit TimeCounter() {
        clock_type tp1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        startTime = tp1.time_since_epoch().count();
    }
    long long int count() {
        return std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()).time_since_epoch().count() - startTime;
    }
};

// class for timer, manual timing
class Timer {
private:
    using clock_type = std::chrono::time_point<std::chrono::system_clock, std::chrono::milliseconds>;
    long long int startTime;
    std::string name;
    bool needDct;
public:
    explicit Timer(std::string _name, bool _needDct = false) {
        name = _name;
        needDct = _needDct;
        clock_type tp1 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        startTime = tp1.time_since_epoch().count();
    }
    void count() {
        clock_type tp2 = std::chrono::time_point_cast<std::chrono::milliseconds>(std::chrono::system_clock::now());
        std::cout << "Timer_" << name << "_" << timerCount++ << " count: " << tp2.time_since_epoch().count() - startTime << "msec" << std::endl;
    }
};

#endif //MLOAM_TIMER_H
