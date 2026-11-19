#pragma once

#include <chrono>

namespace rmrrt {

class StopWatch
{
public:
  StopWatch() { start(); }

  void start() { start_ = Clock::now(); }
  double elapsed() const { return std::chrono::duration<double>(Clock::now() - start_).count(); }

private:
  using Clock = std::chrono::steady_clock;
  Clock::time_point start_;
};

}  // namespace rmrrt
