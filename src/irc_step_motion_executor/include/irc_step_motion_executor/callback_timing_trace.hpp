#pragma once

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

namespace irc_step_motion_executor
{

// Single executor thread only. Nested scopes overlap; do not sum their durations.
// Storage is allocated before callbacks run; file writes happen only at shutdown.
class CallbackTimingTrace
{
public:
  struct Record
  {
    const char * callback;
    std::int64_t start_ns;
    std::int64_t end_ns;
    std::int64_t cpu_ns;
    bool active;
  };

  void configure(const std::string & path, std::size_t capacity = 262144)
  {
    records_.reserve(capacity);
    output_.open(path, std::ios::app);
    if (!output_) {
      throw std::runtime_error("cannot open callback timing trace: " + path);
    }
    capacity_ = capacity;
    enabled_ = true;
  }

  ~CallbackTimingTrace()
  {
    if (!enabled_) {return;}
    output_ << "# session,records," << records_.size() << ",dropped," << dropped_ << '\n';
    output_ << "callback,start_ms,end_ms,wall_ms,thread_cpu_ms,active\n";
    output_ << std::fixed << std::setprecision(6);
    for (const auto & row : records_) {
      output_ << row.callback << ',' << row.start_ns / 1e6 << ',' << row.end_ns / 1e6
              << ',' << (row.end_ns - row.start_ns) / 1e6 << ',' << row.cpu_ns / 1e6
              << ',' << row.active << '\n';
    }
  }

  class Scope
  {
  public:
    Scope(CallbackTimingTrace & trace, const char * name, bool active)
    : trace_(trace.enabled_ ? &trace : nullptr), name_(name), active_(active)
    {
      if (trace_) {
        start_ns_ = wall_now_ns();
        cpu_start_ns_ = cpu_now_ns();
      }
    }

    ~Scope()
    {
      if (!trace_) {return;}
      const auto cpu_end_ns = cpu_now_ns();
      const auto end_ns = wall_now_ns();
      if (trace_->records_.size() < trace_->capacity_) {
        trace_->records_.push_back({name_, start_ns_, end_ns,
          cpu_start_ns_ < 0 || cpu_end_ns < 0 ? -1000000 : cpu_end_ns - cpu_start_ns_,
          active_});
      } else {
        ++trace_->dropped_;
      }
    }

    Scope(const Scope &) = delete;
    Scope & operator=(const Scope &) = delete;

  private:
    static std::int64_t wall_now_ns()
    {
      return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static std::int64_t cpu_now_ns()
    {
      timespec value{};
      if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &value) != 0) {return -1;}
      return static_cast<std::int64_t>(value.tv_sec) * 1000000000 + value.tv_nsec;
    }

    CallbackTimingTrace * trace_;
    const char * name_;
    bool active_;
    std::int64_t start_ns_{0};
    std::int64_t cpu_start_ns_{0};
  };

private:
  bool enabled_{false};
  std::size_t capacity_{0};
  std::uint64_t dropped_{0};
  std::vector<Record> records_;
  std::ofstream output_;
};

}  // namespace irc_step_motion_executor
