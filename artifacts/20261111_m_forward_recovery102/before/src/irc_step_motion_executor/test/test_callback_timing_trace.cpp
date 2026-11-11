#include "irc_step_motion_executor/callback_timing_trace.hpp"

#include <gtest/gtest.h>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <thread>
#include <unistd.h>

using irc_step_motion_executor::CallbackTimingTrace;

TEST(CallbackTimingTrace, BuffersUntilShutdownAndBoundsStorage)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("step_callback_trace_" + std::to_string(getpid()) + ".csv");
  std::filesystem::remove(path);
  {
    CallbackTimingTrace trace;
    trace.configure(path.string(), 2);
    {
      CallbackTimingTrace::Scope outer(trace, "poll_timer", true);
      CallbackTimingTrace::Scope inner(trace, "head_update", true);
    }
    {CallbackTimingTrace::Scope dropped(trace, "heartbeat", false);}
    EXPECT_EQ(std::filesystem::file_size(path), 0u);
  }
  std::ifstream input(path);
  const std::string text{std::istreambuf_iterator<char>(input), {}};
  EXPECT_NE(text.find("records,2,dropped,1"), std::string::npos);
  EXPECT_NE(text.find("head_update,"), std::string::npos);
  EXPECT_NE(text.find("poll_timer,"), std::string::npos);
  EXPECT_EQ(text.find("heartbeat,"), std::string::npos);
  std::filesystem::remove(path);
}

TEST(CallbackTimingTrace, SeparatesWallTimeFromThreadCpuTime)
{
  const auto path = std::filesystem::temp_directory_path() /
    ("step_callback_cpu_" + std::to_string(getpid()) + ".csv");
  std::filesystem::remove(path);
  {
    CallbackTimingTrace trace;
    trace.configure(path.string());
    CallbackTimingTrace::Scope timing(trace, "wait", false);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::ifstream input(path);
  std::string field;
  std::getline(input, field);
  std::getline(input, field);
  for (int i = 0; i < 3; ++i) {std::getline(input, field, ',');}
  std::getline(input, field, ',');
  const auto wall_ms = std::stod(field);
  std::getline(input, field, ',');
  const auto cpu_ms = std::stod(field);
  EXPECT_GE(wall_ms, 19.0);
  EXPECT_GE(cpu_ms, 0.0);
  EXPECT_LT(cpu_ms, wall_ms);
  std::filesystem::remove(path);
}
