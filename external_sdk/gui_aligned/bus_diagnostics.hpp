#pragma once
#include <cstdint>

namespace irc_step {
// SDK 결과와 응답 누락을 분리한다. SyncWrite 성공은 수신 확인이 아니다.
struct BusDiagnostics {
    bool valid{false};
    int result{0};
    std::uint32_t missing_ids{0};
};
}
