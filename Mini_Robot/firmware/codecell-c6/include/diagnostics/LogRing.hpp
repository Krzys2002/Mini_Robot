#pragma once
// Bounded, non-blocking local logging (#164; #156 B12/B13 and the overload policy).
// Portable C++20: the serial port is an injected sink.
//
// - LogRing holds at most timing::kLogRingBytes of complete lines. A line that does not fit
//   whole is dropped (the newest loses) and counted; a caller never waits.
// - ByteBudget is a token bucket that caps the drained average at
//   timing::kLogBytesPerSecondMax, with at most one second of burst.
// - drainLog() writes what the ring holds, the budget allows and the sink says it can take
//   without blocking. A sink that takes nothing (no USB host, or a host that stopped reading)
//   only fills the ring, and later lines are dropped and counted.
//
// Threading: not thread-safe. The production image pushes and drains only from the Arduino
// loopTask (priority 1, the B13 log drain). Supervisor, control and BLE callbacks never log:
// they publish atomic counters that the reporter formats.
#include "TimingBudget.hpp"
#include "runtime/RuntimeTypes.hpp"
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <string_view>

namespace codecell::diagnostics {

using runtime::Instant;

struct LogStats {
    std::uint32_t lines{0};         // Lines accepted into the ring.
    std::uint32_t droppedLines{0};  // Lines that did not fit and were dropped whole.
    std::uint32_t droppedBytes{0};  // Their bytes, terminators included.
    std::uint32_t writtenBytes{0};  // Bytes the sink accepted.
    std::size_t highWater{0};       // Most bytes held at once.
};

class LogRing {
public:
    static constexpr std::size_t kCapacity = timing::kLogRingBytes;

    // Appends `line` and a '\n' as one unit, or drops both. Returns whether it was kept.
    bool push(std::string_view line) noexcept
    {
        const std::size_t needed = line.size() + 1;
        if (needed > kCapacity - size_) {
            ++stats_.droppedLines;
            stats_.droppedBytes += static_cast<std::uint32_t>(std::min<std::size_t>(needed, std::numeric_limits<std::uint32_t>::max()));
            return false;
        }
        for (const char c : line) put(c);
        put('\n');
        ++stats_.lines;
        stats_.highWater = std::max(stats_.highWater, size_);
        return true;
    }

    // The oldest bytes as one contiguous run (it ends at the buffer end when the data wraps).
    [[nodiscard]] std::span<const char> readable() const noexcept
    {
        const std::size_t run = std::min(size_, kCapacity - head_);
        return {buffer_.data() + head_, run};
    }

    // Forgets the first `count` readable bytes (clamped to what is held).
    void consume(std::size_t count) noexcept
    {
        count = std::min(count, size_);
        head_ = (head_ + count) % kCapacity;
        size_ -= count;
        stats_.writtenBytes += static_cast<std::uint32_t>(count);
    }

    [[nodiscard]] std::size_t size() const noexcept { return size_; }
    [[nodiscard]] bool empty() const noexcept { return size_ == 0; }
    [[nodiscard]] const LogStats& stats() const noexcept { return stats_; }

private:
    void put(char c) noexcept
    {
        buffer_[(head_ + size_) % kCapacity] = c;
        ++size_;
    }

    std::array<char, kCapacity> buffer_{};
    std::size_t head_{0};
    std::size_t size_{0};
    LogStats stats_{};
};

// Token bucket in bytes. The first call starts it full (one second of burst).
class ByteBudget {
public:
    static constexpr std::uint64_t kMicrosecondsPerSecond = 1'000'000;

    constexpr ByteBudget(std::size_t bytesPerSecond, std::size_t burstBytes) noexcept
        : rate_(bytesPerSecond), burst_(burstBytes), tokens_(burstBytes)
    {
    }

    // Bytes that may be written at `now`. A clock that does not advance adds nothing.
    [[nodiscard]] std::size_t available(Instant now) noexcept
    {
        if (!last_) last_ = now;
        if (now > *last_) {
            // Carry the sub-byte remainder, so the long-run average is exact.
            credit_ += static_cast<std::uint64_t>((now - *last_).count()) * rate_;
            last_ = now;
            const auto whole = credit_ / kMicrosecondsPerSecond;
            credit_ %= kMicrosecondsPerSecond;
            tokens_ = static_cast<std::size_t>(std::min<std::uint64_t>(burst_, tokens_ + whole));
            if (tokens_ == burst_) credit_ = 0;
        }
        return tokens_;
    }

    void spend(std::size_t bytes) noexcept { tokens_ -= std::min(bytes, tokens_); }

private:
    std::uint64_t rate_;
    std::size_t burst_;
    std::size_t tokens_;
    std::uint64_t credit_{0};
    std::optional<Instant> last_{};
};

// The serial port, seen by the drain. Both calls must return at once.
class ILogSink {
public:
    virtual ~ILogSink() = default;
    // Bytes the port accepts now without waiting (0 when no host reads it).
    [[nodiscard]] virtual std::size_t writable() noexcept = 0;
    // Writes at most `bytes.size()` bytes without waiting; returns how many were taken.
    [[nodiscard]] virtual std::size_t write(std::span<const char> bytes) noexcept = 0;
};

inline constexpr std::size_t kLogBurstBytes = timing::kLogBytesPerSecondMax;
// Periodic report lines are produced only while the ring holds at most this much (once it has
// drained), so that a maintenance reply and a status-change line always find room: the
// reporter waits, the console does not. Without a reading host the round simply stalls.
inline constexpr std::size_t kReportBackpressureBytes = 0;

// One drain pass: at most two writes (the ring may wrap). Returns the bytes written.
inline std::size_t drainLog(LogRing& ring, ByteBudget& budget, ILogSink& sink, Instant now) noexcept
{
    std::size_t total = 0;
    for (int pass = 0; pass < 2 && !ring.empty(); ++pass) {
        const auto run = ring.readable();
        const std::size_t allowed = std::min({run.size(), budget.available(now), sink.writable()});
        if (allowed == 0) break;
        const std::size_t written = std::min(sink.write(run.first(allowed)), allowed);
        ring.consume(written);
        budget.spend(written);
        total += written;
        if (written < allowed) break;
    }
    return total;
}
} // namespace codecell::diagnostics
