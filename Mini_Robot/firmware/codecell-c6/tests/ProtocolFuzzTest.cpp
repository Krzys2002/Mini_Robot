// Deterministic mutation, property and bounded fuzz tests of the #161 v1 command path.
//
// Every input lives in an exact-size heap buffer, so a host sanitizer build
// (-DCODECELL_FIRMWARE_HOST_SANITIZERS=ON, see the firmware README) reports any read past
// the span. A table-driven reference decoder written independently of V1Codec checks
// every verdict. A counting global allocator proves the decoder and the session allocate
// nothing. The fuzz run is reproducible: std::mt19937 output is fixed by the standard,
// and only its raw output is used (no implementation-defined distributions). Override the
// bounded defaults with CODECELL_PROTOCOL_FUZZ_SEED and CODECELL_PROTOCOL_FUZZ_ITERATIONS.
#include "ProtocolVectors.hpp"
#include "RuntimeFakes.hpp"
#include "protocol/V1CommandSession.hpp"
#include "runtime/FirmwareRuntime.hpp"
#include <gtest/gtest.h>
#include <algorithm>
#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <memory>
#include <new>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <vector>

using namespace codecell;
using namespace codecell::protocol::v1;
using namespace std::chrono_literals;
using runtime::Instant;
using runtime::MotionState;

// ---- Allocation counter (replaces the global allocator for this test binary) ---------
namespace {
std::atomic<bool> gCounting{false};
std::atomic<std::size_t> gAllocations{0};

class AllocationCount {
public:
    AllocationCount() noexcept
    {
        gAllocations = 0;
        gCounting = true;
    }
    ~AllocationCount() { gCounting = false; }
    AllocationCount(const AllocationCount&) = delete;
    AllocationCount& operator=(const AllocationCount&) = delete;
    [[nodiscard]] std::size_t count() const noexcept { return gAllocations.load(); }
};
} // namespace

// Every non-aligned form is replaced, so each allocation and its release use the same
// malloc/free pair (a sanitizer build checks this). Aligned forms keep the defaults.
namespace {
void* countedAllocation(std::size_t size) noexcept
{
    if (gCounting.load(std::memory_order_relaxed)) gAllocations.fetch_add(1, std::memory_order_relaxed);
    return std::malloc(size == 0 ? 1 : size);
}
void* countedAllocationOrThrow(std::size_t size)
{
    if (void* p = countedAllocation(size)) return p;
    throw std::bad_alloc{};
}
} // namespace
void* operator new(std::size_t size) { return countedAllocationOrThrow(size); }
void* operator new[](std::size_t size) { return countedAllocationOrThrow(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept { return countedAllocation(size); }
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept { return countedAllocation(size); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }

namespace {
constexpr CommandLimits kLimits{150, 3000, 200};
constexpr std::uint32_t kDefaultSeed = 161;
constexpr std::uint64_t kDefaultIterations = 200'000;
constexpr std::size_t kMaxFuzzLength = 3 * kMaxAttPayload;  // Short, exact, long and oversized writes.
constexpr std::size_t kOversizedLength = 64 * 1024;

std::uint64_t environmentOr(const char* name, std::uint64_t fallback)
{
    const char* text = std::getenv(name);
    return text ? std::strtoull(text, nullptr, 10) : fallback;
}

// An exact-size heap copy: one byte past the end is outside the allocation.
struct ExactBuffer {
    explicit ExactBuffer(std::span<const std::uint8_t> bytes)
        : size(bytes.size()), data(std::make_unique<std::uint8_t[]>(bytes.size()))
    {
        std::copy(bytes.begin(), bytes.end(), data.get());
    }
    [[nodiscard]] std::span<const std::uint8_t> span() const { return {data.get(), size}; }
    std::size_t size;
    std::unique_ptr<std::uint8_t[]> data;
};

// Independent reference: the spec's tables, not V1Codec's code paths.
struct Expected {
    bool frame;
    ResultCode code;
    std::uint8_t type;
    std::uint16_t sequence;
};
Expected reference(const std::vector<std::uint8_t>& b)
{
    if (b.size() < 2) return {false, ResultCode::MalformedLength, 0, 0};
    const std::uint16_t sequence = b.size() >= 4 ? static_cast<std::uint16_t>(b[2] + 256 * b[3]) : 0;
    if (b[0] != 1) return {false, ResultCode::UnsupportedVersion, b[1], sequence};
    static constexpr struct {
        std::uint8_t type;
        std::size_t length;
    } kTable[]{{1, 4}, {2, 10}, {3, 4}, {4, 4}};
    for (const auto& row : kTable) {
        if (row.type != b[1]) continue;
        if (b.size() != row.length) return {false, ResultCode::MalformedLength, b[1], sequence};
        return {true, ResultCode::Accepted, b[1], sequence};
    }
    return {false, ResultCode::UnknownMessageType, b[1], sequence};
}

void expectMatchesReference(const std::vector<std::uint8_t>& bytes)
{
    const ExactBuffer buffer(bytes);
    const auto decoded = decodeCommand(buffer.span());
    const auto expected = reference(bytes);
    ASSERT_EQ(decoded.frame.has_value(), expected.frame);
    if (decoded.frame) {
        ASSERT_EQ(static_cast<std::uint8_t>(decoded.frame->type), expected.type);
        ASSERT_EQ(decoded.frame->sequence, expected.sequence);
        if (decoded.frame->type == MessageType::Velocity) {
            ASSERT_EQ(static_cast<std::uint16_t>(decoded.frame->velocity.linearMillimetresPerSecond),
                      bytes[4] + 256 * bytes[5]);
            ASSERT_EQ(static_cast<std::uint16_t>(decoded.frame->velocity.angularMilliradiansPerSecond),
                      bytes[6] + 256 * bytes[7]);
            ASSERT_EQ(decoded.frame->velocity.validForMilliseconds, bytes[8] + 256 * bytes[9]);
        }
        return;
    }
    ASSERT_EQ(decoded.rejection, expected.code);
    ASSERT_EQ(decoded.echoType, expected.type);
    ASSERT_EQ(decoded.echoSequence, expected.sequence);
}

std::vector<std::vector<std::uint8_t>> commandVectors()
{
    std::vector<std::vector<std::uint8_t>> result;
    for (const auto& v : protocol::fixtures::loadVectors())
        if (v.kind == "stop" || v.kind == "velocity" || v.kind == "estop" || v.kind == "clear_estop")
            result.push_back(v.bytes);
    return result;
}
} // namespace

TEST(ProtocolMutationTest, EverySingleBitFlipOfEveryVectorMatchesTheReference)
{
    for (const auto& vector : protocol::fixtures::loadVectors()) {
        SCOPED_TRACE(vector.name);
        for (std::size_t bit = 0; bit < 8 * vector.bytes.size(); ++bit) {
            auto mutated = vector.bytes;
            mutated[bit / 8] = static_cast<std::uint8_t>(mutated[bit / 8] ^ (1U << (bit % 8)));
            expectMatchesReference(mutated);
        }
    }
}

TEST(ProtocolMutationTest, EveryByteValueAtEveryOffsetMatchesTheReference)
{
    for (const auto& vector : commandVectors()) {
        for (std::size_t at = 0; at < vector.size(); ++at) {
            for (int value = 0; value <= 0xFF; ++value) {
                auto mutated = vector;
                mutated[at] = static_cast<std::uint8_t>(value);
                expectMatchesReference(mutated);
            }
        }
    }
}

TEST(ProtocolMutationTest, DecodingReadsNoMoreThanTheLongestFrame)
{
    // Bytes past the longest command can never change a verdict: the decoder does a fixed
    // amount of work and never scans the input, however long it is.
    std::vector<std::uint8_t> oversized(kOversizedLength, 0x00);
    for (const auto& vector : commandVectors()) {
        std::copy(vector.begin(), vector.end(), oversized.begin());
        const ExactBuffer first(oversized);
        const auto a = decodeCommand(first.span());
        std::fill(oversized.begin() + kVelocityLength, oversized.end(), 0xA5);
        const ExactBuffer second(oversized);
        const auto b = decodeCommand(second.span());
        EXPECT_FALSE(a.frame);
        EXPECT_EQ(a.rejection, ResultCode::MalformedLength);
        EXPECT_EQ(a.rejection, b.rejection);
        EXPECT_EQ(a.echoType, b.echoType);
        EXPECT_EQ(a.echoSequence, b.echoSequence);
        std::fill(oversized.begin(), oversized.end(), 0x00);
    }
}

// A drive that records nothing, so only the code under test could allocate.
class QuietDrive final : public runtime::IDriveOutput {
public:
    bool configure(const drive::DriveProfile&, std::chrono::milliseconds) noexcept override { return true; }
    bool disable() noexcept override { return true; }
    bool command(const runtime::BodyVelocity&, Instant) noexcept override { return true; }
    runtime::StopRampStep rampToStop(Instant) noexcept override { return runtime::StopRampStep::AtRest; }
};

TEST(ProtocolMutationTest, DecoderSessionAndAuthorityNeverAllocate)
{
    runtime::fakes::FakeClock clock;
    runtime::fakes::FakePlatform platform;
    QuietDrive drive;
    runtime::fakes::FakeConfiguration configuration;
    runtime::FirmwareRuntime rt(runtime::RuntimeDependencies{clock, platform, drive, configuration, nullptr});
    ASSERT_EQ(rt.start(), runtime::StartResult::Running);
    struct View final : IRuntimeStatusView {
        const runtime::FirmwareRuntime& r;
        explicit View(const runtime::FirmwareRuntime& runtime) : r(runtime) {}
        runtime::RuntimeStatus status() const noexcept override { return r.status(); }
    } view{rt};
    CommandSession session(rt.commands(), view, kLimits);
    session.open();
    const auto vectors = commandVectors();
    const std::vector<std::uint8_t> oversized(kOversizedLength, 0x01);
    std::size_t allocations = 0;
    {
        AllocationCount counter;
        for (const auto& bytes : vectors) {
            (void)decodeCommand(bytes);
            (void)session.onCommandWrite(bytes, clock.nowMicroseconds());
            (void)session.onEmergencyStopWrite(bytes);
        }
        (void)session.onCommandWrite(oversized, clock.nowMicroseconds());
        (void)session.telemetry({});
        allocations = counter.count();
    }
    EXPECT_EQ(allocations, 0U);
}

// Random short, exact, long and oversized writes on both characteristics against the
// real runtime. Invariants: the verdict matches the reference; a rejected frame changes
// neither the motion state, the drive, nor lastAccepted; only a decodable command is
// ever accepted; and the Emergency Stop characteristic latches every time.
TEST(ProtocolFuzzTest, RandomInputCannotOverreadCrashOrChangeMotionWhenRejected)
{
    const auto seed = static_cast<std::uint32_t>(environmentOr("CODECELL_PROTOCOL_FUZZ_SEED", kDefaultSeed));
    const auto iterations = environmentOr("CODECELL_PROTOCOL_FUZZ_ITERATIONS", kDefaultIterations);
    SCOPED_TRACE("seed " + std::to_string(seed) + ", iterations " + std::to_string(iterations));
    std::mt19937 random(seed);
    const auto vectors = commandVectors();

    runtime::fakes::FakeClock clock;
    clock.time = 1s;
    runtime::fakes::FakePlatform platform;
    runtime::fakes::FakeDrive drive;
    runtime::fakes::FakeConfiguration configuration;
    configuration.value->commandValidityCeiling = std::chrono::milliseconds{kLimits.maxCommandValidityMilliseconds};
    runtime::FirmwareRuntime rt(runtime::RuntimeDependencies{clock, platform, drive, configuration, nullptr});
    ASSERT_EQ(rt.start(), runtime::StartResult::Running);
    struct View final : IRuntimeStatusView {
        const runtime::FirmwareRuntime& r;
        explicit View(const runtime::FirmwareRuntime& runtime) : r(runtime) {}
        runtime::RuntimeStatus status() const noexcept override { return r.status(); }
    } view{rt};
    CommandSession session(rt.commands(), view, kLimits);
    session.open();

    std::uint64_t accepted = 0;
    std::uint64_t latches = 0;
    for (std::uint64_t i = 0; i < iterations; ++i) {
        const std::uint32_t shape = random();
        std::vector<std::uint8_t> bytes;
        if (shape % 4 == 0) {
            bytes = vectors[random() % vectors.size()];  // Mutate a valid frame.
            const auto mutations = 1 + random() % 3;
            for (std::uint32_t m = 0; m < mutations && !bytes.empty(); ++m)
                bytes[random() % bytes.size()] = static_cast<std::uint8_t>(random());
            if (random() % 4 == 0) bytes.resize(random() % (kMaxFuzzLength + 1), 0x00);
        } else {
            bytes.resize(random() % (kMaxFuzzLength + 1));
            for (auto& byte : bytes) byte = static_cast<std::uint8_t>(random());
            if (!bytes.empty() && shape % 4 == 1) bytes[0] = kProtocolVersion;
            if (bytes.size() > 1 && shape % 4 == 1) bytes[1] = static_cast<std::uint8_t>(1 + random() % 4);
        }
        const ExactBuffer buffer(bytes);
        clock.advance(Instant{1ms});
        if (random() % 64 == 0) {  // Occasionally the Emergency Stop characteristic.
            const auto reply = session.onEmergencyStopWrite(buffer.span());
            ASSERT_EQ(static_cast<ResultCode>(reply[kResultCodeAt]), ResultCode::Accepted);
            ASSERT_EQ(rt.status().state, MotionState::EmergencyStop);
            ASSERT_FALSE(rt.status().outputEnabled);
            ++latches;
            continue;
        }
        const auto before = rt.status();
        const auto lastBefore = session.lastAcceptedSequence();
        const auto eventsBefore = drive.events.size();
        const auto reply = session.onCommandWrite(buffer.span(), clock.nowMicroseconds());
        const auto code = static_cast<ResultCode>(reply[kResultCodeAt]);
        const auto expected = reference(bytes);
        if (!expected.frame) {
            ASSERT_EQ(code, expected.code) << i;
        }
        if (code != ResultCode::Accepted) {
            ASSERT_EQ(rt.status().state, before.state) << i;
            ASSERT_EQ(rt.status().outputEnabled, before.outputEnabled) << i;
            ASSERT_EQ(drive.events.size(), eventsBefore) << i;
            ASSERT_EQ(session.lastAcceptedSequence(), lastBefore) << i;
        } else {
            ASSERT_TRUE(expected.frame) << i;
            ++accepted;
        }
        rt.safetyStep().step();  // Both tasks keep running, so expiry and stop ramps progress.
        rt.controlStep().step();
    }
    // The run exercised both outcomes, not only rejections.
    EXPECT_GT(accepted, 0U);
    EXPECT_GT(latches, 0U);
    EXPECT_NE(rt.status().state, MotionState::Fault);
}
