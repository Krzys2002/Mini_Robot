#pragma once
// Allocation counter shared by the host tests that prove a path allocates nothing (#161
// codec and session, #163 BLE lifecycle). ProtocolFuzzTest.cpp replaces the global
// allocator of the test binary once and counts through these variables; other tests only
// open an AllocationCount scope.
#include <atomic>
#include <cstddef>

namespace codecell::testing {
inline std::atomic<bool> gCountingAllocations{false};
inline std::atomic<std::size_t> gAllocations{0};

class AllocationCount {
public:
    AllocationCount() noexcept
    {
        gAllocations = 0;
        gCountingAllocations = true;
    }
    ~AllocationCount() { gCountingAllocations = false; }
    AllocationCount(const AllocationCount&) = delete;
    AllocationCount& operator=(const AllocationCount&) = delete;
    [[nodiscard]] std::size_t count() const noexcept { return gAllocations.load(); }
};
} // namespace codecell::testing
