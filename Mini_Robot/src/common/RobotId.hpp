#pragma once

#ifndef Q_MOC_RUN
// Qt 6.4 moc cannot parse GCC 13's C++20 comparison headers.
#include <compare>
#endif
#include <cstdint>
#include <functional>

namespace micro_robot {

// Application identity, independent of hardware addresses. All values, including
// zero, are valid; represent a missing identity with std::optional<RobotId>.
class RobotId final {
public:
    explicit constexpr RobotId(std::uint32_t value) noexcept : value_(value) {}

    [[nodiscard]] constexpr std::uint32_t value() const noexcept { return value_; }

    constexpr auto operator<=>(const RobotId&) const noexcept = default;

private:
    std::uint32_t value_;
};

} // namespace micro_robot

template <>
struct std::hash<micro_robot::RobotId> {
    [[nodiscard]] std::size_t operator()(micro_robot::RobotId id) const noexcept
    {
        return std::hash<std::uint32_t>{}(id.value());
    }
};
