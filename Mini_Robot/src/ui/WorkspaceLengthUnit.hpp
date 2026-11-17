#pragma once
#include <QString>
#include <QLocale>

namespace micro_robot {
enum class WorkspaceLengthUnit { Meters, Centimeters };
inline constexpr double WorkspaceCentimetersPerMeter = 100;
[[nodiscard]] inline double workspaceUnitsPerMeter(WorkspaceLengthUnit unit)
{ return unit == WorkspaceLengthUnit::Centimeters ? WorkspaceCentimetersPerMeter : 1; }
[[nodiscard]] inline QString workspaceUnitLabel(WorkspaceLengthUnit unit)
{ return unit == WorkspaceLengthUnit::Centimeters ? QStringLiteral("cm") : QStringLiteral("m"); }
[[nodiscard]] inline QString workspaceLengthText(double value)
{ return QLocale::c().toString(value,'g',QLocale::FloatingPointShortest); }
// Accept the operator locale and portable dot decimals, never grouped numbers.
[[nodiscard]] inline double parseWorkspaceLength(const QString& text, bool* ok)
{
    auto locale=QLocale(); locale.setNumberOptions(QLocale::RejectGroupSeparator);
    const double value=locale.toDouble(text.trimmed(),ok);
    if (*ok) return value;
    auto portable=QLocale::c(); portable.setNumberOptions(QLocale::RejectGroupSeparator);
    return portable.toDouble(text.trimmed(),ok);
}
} // namespace micro_robot
