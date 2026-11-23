#pragma once

#include "ui/WorkspaceLengthUnit.hpp"

#include <QEvent>
#include <QFormLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QStringList>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

namespace micro_robot {

inline bool confirmCalibrationLoss(QWidget* parent, const QString& title, const QString& warning)
{
    return QMessageBox::warning(parent, title, warning, QMessageBox::Ok | QMessageBox::Cancel,
                                QMessageBox::Cancel) == QMessageBox::Ok;
}

class CalibrationForm final : public QObject {
public:
    explicit CalibrationForm(QWidget* parent) : QObject(parent) {}

    void watch(QWidget* field, std::function<QString()> read, std::function<QString()> error = {})
    {
        fields_.push_back({field, std::move(read), std::move(error)});
        field->installEventFilter(this);
    }

    QLineEdit* number(QFormLayout& layout, const QString& name, const QString& label,
                     double minimum, double maximum, std::optional<double> defaultValue = {},
                     bool integer = false)
    {
        auto* field = new QLineEdit;
        field->setObjectName(name);
        layout.addRow(defaultValue ? label + QStringLiteral(" · blank uses %1").arg(*defaultValue) : label, field);
        watch(field, [field] { return field->text(); }, [=] {
            bool parsed = false;
            const auto input = field->text().trimmed();
            if (input.isEmpty() && defaultValue) return QString{};
            const double value = integer ? static_cast<double>(input.toInt(&parsed)) : parseWorkspaceLength(input, &parsed);
            if (parsed && std::isfinite(value) && value >= minimum && value <= maximum) return QString{};
            return QStringLiteral("Enter %1 between %2 and %3.")
                .arg(integer ? QStringLiteral("an integer") : QStringLiteral("a number")).arg(minimum).arg(maximum);
        });
        return field;
    }

    static double value(const QLineEdit* field, double defaultValue = std::numeric_limits<double>::quiet_NaN())
    {
        if (field->text().trimmed().isEmpty()) return defaultValue;
        bool parsed = false;
        const double value = parseWorkspaceLength(field->text(), &parsed);
        return parsed ? value : std::numeric_limits<double>::quiet_NaN();
    }

    static void setValue(QLineEdit* field, double value)
    {
        field->setText(workspaceLengthText(value));
    }

    QStringList snapshot() const
    {
        QStringList values;
        for (const auto& input : fields_) values.push_back(input.read());
        return values;
    }

    QString validate()
    {
        QString first;
        for (const auto& input : fields_) {
            const auto error = validate(input);
            if (first.isEmpty()) first = error;
        }
        return first;
    }

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        if (event->type() == QEvent::FocusOut) {
            for (const auto& input : fields_)
                if (input.field == watched) { validate(input); break; }
        }
        return false;
    }

private:
    struct Field {
        QWidget* field;
        std::function<QString()> read;
        std::function<QString()> error;
    };

    static QString validate(const Field& input)
    {
        if (!input.error) return {};
        const auto error = input.error();
        input.field->setStyleSheet(error.isEmpty() ? QString{} : QStringLiteral("border: 1px solid #b42318;"));
        input.field->setToolTip(error);
        return error;
    }

    std::vector<Field> fields_;
};
} // namespace micro_robot
