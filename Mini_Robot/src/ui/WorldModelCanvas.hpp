#pragma once

#include <QWidget>

namespace micro_robot {
class DesktopController;

class WorldModelCanvas final : public QWidget {
public:
    explicit WorldModelCanvas(DesktopController& controller, QWidget* parent = nullptr);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    DesktopController& controller_;
};

} // namespace micro_robot
