#pragma once

#include "control/ControlLoop.hpp"
#include <QObject>
#include <QTimer>

namespace micro_robot {

// Timer adapter only. Same event-loop/owner thread as the injected service,
// WorldModel and RobotManager; no additional thread or cross-thread data access.
// Policy work must obey IController's prompt/nonblocking contract.
class QtControlLoopScheduler final : public QObject {
    Q_OBJECT
public:
    explicit QtControlLoopScheduler(ControlLoop& loop, QObject* parent = nullptr);
    ~QtControlLoopScheduler() override;
    [[nodiscard]] bool start();
    void stop();
signals:
    void cycleCompleted(); // Inspect loop.lastCycle(); contains no borrowed data.
private:
    ControlLoop& loop_;
    QTimer timer_;
};

} // namespace micro_robot
