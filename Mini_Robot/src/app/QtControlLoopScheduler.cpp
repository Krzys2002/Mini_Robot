#include "app/QtControlLoopScheduler.hpp"

#include <limits>
#include <stdexcept>

namespace micro_robot {
QtControlLoopScheduler::QtControlLoopScheduler(ControlLoop& loop, QObject* parent)
    : QObject(parent), loop_(loop)
{
    if (loop.config().period.count() > std::numeric_limits<int>::max())
        throw std::invalid_argument("Control period exceeds Qt timer range");
    timer_.setInterval(loop.config().period);
    timer_.setTimerType(Qt::PreciseTimer);
    connect(&timer_, &QTimer::timeout, this, [this] {
        const bool recorded = loop_.poll();
        if (loop_.state() != ControlLoopState::Running) timer_.stop();
        if (recorded) emit cycleCompleted();
    });
}
QtControlLoopScheduler::~QtControlLoopScheduler() { stop(); }
bool QtControlLoopScheduler::start()
{
    if (!loop_.start()) return false;
    timer_.start();
    return true;
}
void QtControlLoopScheduler::stop()
{
    timer_.stop();
    loop_.stop();
}
} // namespace micro_robot
