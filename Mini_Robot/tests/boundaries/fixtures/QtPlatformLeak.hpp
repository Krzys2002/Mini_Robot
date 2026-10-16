// Checker fixture only; never compiled. Each Qt platform conditional below must
// be reported, even in a target that is allowed to use Qt Core.
#pragma once

#include <QObject>

#ifdef Q_OS_MACOS
#elif defined(Q_OS_LINUX)
#endif
#if defined(Q_OS_WIN)
#endif
