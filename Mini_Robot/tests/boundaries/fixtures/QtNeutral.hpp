// Checker fixture only; never compiled. Ordinary Qt Core usage must pass in a
// target that is allowed to use Qt Core.
#pragma once

#include <QObject>
#include <QString>
#include <QTimer>
#include <QtCore/QList>

#if QT_VERSION >= QT_VERSION_CHECK(6, 5, 0)
#endif
