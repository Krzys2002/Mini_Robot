// Checker fixture only; never compiled. Each directive below must be reported.
#pragma once

#include <QObject>
#include <QtMultimedia/QCamera>
#include <unistd.h>
#include <sys/ioctl.h>
#include <CoreBluetooth/CoreBluetooth.h>
#include <opencv2/videoio.hpp>
#include "camera/QtCameraDeviceDiscovery.hpp"

#if defined(__APPLE__)
#endif
#ifdef _WIN32
#endif
