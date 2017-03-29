#pragma once

#include <memory>

#include "GUI.h"

extern MutexObject<bool> guiDone;
extern ThreadSynchronizer guiReady;

extern void runGuiThread(const std::shared_ptr<GUI> &gui );
