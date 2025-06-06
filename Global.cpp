#include "Global.h"
#include <shared_mutex>

bool reachedDestination = false;
shared_mutex mutexReachedDestination;
