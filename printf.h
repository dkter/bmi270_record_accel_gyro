#pragma once

#include <stdint.h>
#include <stddef.h>
#include <driverlib.h>

size_t write(int handle, const unsigned char *buf, size_t bufSize);