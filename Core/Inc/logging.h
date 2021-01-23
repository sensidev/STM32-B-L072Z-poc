#ifndef B_L072Z_LOGGING_H
#define B_L072Z_LOGGING_H

#include "stdio.h"

void log_info(const char *fmt, ...);

void log_uint8_buffer(const char *prefix, const uint8_t *buffer, uint8_t length);

#endif //B_L072Z_LOGGING_H
