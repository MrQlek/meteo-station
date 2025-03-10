#ifndef __IO_UTILS_H__
#define __IO_UTILS_H__

#include <stdio.h>

#define NEWLINE "\r\n"
#define OUT(msg, ...) printf(msg NEWLINE, ##__VA_ARGS__)
#define INFO(msg, ...) OUT("INFO: "msg, ##__VA_ARGS__)
#define DUMPI(x) OUT(# x " = %d", (int)x)
#define DUMPX(x) OUT(# x " = 0x%02x", (unsigned int)x)

#endif // !__IO_UTILS_H__