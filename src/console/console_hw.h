#ifndef __CONSOLE_HW_H__
#define __CONSOLE_HW_H__

#include "results.h"

extern void console_init();

extern int console_write(char *ptr, int len);
extern int console_write_blocking(char *ptr, int len);

#endif // !__CONSOLE_HW_H__