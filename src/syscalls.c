#include <sys/stat.h>
#include <sys/times.h>
#include <errno.h>
#include "stm32l4xx.h"

#undef errno
extern int errno;

#define UNUSED_PARAM            __attribute__((unused)) //TODO move to utils
#define WEAK                    __attribute__((weak)) //TODO move to utils

void _exit(int exit_code UNUSED_PARAM)
{
    while (1) {
        //TODO
    }
}

int _close(int file UNUSED_PARAM) {
    return -1;
}

char *__env[1] = { 0 };
char **environ = __env;

int _execve(char *name UNUSED_PARAM, 
    char **argv UNUSED_PARAM, char **env UNUSED_PARAM) {

    errno = ENOMEM;
    return -1;
}

int _fork(void) {
    errno = EAGAIN;
    return -1;
}


int _fstat(int file UNUSED_PARAM, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _getpid(void) {
    return 1;
}

int _isatty(int file UNUSED_PARAM) {
    return 1;
}

int _kill(int pid UNUSED_PARAM, int sig UNUSED_PARAM) {
    errno = EINVAL;
    return -1;
}

int _link(char *old UNUSED_PARAM, char *new UNUSED_PARAM) {
    errno = EMLINK;
    return -1;
}

int _lseek(int file UNUSED_PARAM, int ptr UNUSED_PARAM, int dir UNUSED_PARAM) {
    return 0;
}

WEAK int _open(const char *name UNUSED_PARAM, int flags UNUSED_PARAM,
    int mode UNUSED_PARAM) {

    return -1;
}

WEAK int _read(int file UNUSED_PARAM, char *ptr UNUSED_PARAM, int len UNUSED_PARAM) {
    return 0;
}

WEAK int _write(int file UNUSED_PARAM, char *ptr UNUSED_PARAM, int len) {
    return len;
}

register char * stack_ptr __asm("sp");

caddr_t _sbrk(int incr) {
    extern char __bss_end__;		/* Defined by the linker */
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0) {
        heap_end = &__bss_end__;
    }
    prev_heap_end = heap_end;
    if (heap_end + incr > stack_ptr) {
        while (1) {
            // TODO
            // Heap and stack collision
        }
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

int _stat(char *file UNUSED_PARAM, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _times(struct tms *buf UNUSED_PARAM) {
    return -1;
}

int _unlink(char *name UNUSED_PARAM) {
    errno = ENOENT;
    return -1; 
}

int _wait(int *status UNUSED_PARAM) {
    errno = ECHILD;
    return -1;
}

