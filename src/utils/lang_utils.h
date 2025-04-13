#ifndef __LANG_UTILS_H__
#define __LANG_UTILS_H__

#include <stdint.h>
#include <stdbool.h>

#define CAT(a, b) a##b

#define VERIFY_ENUM(x, _enum) (x < CAT(_enum,_LIMIT))

#define MUT
#define OUTPUT

#define UNUSED_PARAM            __attribute__((unused))

#define ARRAY_INIT_ALL_ZEROS {}
#define STRUCT_INIT_ALL_ZEROS {}

#define NELEMS(a) ((int)(sizeof(a)/sizeof((a)[0])))

#define UNUSED(variable) ((void) (variable))

#endif // !__LANG_UTILS_H__