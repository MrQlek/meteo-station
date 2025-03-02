#ifndef __LANG_UTILS_H__
#define __LANG_UTILS_H__

#include <stdint.h>
#include <stdbool.h>

#define CAT(a, b) a##b

#define VERIFY_ENUM(x, _enum) (x < CAT(_enum,_LIMIT))

#endif // !__LANG_UTILS_H__