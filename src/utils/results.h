#ifndef __RESULTS_H__
#define __RESULTS_H__

#include <stdint.h>

typedef enum {
    RESULT_OK = 0,
    RESULT_GENERIC_ERROR = 1,
    RESULT_TIMEOUT_ERROR = 2,
} result_t;

typedef struct {
    uint8_t value;
    result_t result;
} byte_result_t;

#define BYTE_RESULT(_val, _result) (byte_result_t) { \
    .value = _val, \
    .result = _result, \
}

#define PROPAGATE_ERROR(_val) { \
    result_t _res = _val; \
    if(_res != RESULT_OK) { \
        return _res; \
    } \
}

#endif // !__RESULTS_H__