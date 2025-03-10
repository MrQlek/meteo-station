#ifndef __ARM_UTILS_H__
#define __ARM_UTILS_H__

#define ARM_CRITICAL_SECTION(action) { \
    uint32_t irq_state = __get_PRIMASK(); \
    __disable_irq(); \
    action; \
    if(irq_state == 0) { \
        __enable_irq(); \
    } \
}

#endif // !__ARM_UTILS_H__