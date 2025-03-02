#ifndef __ASSERTS_H__
#define __ASSERTS_H__

#define _assert(x) ({x;})
#define assert_pointer(x)
#define ASSERT(x) _Static_assert(x)

#endif // !__ASSERTS_H__