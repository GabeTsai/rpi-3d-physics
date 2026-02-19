#include "test.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
__declspec(align(8))
#elif defined(__GNUC__)
__attribute__((aligned(8)))
#endif
uint32_t test[2] = {
0x15827d80, 0x10020027
};
#ifdef __HIGHC__
#pragma Align_to(8, test)
#ifdef __cplusplus
}
#endif
#endif
