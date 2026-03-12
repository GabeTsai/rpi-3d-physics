#include "simple-frag-shader.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef _MSC_VER
__declspec(align(8))
#elif defined(__GNUC__)
__attribute__((aligned(8)))
#endif
uint32_t simple_frag_shader[18] = {
0x009e7000, 0x100009e7,
0x009e7000, 0x100009e7,
0x009e7000, 0x400009e7,
0xff0000ff, 0xe0020827,
0x159e7000, 0x10020ba7,
0x009e7000, 0x500009e7,
0x009e7000, 0x300009e7,
0x009e7000, 0x100009e7,
0x009e7000, 0x100009e7
};
#ifdef __HIGHC__
#pragma Align_to(8, simple_frag_shader)
#ifdef __cplusplus
}
#endif
#endif
