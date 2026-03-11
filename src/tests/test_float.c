#include "rpi.h"
#include <stdint.h>
#include <math.h>
#include "float-helpers.h"

void notmain(void) {
    float x = 3.23f;
    float y = 4.73f; // 
    int16_t xs = float_to_fixed12_4(x);
    int16_t ys = float_to_fixed12_4(y);
    int16_t xs_round = float_to_fixed12_4_round(x);
    int16_t ys_round = float_to_fixed12_4_round(y);
    
    // using rounding gets us closer to original float than truncation
    // 12.4 values are spaced 1/16 apart. truncation always picks the lower 
    // 1/16 value. rounding picks the nearest 1/16 value.
    assert(fabs(fixed12_4_to_float(xs_round) - x) < fabs(fixed12_4_to_float(xs) - x));
    assert(fabs(fixed12_4_to_float(ys_round) - y) < fabs(fixed12_4_to_float(ys) - y));

    printk("packed: %x\n", pack_xs_ys_fixed12_4(x, y));
    int16_t xs_out, ys_out;

    // packing and unpacking should be inverse operations
    unpack_xs_ys_fixed12_4(pack_xs_ys_fixed12_4(xs, ys), &xs_out, &ys_out);
    assert(xs_out == xs);
    assert(ys_out == ys);
}