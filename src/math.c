/*
    math.c -- math routines.   Implemented via GNU MP library.
*/

#include "hw6180.h"
#include <limits.h>
#include <gmp.h>

static inline void set36u(mpz_t rop, t_uint64 val)
{
#if LONG_MAX > 2147483647L
    mpz_set_ui(rop, val);
#else
    uint32 parts[2];
    parts[0] = val >> 32;   // upper bits
    parts[1] = val & MASKBITS(32);
    mpz_import(rop, 2, 1, sizeof(parts[0]), 0, 0, parts);
#endif
}


#if 0
static inline void set36(mpz_t rop, t_int64 val)
{
#if LONG_MAX > 2147483647L
    // 64bit longs -- no special effort
    mpz_set_si(rop, val);
#else
    // The longs are 32bits and GMP doesn't support long long
    // arguments, so build up via calls using 18 bits at a time

    // oops, mpz_import exists
#if 0
    int neg;
    if ((neg = (val < 0)))
        val = -val;
    mpz_set_si(rop, (long) (val >> 18));        // put in the uppermost bits
    mpz_mul_2exp(rop, rop, 18); // shift them into position
    mpz_t low;
    mpz_init2(low, 36);
    mpz_set_si(low, (long) (val & MASK18));
    mpz_ior(rop, rop, low); // add in the low bits
    if (neg)
        mpz_neg(rop, rop);
    mpz_clear(low);
#else
    int neg;
    if ((neg = (val < 0)))
        val = -val;
    uint32 parts[2];
    parts[0] = val >> 32;   // upper bits
    parts[1] = val & MASKBITS(32);
    mpz_import(rop, 2, 1, sizeof(parts[0]), 0, 0, parts);
    if (neg)
        mpz_neg(rop, rop);
#endif
#endif
}
#endif


void mpy(t_int64 a, t_int64 b, t_uint64* hip, t_uint64 *lowp)
{
    // Signed multiply of two 36bit integers, a and b, with results
    // into *hip and *lowp.  If desired, it's safe for hip and/or lowp
    // to point to a and/or b.

    flag_t neg = 0;
    flag_t maxneg = 0;
    if (a == 0) {
        if (b == 0) {
            *lowp = 0;
            *hip = 0;
            return;
        }
    } else if (a < 0) {
        maxneg = a == - ((t_int64)1<<35);
        a = -a;
        neg = 1;
    }
    if (b < 0) {
        maxneg &= b == - ((t_int64)1<<35);
        if (maxneg) {
            *lowp = 0;
            *hip = (t_uint64)1 << 34;       
            return;
        }
        b = -b;
        neg = !neg;
    }

    mpz_t aa, bb;

    mpz_init2(aa, 36);
    mpz_init2(bb, 36);
    set36u(aa, a);
    set36u(bb, b);

    mpz_mul(aa, aa, bb);
    
    // Export with least significant first in case result is exactly one word.
    // Also initialize results to zero for the same reason.
    t_uint64 bits[2] = { 0, 0 };
    size_t count = 2;
    mpz_export(bits, &count, -1, sizeof(bits[0]), 0, 64-36, aa);

    if (neg) {
        t_uint64 hi = (~bits[1]) & MASK36;
        t_uint64 low = bits[0];
        if (low != 0)
            low = ((~low) + 1) & MASK36;    // no carry
        else
            ++hi;       // complement of zero, then plus one yields zero with carry
        *lowp = low;
        *hip = hi;
    } else {
        *hip = bits[1];
        *lowp = bits[0];
    }

    mpz_clear(aa);
    mpz_clear(bb);
}
