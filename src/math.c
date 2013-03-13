/*
    math.c -- math routines.   Implemented via GNU MP library.
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
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


static inline void set72u(mpz_t rop, t_uint64 hi, t_uint64 low)
{
#if LONG_MAX > 2147483647L
    t_uint64 parts[2];
    parts[0] = hi;
    parts[1] = low;
    mpz_import(rop, 2, 1, sizeof(parts[0]), 0, 64-36, parts);
#else
    uint32 parts[4];
    parts[0] = hi >> 18;
    parts[1] = hi & MASKBITS(18);
    parts[2] = low >> 18;
    parts[3] = low & MASKBITS(18);
    mpz_import(rop, 4, 1, sizeof(parts[0]), 0, 32-18, parts);
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


void mpy(t_uint64 a, t_uint64 b, t_uint64* hip, t_uint64 *lowp)
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
    } else if (bit36_is_neg(a)) {
        maxneg = a == - ((t_int64)1<<35);
        a = -a; // result will be 35 or less bits
        neg = 1;
    }
    if (bit36_is_neg(b)) {
        maxneg &= b == - ((t_int64)1<<35);
        if (maxneg) {
            *lowp = 0;
            *hip = (t_uint64)1 << 34;       
            return;
        }
        b = -b; // result will be 35 or less bits
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
        if (low == 0)
            ++hi;       // complement of zero, then plus one yields low of zero with carry into hi
        else
            low = ((~low) + 1) & MASK36;    // no carry possible
        *lowp = low;
        *hip = hi;
    } else {
        *hip = bits[1];
        *lowp = bits[0];
    }

    mpz_clear(aa);
    mpz_clear(bb);
}

void mpy72fract(t_uint64 ahi, t_uint64 alow, t_uint64 b, t_uint64* hip, t_uint64 *lowp)
{
    // Signed multiply of a 72bit fraction by a 36bit fraction, with results
    // into *hip and *lowp.  72 MSB bits are returned; excess less significant
    // bits are discarded.
    // If desired, it's safe for hip and/or lowp to point to a and/or b.

    // BUG: We may mishandle the max negative value
    flag_t neg = 0;
    if (ahi == 0 && alow == 0) {
        *lowp = 0;
        *hip = 0;
        return;
    } else if (b == 0) {
        *lowp = 0;
        *hip = 0;
    } else if (bit36_is_neg(ahi)) {
        negate72(&ahi, &alow);
        neg = 1;
    }
    if (bit36_is_neg(b)) {
        b = -b; // results is 35 bits or less
        neg = !neg;
    }

    // We want the first bit to represent 2^-1, not -1*2^0 for the multiply
    // No need to mask the following because ahi and b are non-negative
    b <<= 1;
    ahi <<= 1;
    ahi |= (alow >> 35);
    alow = (alow << 1) & MASK36;

    mpz_t aa, bb;

    mpz_init2(aa, 72);
    mpz_init2(bb, 36);
    set72u(aa, ahi, alow);
    set36u(bb, b);

    mpz_mul(aa, aa, bb);
    
    // Export most significant first because we'll discard all but the first two limbs
    unsigned int count = 0;
    t_uint64 *bits = mpz_export(NULL, &count, 1, sizeof(*bits), 0, 64-36, aa);
    if (bits == NULL) {
        *lowp = 0;
        *hip = 0;
    } else {
        *hip = bits[0];
        if (count < 1)
            *lowp = 0;
        else
            *lowp = bits[1];
        free(bits);
        // The first bit now represents 2^-1, but the H6180 wants it to be -1*2^0, so shift
        *lowp >>= 1;
        if ((*hip & 1) != 0)
            *lowp |= ((t_uint64)1<<35);
        *hip >>= 1;
        // We did a positive multiplication, so negate now if needed
        if (neg)
            negate72(hip, lowp);
    }

    mpz_clear(aa);
    mpz_clear(bb);
}


void div72(t_uint64 hi, t_uint64 low, t_uint64 divisor, t_uint64* quotp, t_uint64* remp)
{
    // unsigned 72 bit value divided by 36 bit value with remainder

    mpz_t num, denom, quot, rem;

    mpz_init2(num, 72);
    mpz_init2(denom, 36);
    mpz_init2(quot, 36);
    mpz_init2(rem, 36);
    set72u(num, hi, low);
//printf("div72: num {%012llo,%012llo} ==> ", hi, low); mpz_out_str(stdout, 8, num); printf(".\n");
    set36u(denom, divisor);

    //mpz_mul_2exp (num, num, 36);  // scale numerator
    // No scaling -- scaled 72bit numerator, scaled 36 bit denominator; result is scaled 36 bit quotient (but may need 72 bits if denominator was one)
    mpz_tdiv_qr (quot, rem, num, denom);    // num/denom => quot, rem
//printf("div72: num = "); mpz_out_str(stdout, 8, num);
//printf(", denom = "); mpz_out_str(stdout, 8, denom);
//printf(" ===> quot "); mpz_out_str(stdout, 8, quot);
//printf(", remainder "); mpz_out_str(stdout, 8, rem);
//printf("\n");
    
    // Caller wants only the most significant bits of the quotient
    t_uint64 *bits;
    size_t count = 0;
    bits = mpz_export(NULL, &count, 1, sizeof(*bits), 0, 64-36, quot);
    if (bits == NULL) {
        *quotp = 0;
    } else {
        *quotp = *bits;
        free(bits);
    }

    // We should shift remainder right by 36 bits -- but caller just wants
    // the lower 36 bits not the more significant 36 zeros
    count = 0;
    bits = mpz_export(NULL, &count, 1, sizeof(*bits), 0, 64-36, rem);
    if (bits == NULL) {
        *remp = 0;
    } else {
        if (count < 1)
            *remp = 0;
        else
            *remp = *bits;
        free(bits);
    }

    mpz_clear(num);
    mpz_clear(denom);
    mpz_clear(quot);
    mpz_clear(rem);
}
