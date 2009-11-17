// ============================================================================
// === Operations on 36-bit pseudo words
/*
    Operations on 36-bit pseudo words

    Multics 36bit words are simulated with 64bit integers.  Multics
    uses big endian representation.  Bit numbering is left to right;
    bit number zero is the leftmost or the most significant bit (MSB).
    The documentation refers to the right-most or least significant bit
    as position 35. 

    Note that with the LSB zero convention, the value of a bit position
    matches up with its twos-complement value, e.g. turning on only
    bit #35 results in a value of 2 raised to the 35th power.  With the
    MSB zero convention used in Multics, turning on only bit 35 results
    in the twos-complement value of one.

    The following macros support operating on 64bit words as though the
    right-most bit were bit 35.
*/


// ============================================================================

/*
 * Constants and a function for ordinary masking of low (rightmost) bits.
 */

static const t_uint64 MASK36 = ~(~((t_uint64)0)<<36);   // lower 36 bits all on
static const t_uint64 MASK18 = ~(~((t_uint64)0)<<18);   // lower 18 bits all on
#define MASKBITS(x) ( ~(~((t_uint64)0)<<x) )    // lower (x) bits all ones

// ============================================================================

/*
 * Extract, set, or clear the (i)th bit of a 36-bit word (held in a uint64).
 */

#define bitval36(word,i) ( ((word)>>(35-i)) & (uint64_t) 1 )

#define bitset36(word,i) ( (word) | ( (uint64_t) 1 << (35 - i)) )

#define bitclear36(word,i) ( (word) & ~ ( (uint64_t) 1 << (35 - i)) )

// ============================================================================

/*
 * getbits36()
 *
 * Extract a range of bits from a 36-bit word.
 */

static inline t_uint64 getbits36(t_uint64 x, int i, unsigned n) {
    // bit 35 is right end, bit zero is 36th from the right
    int shift = 35-i-n+1;
    if (shift < 0 || shift > 35) {
        log_msg(ERR_MSG, "getbits36", "bad args (%Lo,i=%d,n=%d)\n", x, i, n);
        cancel_run(STOP_BUG);
        return 0;
    } else
        return (x >> (unsigned) shift) & ~ (~0 << n);
}

// ============================================================================

/*
 * setbits36()
 *
 * Set a range of bits in a 36-bit word -- Returned value is x with n bits
 * starting at p set to the n lowest bits of val 
 */

static inline t_uint64 setbits36(t_uint64 x, int p, unsigned n, t_uint64 val)
{
    int shift = 36 - p - n;
    if (shift < 0 || shift > 35) {
        log_msg(ERR_MSG, "setbits36", "bad arg, shift = %d\n", shift);
        cancel_run(STOP_BUG);
        return 0;
    }
    t_uint64 mask = ~ (~0<<n);  // n low bits on
    mask <<= (unsigned) shift;  // shift 1s to proper position; result 0*1{n}0*
    // caller may provide val that is too big, e.g., a word with all bits
    // set to one, so we mask val
    t_uint64 result = (x & ~ mask) | ((val&MASKBITS(n)) << (36 - p - n));
    return result;
}

// ============================================================================

/*
 * bit#_is_neg()
 *
 * Functions to determine if bit-36, bit-18, or bit-n word's MSB is
 * on.
 */

#define bit36_is_neg(x) (((x) & (((t_uint64)1)<<35)) != 0)

#define bit18_is_neg(x) (((x) & (((t_uint64)1)<<17)) != 0)

#define bit_is_neg(x,n) (((x) & (((t_uint64)1)<<((n)-1))) != 0)

//=============================================================================

/*
 * sign36()
 *
 * Extract a 36-bit signed value from a 36-bit word.
 *
 */

static inline t_int64 sign36(t_uint64 x)
{
    if (bit36_is_neg(x)) {
        t_int64 r = - (((t_int64)1<<36) - (x&MASK36));
        return r;
    }
    else
        return x;
}

/*
 * sign18()
 *
 * Extract an 18bit signed value from a 36-bit word.
 *
 */

static inline int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        return r;
    }
    else
        return x;
}

/*
 * sign15()
 *
 * Extract an 15bit signed value from a 36-bit word.
 *
 */

static inline int32 sign15(uint x)
{
    if (bit_is_neg(x,15)) {
        int32 r = - ((1<<15) - (x&MASKBITS(15)));
        return r;
    }
    else
        return x;
}

//=============================================================================

/*
 * negate36()
 *
 * Negate an 36-bit signed value.  Result is in Multics representation.
 *
 */

static inline t_int64 negate36(t_uint64 x)
{
    // overflow not detected
    if (bit36_is_neg(x))
        return ((~x & MASK36) + 1) & MASK36;    // todo: only one mask needed?
    else
        return (- x) & MASK36;
}

/*
 * negate18()
 *
 * Negate an 18-bit signed value within the lower part of a 36bit word.
 * Result is in Multics representation; use sign18()
 * to extract values for computation.
 *
 */

static inline int32 negate18(t_uint64 x)
{
    // overflow not detected
    if (bit18_is_neg(x))
        return ((~x & MASK18) + 1) & MASK18;    // todo: only one mask needed?
    else
        return (- x) & MASK18;
}

/*
 * negate72()
 *
 * Arguments are pointers to two 36-bit words, one holding the high bits and
 * the other the low bits.
 * Result is in Multics representation.
 *
 */

static inline void negate72(t_uint64* hip, t_uint64* lop)
{
    // BUG? -- overflow not detected/reported.
    *hip = (~ *hip) & MASK36;
    *lop = (~ *lop) & MASK36;
    ++ *lop;
    if ((*lop >> 36) != 0) {
        *lop &= MASK36;
        ++ *hip;
        *hip = *hip & MASK36;
    }
}


//=============================================================================
