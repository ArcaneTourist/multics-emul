/*
    math_real.c -- instructions for fixed point fractions and floating point

    Called by routines in opu.c

    Multics uses a 2's complement representation for fixed binary numbers.
    The two major formats are the q1.35 aka the fx1.36 format and the
    q1.71 aka fx1.71 format.   In the latter format, the least significant
    bit (71st bit) of the 72 bit double word is ignored.

*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#include "hw6180.h"

// ============================================================================

#if 0
static int divide_fraction(t_uint64* ahip, t_uint64 *alowp, t_uint64* bp)
{
}
#endif

// ============================================================================

int instr_dvf(t_uint64 word)
{
    // dvf -- divide fraction; 71-bit signed fractional dividend is
    // divided by a 36-bit fractional divisor

    log_msg(INFO_MSG, "opu::dvf", "AQ = {%012llo,%012llo} aka (%lld,%lld).  Divisor = %012llo aka %lld.\n", reg_A, reg_Q, reg_A, reg_Q, word, word);

    double aq = multics_to_double(reg_A, reg_Q, 0, 1);
    double div = multics_to_double(word, 0, 0, 1);
    log_msg(INFO_MSG, "opu::dvf", "%.6g/%.6g\n", aq, div);
    int dividend_is_neg = bit36_is_neg(reg_A);
    int divisor_is_neg = bit36_is_neg(word);

    if (dividend_is_neg)
        negate72(&reg_A, &reg_Q);
    if (divisor_is_neg)
        word = negate36(word);

    if (reg_A >= word) { // We're comparing fractions, so we can ignore the less significant bits of the bigger word
    // if (reg_A || reg_Q >= word) -- treg_As expression would be for integers, not fractions
        log_msg(WARN_MSG, "opu::dvf", "Numerator larger than denominator -- not generating fault though.\n");
    }
    // if (reg_A >= word || word == 0)
    if (word == 0) {
        fault_gen(div_fault);
        IR.neg = dividend_is_neg;
        IR.zero = word == 0;
        return 1;
    }

    // Shift AQ right one -- adjust for scaling?  Loses bit 72 as noted in AL-39.
    reg_Q = setbits36(reg_Q>>1, 0, 1, reg_A&1); 
    reg_A >>= 1;

    // Do unsigned division of the two values
    t_uint64 quot;
    t_uint64 rem;
    div72(reg_A, reg_Q, word, &quot, &rem);
    if (rem != 0)
        if (dividend_is_neg)
            rem = negate36(rem);
    if (dividend_is_neg != divisor_is_neg)
        quot = negate36(quot);

    reg_A = quot;
    reg_Q = rem;
    IR.zero = reg_A == 0;
    IR.neg = bit36_is_neg(reg_A);

    double tmp = multics_to_double(reg_A, 0, 0, 1);
    log_msg(INFO_MSG, "opu::dvf", "quotient:  A => %.6g\n", tmp);
    tmp = multics_to_double(reg_Q, 0, 0, 1);
    log_msg(INFO_MSG, "opu::dvf", "remainder: Q => %.6g\n", tmp);

    return 0;
}

// ============================================================================

/*
 * negate28()
 *
 * Negate an 28-bit signed value within the lower part of a 36bit word.
 * Result is in Multics representation; use sign28()
 * to extract values for computation.
 *
 */

static inline int32 negate28(t_uint64 x)
{
    // overflow not detected
    if (bit_is_neg(x,28))
        return ((~x & MASKBITS(28)) + 1) & MASKBITS(28);
    else
        return (- x) & MASKBITS(28);
}

// ============================================================================

/*
 * sign28()
 *
 * Extract an 28bit signed value from a 36-bit word.
 *
 */

static inline int32 sign28(t_uint64 x)
{
    if (bit_is_neg(x, 28)) {
        int32 r = - ((1<<28) - (x&MASKBITS(28)));
        return r;
    }
    else
        return x;
}


// ============================================================================

int instr_ufas(t_uint64 word, flag_t subtract)
{
    log_msg(DEBUG_MSG, "opu::ufa", "E = %03o(%d) AQ = {%012llo,%012llo} aka (%lld,%lld).\n", reg_E, reg_E, reg_A, reg_Q, reg_A, reg_Q);
    double x = multics_to_double(reg_A, reg_Q, 0, 1);
    log_msg(INFO_MSG, "opu::ufa", "AQE is %.4g * 2^%d\n", x, reg_E);

    uint8 op_exp = getbits36(word, 0, 8);
    uint32 op_mant = getbits36(word, 8, 28);    // 36-8=28 bits
    log_msg(DEBUG_MSG, "opu::ufa", "op = %012llo => exp %03o(%d) and mantissa %010o (%d)\n", word, op_exp, (int8) op_exp, op_mant, op_mant);
    x = multics_to_double(op_mant, 0,  0, 1);
    log_msg(INFO_MSG, "opu::ufa", "op is %g * 2^%d\n", x, op_exp);

    if (op_mant == 0) {
        // short circuit the addition of zero
        IR.zero = reg_A == 0 && reg_Q == 0;
        IR.neg = bit36_is_neg(reg_A);
        IR.exp_overflow = 0;
        IR.exp_underflow = 0;
        IR.carry = 0;
        log_msg(INFO_MSG, "opu::ufa", "addition of zero.\n");
        return 0;
    }

    if (subtract) {
        double x = multics_to_double(op_mant << 8, 0, 0, 1);
        log_msg(INFO_MSG, "opu::ufs", "Op mantissa originally %sb (aka %#o aka %+d).\n", bin2text(op_mant, 36), op_mant, x);
        log_msg(INFO_MSG, "opu::ufs", "Op mantissa originally %sb (aka %#o aka %+d).\n", bin2text(op_mant, 28), op_mant, x);
        op_mant = negate28(op_mant);
        x = multics_to_double(op_mant << 8, 0, 0, 1);
        log_msg(INFO_MSG, "opu::ufs", "Negating yields:       %sb (aka %#o aka %+d).\n", bin2text(op_mant, 28), op_mant, x);
        log_msg(INFO_MSG, "opu::ufs", "Negating yields:       %sb (aka %#o aka %+d).\n", bin2text(op_mant, 36), op_mant, x);
    }

    int untested = 0;
    uint8 new_e;
    if ((int8) op_exp < (int8) reg_E) {
        // operand has the smaller exponent
        new_e = reg_E;
        int n = (int8) reg_E - (int8) op_exp;
        op_mant >>= n;
        log_msg(INFO_MSG, "opu::ufa", "exp diff is %d; op mantissa now %010o\n", n, op_mant);
    } else {
        // AQ has the smaller exponent
        untested = 1;
        new_e = op_exp;
        int n = (int8) op_exp - (int8) reg_E;
        // Shift AQ right n bits
        reg_Q = setbits36(reg_Q >> n, 0, n, reg_A);
        reg_A >>= n;
        log_msg(NOTIFY_MSG, "opu::ufa", "exp diff is %d; AQ mantissa now {%012llo, %012llo} (%lld,%lld)\n", n, reg_A, reg_Q, reg_A, reg_Q);
        reg_E = op_exp;
    }

    int op_is_neg = (op_mant >> 27) != 0;
    int ret;
    if (op_is_neg) {
        op_mant = setbits36(op_mant, 0, 8, 1);  // sign extend op_mant
        ret = add72(MASK36, op_mant, &reg_A, &reg_Q, 0);
    } else
        ret = add72(0, op_mant, &reg_A, &reg_Q, 0);
    if (IR.overflow) {
        ++ reg_E;
        reg_E &= 0177;
        // Shift AQ by one and invert sign bit
        reg_Q = setbits36(reg_Q >> 1, 0, 1, reg_A & 1);
        int sign = bit36_is_neg(reg_A);
        reg_A >>= 1;
        reg_A = setbits36(reg_A, 0, 1, ! sign);
        // NOTE: fno would turn off the overflow flag, but we don't
    }

    x = multics_to_double(reg_A, reg_Q, 0, 1);
    log_msg(INFO_MSG, "opu::ufa", "resulting AQE is %.6g * 2^%d\n", x, reg_E);

    if (untested) {
        log_msg(NOTIFY_MSG, "OPU::ufa", "Unnormalized Floating Add is untested\n");
        cancel_run(STOP_WARN);
        return 1;
    }
    
    return 0;
}

// ============================================================================

int instr_ufm(t_uint64 word)
{
    int ret = 0;

    log_msg(INFO_MSG, "opu::ufm", "E = %03o(%d) AQ = {%012llo,%012llo} aka (%lld,%lld).\n", reg_E, reg_E, reg_A, reg_Q, reg_A, reg_Q);
    double x = multics_to_double(reg_A, reg_Q, 0, 1);
    log_msg(NOTIFY_MSG, "opu::ufm", "AQE is %g * 2^%d\n", x, reg_E);

    uint8 op_exp = getbits36(word, 0, 8);
    t_uint64 op_mant = getbits36(word, 8, 28) << 8; // 36-8=28 bits
    log_msg(INFO_MSG, "opu::ufm", "op = %012llo => exp %03o(%d) and mantissa %012llo (%lld)\n", word, op_exp, (int8) op_exp, op_mant, op_mant);
    x = multics_to_double(op_mant, 0,  0, 1);
    log_msg(NOTIFY_MSG, "opu::ufm", "op is %g * 2^%d\n", x, op_exp);

    IR.exp_underflow = 0;
    IR.exp_overflow = 0;
    int exp = (int8) reg_E + op_exp;
    // BUG: Do we need to generate faults?
    if (exp < -128) {
        IR.exp_underflow = 1;
        // ret = 1;
        log_msg(NOTIFY_MSG, "opu::ufm", "exp underflow.\n");
        cancel_run(STOP_IBKPT);
    } else if (exp > 127) {
        IR.exp_overflow = 1;
        // ret = 1;
        log_msg(NOTIFY_MSG, "opu::ufm", "exp overflow.\n");
        cancel_run(STOP_IBKPT);
    }
    reg_E = (unsigned) exp & MASKBITS(8);
    log_msg(INFO_MSG, "opu::ufm", "new exp is %d aka %#o\n", exp, reg_E);

    int normalize = reg_A == ((t_uint64) 1 << 35) && reg_Q == 0 && op_mant == ((t_uint64) 1 << 35);
#if 0
    if (normalize) {
        log_msg(NOTIFY_MSG, "opu::ufm", "Normalizing operand...\n");
        if (instr_fno() != 0)
            return 1;
    } else
#endif
    {
        t_uint64 a = reg_A;
        t_uint64 q = reg_Q;
        mpy72fract(reg_A, reg_Q, op_mant, &reg_A, &reg_Q);
        log_msg(INFO_MSG, "opu::ufm", "Multiplying {%012llo,%012llo} by {%012llo} yields {%012llo,%012llo}\n", a, q, op_mant, reg_A, reg_Q);
    }

    if (normalize) {
        log_msg(NOTIFY_MSG, "opu::ufm", "Normalizing results....\n");
        ret = instr_fno();
    }

    x = multics_to_double(reg_A, reg_Q, 0, 1);
    log_msg(NOTIFY_MSG, "opu::ufm", "resulting AQE is %g * 2^%d\n", x, reg_E);

    if (normalize) {
        log_msg(NOTIFY_MSG, "opu::ufm", "Auto Breakpoint for normalize mode.\n");
        (void) cancel_run(STOP_IBKPT);
    }
    return ret;
}

// ============================================================================

int instr_fno()
{
    if (opt_debug) {
        double x = multics_to_double(reg_A, reg_Q, 0, 1);
        log_msg(NOTIFY_MSG, "OPU::fno", "AQE initially {%0llo,%012llo},%03o => %g * 2^%d\n", reg_A, reg_Q, reg_E, x, reg_E);
    }

    if (IR.overflow) {
        // Shift AQ by one and invert sign bit
        reg_Q = setbits36(reg_Q >> 1, 0, 1, reg_A & 1);
        int sign = bit36_is_neg(reg_A);
        reg_A >>= 1;
        reg_A = setbits36(reg_A, 0, 1, ! sign); // BUG: Should we invert the old sign or the shifted-in zero?
        IR.overflow = 0;
        return 0;
    }

    // Note: no special case test needed for AQ == 0 before the loop

    // BUG: What is the exact behavior on underflow?  Do we shift or not?

    while (getbits36(reg_A, 0, 1) == getbits36(reg_A, 1, 1)) {
        // Left shift AQ
        reg_A <<= 1;
        reg_A &= MASK36;
        reg_A |= getbits36(reg_Q, 0, 1);
        reg_Q <<= 1;
        reg_Q &= MASK36;
        IR.zero = reg_A == 0 && reg_Q == 0;
        if (IR.zero) {
            reg_E = -128;
            break;
        }
        // Reduce exponent
        if (reg_E == -128) {
            IR.exp_underflow = 1;
            reg_E = 0;
            break;
        }
        -- reg_E;
    }

    IR.neg = bit36_is_neg(reg_A);
    // IR.exp_overflow = 0;

    if (IR.zero) {
        log_msg(NOTIFY_MSG, "OPU::fno", "Result is zero.\n");
        return 0;
    }

    if (opt_debug) {
        double x = multics_to_double(reg_A, reg_Q, 0, 1);
        log_msg(DEBUG_MSG, "OPU::fno", "AQE now:      {%0llo,%012llo},%03o => %g * 2^%d\n", reg_A, reg_Q, reg_E, x, reg_E);
    }

    return 0;
}

// ============================================================================

double multics_to_double(t_uint64 xhi, t_uint64 xlo, int show, int is_signed)
{
    // Convert from Multics fixed point fractional to "C" double.
    // Results will be approximate for some hardware.
    // xhi is the more significant bits just after the decimal point.

    t_uint64 orig_hi = xhi;
    t_uint64 orig_lo = xlo;
    
    int sign = 0;
    if (is_signed) {
        // The MSB is a sign bit; the next bit represents 1/2 and the following 1/4, etc
        sign = bit36_is_neg(xhi) ? -1 : 0;
        // 72-bit left shift: remove sign, move bit representing 1/2 to MSB
        xhi <<= 1;
        xhi &= MASK36;
        xhi |= bit36_is_neg(xlo);
        xlo <<= 1;
        xlo &= MASK36;
    }

    double val = 0;
    double fact = 2;
    int nbits = 0;
    for (int which = 0; which <= ((xlo == 0) ? 0 : 1); ++which) {
        t_int64 x = (which == 0) ? xhi : xlo;
        t_uint64 bitsel = (t_uint64) 1 << 35;
        for (int i = 0; i < 36; ++i) {
            ++ nbits;
            if ((x & bitsel) != 0) {
                val += (double) 1 / fact;
                if (show)
                    out_msg("bit %d on: val now %g\n", nbits, val);
            }
            bitsel >>= 1;
            fact *= 2;
        }
    }
    if (is_signed)
        val += sign;
    if (show)
        log_msg(NOTIFY_MSG, "OPU::fixed", " {%#llo,%#llo} => %g\n", orig_hi, orig_lo, val);
    return val;
}

// ============================================================================

#if 0
static void normalize(t_uint64 word)
{
}
#endif

// ============================================================================

/*
 * Normalized Numbers -- AL39, Chapter 2
 * A floating-point binary number is said to be normalized if the relation
 * -0.5 > M > -1 or 0.5 <= M < 1 or [M=0 and E=-128]
 *  is satisfied. This is a result of using a 2's complement mantissa. Bits 8 and 9 are different unless
 *  the number is zero. The presence of unnormalized numbers in any finite mantissa arithmetic can
 *  only degrade the accuracy of results. For example, in an arithmetic allowing only two digits in the
 *  mantissa, the number 0.005x102 has the value zero instead of the value one-half.
 *  Normalization is a process of shifting the mantissa and adjusting the exponent until the
 *  relation above is satisfied. Normalization may be used to recover some or all of the extra bits of
 *  the overlength AQ-register after a floating-point operation.
 *  There are cases where the limits of the registers force the use of unnormalized numbers.
 *  For example, in an arithmetic allowing three digits of mantissa and one digit of exponent, the
 *  calculation 0.3x10-10 - 0.1x10-11 (the normalized case) may not be made, but 0.03x10-9 -
 *  0.001x10-9 = 0.029x10-9 (the unnormalized case) is a valid result.
*/

/*
NOTES: The ufa instruction is executed as follows:
The mantissas are aligned by shifting the mantissa of the operand
having the algebraically smaller exponent to the right the number of
places equal to the absolute value of the difference in the two
exponents. Bits shifted beyond the bit position equivalent to AQ71 are
lost.
The algebraically larger exponent replaces C(E).
The sum of the mantissas replaces C(AQ).
If an overflow occurs during addition, then;
C(AQ) are shifted one place to the right.
C(AQ)0 is inverted to restore the
*/

