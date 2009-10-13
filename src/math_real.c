/*
    math_real.c -- instructions for fixed point fractions and floating point

    Called by routines in opu.c

    Multics uses a 2's complement representation for fixed binary numbers.
    The two major formats are the q1.35 aka the fx1.36 format and the
    q1.71 aka fx1.71 format.   In the latter format, the least significant
    bit (71st bit) of the 72 bit double word is ignored.

*/

#include "hw6180.h"
double multics_to_double(t_uint64 xhi, t_uint64 xlo, int show, int is_signed);

// ============================================================================

static inline t_int64 negate36(t_uint64 x)
{
    // overflow not detected
    if (bit36_is_neg(x))
        return ((~x & MASK36) + 1) & MASK36;    // todo: only one mask needed?
    else
        return (- x) & MASK36;
}


static inline void negate72(t_uint64* a, t_uint64* b)
{
    // BUG? -- overflow not detected
    *a = (~ *a) & MASK36;
    *b = (~ *b) & MASK36;
    ++ *b;
    if ((*b >> 36) != 0) {
        *b &= MASK36;
        ++ *a;
        *a = *a & MASK36;
    }
}

// ============================================================================

int instr_dvf(t_uint64 word)
{
    // dvf -- divide fraction; 71-bit signed fractional dividend is
    // divided by a 36-bit fractional divisor

    log_msg(NOTIFY_MSG, "opu::dvf", "AQ = {%012llo,%012llo} aka (%lld,%lld).  Divisor = %012llo aka %lld.\n", reg_A, reg_Q, reg_A, reg_Q, word, word);

    double aq = multics_to_double(reg_A, reg_Q, 1, 1);
    double div = multics_to_double(word, 0, 1, 1);
    log_msg(NOTIFY_MSG, "opu::dvf", "%g/%g\n", aq, div);
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

    // shift AQ right one
    reg_Q = setbits36(reg_Q>>1, 0, 1, reg_A&1); 
    reg_A >>= 1;

    t_uint64 quot;
    t_uint64 rem;
    // printf("Calling div72 {%012llo,%012llo}/%012llo\n", reg_A, reg_Q, word); 
    div72(reg_A, reg_Q, word, &quot, &rem);
    // printf("Div72 returns %012llo rem %012llo -- %lld rem %lld\n", quot, rem, quot, rem);
    if (rem != 0)
        if (dividend_is_neg)
            rem = negate36(rem);
    if (dividend_is_neg != divisor_is_neg)
        quot = negate36(quot);
    //double q = convert(quot, 0, 1);
    //double r = convert(rem, 0, 1);
    //printf("Result: %#012llo rem %012llo -- decimal %lld rem %lld.  Representing %g rem %g\n", quot, rem, quot, rem, q, r);

    reg_A = quot;
    reg_Q = rem;
    IR.zero = reg_A == 0;
    IR.neg = bit36_is_neg(reg_A);

    double tmp = multics_to_double(reg_A, 0, 0, 1);
    log_msg(NOTIFY_MSG, "opu::dvf", "quotient:  A = %012llo => %g\n", reg_A, tmp);
    tmp = multics_to_double(reg_Q, 0, 0, 1);
    log_msg(NOTIFY_MSG, "opu::dvf", "remainder: Q = %012llo => %g\n", reg_Q, tmp);

    return 0;
}

// ============================================================================

int instr_ufa(t_uint64 word)
{
    log_msg(NOTIFY_MSG, "opu::ufa", "E = %03o(%d) AQ = {%012llo,%012llo} aka (%lld,%lld).\n", reg_E, reg_E, reg_A, reg_Q, reg_A, reg_Q);

    // int aq_neg = bit36_is_neg(reg_A);
    uint8 op_exp = getbits36(word, 0, 8);
    uint32 op_mant = getbits36(word, 8, 28);    // 36-8=28 bits
    log_msg(NOTIFY_MSG, "opu::ufa", "op = %012llo => exp %03o(%d) and mantissa %010o (%d)\n", word, op_exp, (int8) op_exp, op_mant, op_mant);

    if (op_mant == 0) {
        // short circuit the addition of zero
        IR.zero = reg_A == 0 && reg_Q == 0;
        IR.neg = bit36_is_neg(reg_A);
        IR.exp_overflow = 0;
        IR.exp_underflow = 0;
        IR.carry = 0;
        log_msg(NOTIFY_MSG, "opu::ufa", "addition of zero, short circuiting otherwise unimplemented code.\n");
        return 0;
    }

    uint8 new_e;
    if ((int8) op_exp < (int8) reg_E) {
        // operand has the smaller exponent
        new_e = reg_E;
        int n =(int8) reg_E - (int8) op_exp;
        op_mant >>= n;
        log_msg(NOTIFY_MSG, "opu::ufa", "exp diff is %d; op mantissa now %010o\n", n, op_mant);
    } else {
        // AQ has the smaller exponent
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

    return 1;   // untested
}

// ============================================================================

int instr_ufm(t_uint64 word)
{
    int ret = 0;

    log_msg(NOTIFY_MSG, "opu::ufm", "E = %03o(%d) AQ = {%012llo,%012llo} aka (%lld,%lld).\n", reg_E, reg_E, reg_A, reg_Q, reg_A, reg_Q);

    // int aq_neg = bit36_is_neg(reg_A);
    uint8 op_exp = getbits36(word, 0, 8);
    t_uint64 op_mant = getbits36(word, 8, 28) << 8; // 36-8=28 bits
    log_msg(NOTIFY_MSG, "opu::ufm", "op = %012llo => exp %03o(%d) and mantissa %012llo (%lld)\n", word, op_exp, (int8) op_exp, op_mant, op_mant);

    IR.exp_underflow = 0;
    IR.exp_overflow = 0;
    int exp = (int8) reg_E + op_exp;
    // BUG: Do we need to generate faults?
    if (exp < -128) {
        IR.exp_underflow = 1;
        log_msg(NOTIFY_MSG, "opu::ufm", "exp underflow.\n");
        // ret = 1;
    } else if (exp > 127) {
        IR.exp_overflow = 1;
        log_msg(NOTIFY_MSG, "opu::ufm", "exp overflow.\n");
        // ret = 1;
    }
    reg_E = (unsigned) exp & MASKBITS(8);
    log_msg(NOTIFY_MSG, "opu::ufm", "new exp is %d aka %#o\n", exp, reg_E);

    if (reg_A == ((t_uint64) 1 << 35) && reg_Q == 0 && op_mant == ((t_uint64) 1 << 35)) {
        log_msg(NOTIFY_MSG, "opu::ufm", "Need to normalize operands....\n");
        cancel_run(STOP_BUG);
        // BUG: the following probably isn't the proper way to normalize...
        reg_A = ((t_uint64) 1 << 34);   // 0.5
        if (reg_E == 127) {
            reg_E = 128;
            IR.exp_overflow = 1;
            log_msg(NOTIFY_MSG, "opu::ufm", "exp overflow.\n");
            // ret = 1;
        } else
            ++ reg_E;
    } else {
        t_uint64 a = reg_A;
        t_uint64 q = reg_Q;
        mpy72fract(reg_A, reg_Q, op_mant, &reg_A, &reg_Q);
        log_msg(NOTIFY_MSG, "opu::ufm", "Multiplying {%012llo,%012llo} by {%012llo} yields {%012llo,%012llo}\n", a, q, op_mant, reg_A, reg_Q);
    }

    return ret;
}

// ============================================================================

int instr_fno()
{
    if (IR.overflow) {
        // Shift AQ by one and invert sign bit
        reg_Q = setbits36(reg_Q >> 1, 0, 1, reg_A & 1);
        int sign = bit36_is_neg(reg_A);
        reg_A >>= 1;
        reg_A = setbits36(reg_A, 0, 1, ! sign);
        IR.overflow = 0;
    }
    if ((IR.zero = reg_A == 0 && reg_Q == 0) != 0)
        reg_E = 0200;   // -128
    IR.neg = bit36_is_neg(reg_A);
    IR.exp_overflow = 0;
    IR.exp_underflow = 0;

    if (IR.zero) {
        log_msg(NOTIFY_MSG, "OPU::fno", "Result is zero.\n");
        return 0;
    }

    // BUG: we don't normalize
    log_msg(ERR_MSG, "OPU::fno", "Normalize not implemented.\n");
    cancel_run(STOP_BUG);
    return 1;
}

// ============================================================================

double multics_to_double(t_uint64 xhi, t_uint64 xlo, int show, int is_signed)
{
    // Convert from Multics fixed point fractional to "C" double.
    // Results will be approximate for some hardware.
    // xhi is the more significant bits just after the decimal point.

    t_uint64 orig_hi = xhi;
    t_uint64 orig_lo = xlo;
    
    xlo = setbits36(xlo, 35, 1, 0); // throw it away, use 71 bits, not 72
    // reg_Q = setbits36(reg_Q>>1, 0, 1, reg_A&1);  // BUG
    xhi = setbits36(xhi>>1, 0, 1, xhi&1);   // BUG fixed

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
                out_msg("bit %d on: val now %g\n", nbits, val);
            }
            bitsel >>= 1;
            fact *= 2;
        }
    }
    if (is_signed)
        val += sign;
    if (show) {
        //printf("%#o aka %sb => %0*o aka %sb represents %g\n", orig, bin2text(orig,word_size), word_size/3, x, bin2text(x,word_size), val);
        // printf("%#lloo => de-signed/masked %0*lloo aka %sb represents (%c) %g\n",
        //  orig, word_size/3, x, fbin2text(x,word_size), (sign == 0) ? '+' : '-', val);
        log_msg(NOTIFY_MSG, "OPU::fixed", " {%#llo,%#llo} => %g\n", orig_hi, orig_lo, val);
    }
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

/*
 * fno
NOTES: The fno instruction normalizes the number in C(EAQ) if C(AQ) ? 0 and the
overflow indicator is OFF.
A normalized floating number is defined as one whose mantissa lies in the
interval [0.5,1.0] such that
0.5 <= | C(AQ) | < 1.0
which, in turn, requires that C(AQ)0 ? C(AQ)1.
If the overflow indicator is ON, then C(AQ) is shifted one place to the right,
C(AQ)0 is inverted to reconstitute the actual sign, and the overflow
indicator is set OFF. This action makes the fno instruction useful in
correcting overflows that occur with fixed point numbers.
Normalization is performed by shifting C(AQ)1,71 one place to the left and
reducing C(E) by 1, repeatedly, until the conditions for C(AQ)0 and C(AQ)1
are met. Bits shifted out of AQ1 are lost.
If C(AQ) = 0, then C(E) is set to -128 and the zero indicator is set ON.
Attempted repetition with the rpl instruction causes an illegal procedure
fault.
*/
