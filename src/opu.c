/*
    opu.cpp -- implements instruction execution.

    Called by routines in hw6180_cpu.c.

    Some of the simpler mathematical and logical functions are implemented
    here, but more complicated math functions are provided in math.h.
    Makes heavy use of the addressing functions in apu_basic.c and
    opu_eis_mw.c

    BUG: Unclear if we need to sometimes set IR based on 18 bit values when
    an 18-bit ",dl" operand is used.   This is partially implemented when
    constant do_18bit_math is set to true.

    BUG: Create a "store_op" that checks TPR.is_value before storing 
    against TPR.CA.   This should handle most illegal modifier issues.
*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#include <ctype.h>  // for isprint
#include "hw6180.h"

static int do_18bit_math;   // diag tape seems to want this, probably inappropriately

// ============================================================================

extern uint32 sim_brk_summ;

// #define ABS(x) ( (x) >= 0 ? (x) : -(x) )

static inline t_uint64 lrotate36(t_uint64 x, unsigned n);
static inline void lrotate72(t_uint64* ap, t_uint64* bp, unsigned n);
static inline uint min(uint a, uint b);
static inline uint max(uint a, uint b);
static inline uint max3(uint a, uint b, uint c);

static int do_op(instr_t *ip);
static void scu2words(t_uint64 *words);
static void words2scu(const t_uint64 *words);
static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);
static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add18(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int sub36(t_uint64 a, t_uint64 b, t_uint64 *dest);
//int add72(t_uint64 ahi, t_uint64 alow, t_uint64* dest1, t_uint64* dest2, int is_unsigned);
static int do_epp(int epp);
static int do_eawp(int n);
static int do_epbp(int n);
static int do_easp(int n);
static int do_an_op(instr_t *ip);   // todo: hack, fold into do_op
static void spri_to_words(int reg, t_uint64* word0p, t_uint64 *word1p);
static int do_spri(int n);
static int do_spbp(int n);
static void do_ldi_ret(t_uint64 word, int is_ret);

static int op_dvf(const instr_t* ip);
static int op_ufa(const instr_t* ip, flag_t subtract);
static int op_ufm(const instr_t* ip);
static void long_right_shift(t_uint64 *hip, t_uint64 *lop, int n, int is_logical);
static void rsw_get_port_config(int low_port);
static void equalize_mantissas(const char* op, t_uint64 *a_mant, uint8 a_exp, t_uint64 *b_mant, uint8 b_exp, int* new_exp);
static void equalize_mantissas72(const char* op, t_uint64* a_mant_hip, t_uint64* a_mant_lop, uint8 a_exp, t_uint64* b_mant_hip, t_uint64* b_mant_lop, uint8 b_exp, int* new_exp);
// static int cmp_fract(t_uint64 x, t_uint64 y);

static uint saved_tro;

// BUG: move externs to hdr file
extern switches_t switches;

static t_uint64 scu_data[8];    // For SCU instruction

// ============================================================================

void execute_instr(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)
    // BUG: handle interrupt inhibit

    do_op(&cu.IR);
}

// ============================================================================

static int fetch_op(const instr_t *ip, t_uint64 *wordp)
{
    if (TPR.is_value) {
        *wordp = TPR.value;
        return 0;
    }
    return fetch_word(TPR.CA, wordp);
}


// ============================================================================

static int do_op(instr_t *ip)
{
    // Wrapper for do_an_op() with detection of change in address mode (for debugging).

    ++ sys_stats.n_instr;
#if FEAT_INSTR_STATS
    ++ sys_stats.instr[ip->opcode].nexec;
#if FEAT_INSTR_STATS_TIMING
    uint32 start = sim_os_msec();
#endif
#endif

    do_18bit_math = 0;
    // do_18bit_math = (switches.FLT_BASE != 2);    // diag tape seems to want this, probably inappropriately

    addr_modes_t orig_mode = get_addr_mode();
    int orig_ic = PPR.IC;

#if 0
    if (ip->is_eis_multiword) {
        extern DEVICE cpu_dev;
        ++opt_debug; ++ cpu_dev.dctrl;  // BUG
    }
#endif
    int ret = do_an_op(ip);
#if 0
    if (ip->is_eis_multiword) {
        extern DEVICE cpu_dev;
        --opt_debug; --cpu_dev.dctrl;   // BUG
    }
#endif

    addr_modes_t mode = get_addr_mode();
    if (orig_mode != mode) {
        if (orig_ic == PPR.IC) {
            if (cpu.trgo)
                log_msg(WARN_MSG, "OPU", "Transfer to current location detected; Resetting addr mode as if it were sequential\n");
            set_addr_mode(orig_mode);
            log_msg(DEBUG_MSG, "OPU", "Resetting addr mode for sequential instr\n");
        } else {
            log_msg(NOTIFY_MSG, "OPU", "Address mode has been changed with an IC change.  Was %#o, now %#o\n", orig_ic, PPR.IC);
            if (switches.FLT_BASE == 2) {
                cancel_run(STOP_IBKPT);
                log_msg(NOTIFY_MSG, "OPU", "Auto breakpoint.\n");
            }
        }
    }

#if FEAT_INSTR_STATS
#if FEAT_INSTR_STATS_TIMING
    sys_stats.instr[ip->opcode].nmsec += sim_os_msec() - start;
#endif
#endif

    return ret;
}


static int do_an_op(instr_t *ip)
{
    // Returns non-zero on error or non-group-7  fault
    // BUG: check for illegal modifiers

    cpu.opcode = ip->opcode;

    uint op = ip->opcode;
    char *opname = opcodes2text[op];

    cu.rpts = 0;    // current instruction isn't a repeat type instruction (as far as we know so far)
    saved_tro = IR.tally_runout;

    int bit27 = op % 2;
    op >>= 1;
    if (opname == NULL) {
        if (op == 0172 && bit27 == 1) {
            log_msg(WARN_MSG, "OPU", "Unavailable instruction 0172(1).  The ldo  instruction is only available on ADP aka ORION aka DPS88.  Ignoring instruction.\n");
            cancel_run(STOP_BUG);
            return 0;
        }
        log_msg(WARN_MSG, "OPU", "Illegal opcode %03o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    } else {
        if (opt_debug) log_msg(DEBUG_MSG, "OPU", "Opcode %#o(%d) -- %s\n", op, bit27, instr2text(ip));
    }
    
    // Check instr type for format before addr_mod
    // Todo: check efficiency of lookup table versus switch table
    // Also consider placing calls to addr_mod() in next switch table
    flag_t initial_tally = IR.tally_runout;
    cpu.poa = 1;        // prepare operand address flag
    if (ip->is_eis_multiword) {
        log_msg(DEBUG_MSG, "OPU", "Skipping addr_mod() for EIS instr.\n");
    } else if (bit27 == 0) {
        switch (op) {
            case opcode0_epp0 + 0:  // BUG: this construct just used to allow counting how many opcodes are implemented
            case opcode0_epp2 + 0:
            case opcode0_epp4 + 0:
            case opcode0_epp6 + 0:
                addr_mod(ip);       // note that ip == &cu.IR
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
    } else {
        switch (op) {
            case opcode1_a4bd + 0:  // BUG: this construct just used to allow counting how many opcodes are implemented
            case opcode1_a6bd + 0:
            case opcode1_a9bd + 0:
            case opcode1_abd + 0:
            case opcode1_awd + 0:
            case opcode1_s9bd + 0:
                addr_mod_eis_addr_reg(ip);
                break;
            case opcode1_epp1 + 0:
            case opcode1_epp3 + 0:
            case opcode1_epp5 + 0:
            case opcode1_epp7 + 0:
                addr_mod(ip);       // note that ip == &cu.IR
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
    }
    cpu.poa = 0;
    
    if (bit27 == 0) {
        switch (op) {
            case opcode0_eaa:
                reg_A = (t_uint64) TPR.CA << 18;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            case opcode0_eaq:
                reg_Q = (t_uint64) TPR.CA << 18;
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return 0;
            case opcode0_eax0:
            case opcode0_eax1:
            case opcode0_eax2:
            case opcode0_eax3:
            case opcode0_eax4:
            case opcode0_eax5:
            case opcode0_eax6:
            case opcode0_eax7: {
                int n = op & 07;
                reg_X[n] = TPR.CA;
                IR.zero = reg_X[n] == 0;
                IR.neg = bit18_is_neg(reg_X[n]);
                return 0;
            }

            case opcode0_lca: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_A = negate36(word);
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                    IR.overflow = reg_A == ((t_uint64)1<<35);
                }
                return ret;
            }

            // opcode0_lcaq unimplemented

            case opcode0_lcq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_Q = negate36(word);
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
                    IR.overflow = reg_Q == ((t_uint64)1<<35);
                }
                return ret;
            }

            case opcode0_lcx0:
            case opcode0_lcx1:
            case opcode0_lcx2:
            case opcode0_lcx3:
            case opcode0_lcx4:
            case opcode0_lcx5:
            case opcode0_lcx6:
            case opcode0_lcx7: {    // Load Complement (into) Index Register N
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    //reg_X[n] = (word >> 18) & MASK18; // reg is 18 bits
                    //reg_X[n] = (((~ reg_X[n]) & MASK18) + 1) & MASK18;
                    reg_X[n] = negate18(word >> 18);
                    log_msg(INFO_MSG, "OPU::instr::lcx*", "X[%d]: Loaded complement of %#llo => %#llo(%d); result is %#o(%d).\n", n, word, word >> 18, sign18(word >>18), reg_X[n], sign18(reg_X[n]));
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }

            case opcode0_lda: {
                int ret = fetch_op(ip, &reg_A);
                if (ret == 0) {
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_ldac: {
                int ret = fetch_op(ip, &reg_A);
                if (ret == 0) {
                    ret = store_word(TPR.CA, 0);
                    if (ret == 0) {
                        IR.zero = reg_A == 0;
                        IR.neg = bit36_is_neg(reg_A);
                    }
                }
                return ret;
            }
            case opcode0_ldaq: {    // Load AQ reg
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    reg_A = word1;
                    reg_Q = word2;
                    IR.zero = word1 == 0 && word2 == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_ldi: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0)
                    do_ldi_ret(word, 0);
                return ret;
            }

            case opcode0_ldq: { // load Q reg
                int ret = fetch_op(ip, &reg_Q);
                if (ret == 0) {
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
                }
                return ret;
            }
            case opcode0_ldqc: {    // load Q reg & clear
                int ret = fetch_op(ip, &reg_Q);
                if (ret == 0) {
                    ret = store_word(TPR.CA, 0);
                }
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return ret;
            }
            case opcode0_ldx0:  // Load X[n]
            case opcode0_ldx1:
            case opcode0_ldx2:
            case opcode0_ldx3:
            case opcode0_ldx4:
            case opcode0_ldx5:
            case opcode0_ldx6:
            case opcode0_ldx7: {
                // BUG: manual says bits 0..17, but alm program trying to use constant 2.  Resolved?
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_X[n] = (word >> 18) & MASK18;   // reg is 18 bits
                    log_msg(DEBUG_MSG, "OPU::instr::ldx*", "X[%d]: Loaded %#llo => %#llo (%#o aka %#o)\n", n, word, word >> 18, reg_X[n], reg_X[n] & MASK18);
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }
            case opcode0_lreg: {
                t_uint64 words[8];
                int ret;
                if ((ret = fetch_yblock8(TPR.CA, words)) == 0) {
                    reg_X[0] = words[0] >> 18;
                    reg_X[1] = words[0] & MASK18;
                    reg_X[2] = words[1] >> 18;
                    reg_X[3] = words[1] & MASK18;
                    reg_X[4] = words[2] >> 18;
                    reg_X[5] = words[2] & MASK18;
                    reg_X[6] = words[3] >> 18;
                    reg_X[7] = words[3] & MASK18;
                    reg_A = words[4];
                    reg_Q = words[5];
                    reg_E = getbits36(words[6], 0, 8);
                    // NOTE: sreg stors TR and RALR, but lreg does not restore them
                }
                return ret;
            }

            case opcode0_lxl0:  // Load Index Reg N from lower
            case opcode0_lxl1:
            case opcode0_lxl2:
            case opcode0_lxl3:
            case opcode0_lxl4:
            case opcode0_lxl5:
            case opcode0_lxl6:
            case opcode0_lxl7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0)
                    reg_X[n] = word & MASK18;
                IR.zero = reg_X[n] == 0;
                IR.neg = bit18_is_neg(reg_X[n]);
                return ret;
            }
            case opcode0_sreg: {
                t_uint64 words[8];
                words[0] = ((t_uint64) reg_X[0] << 18) | reg_X[1];
                words[1] = ((t_uint64) reg_X[2] << 18) | reg_X[3];
                words[2] = ((t_uint64) reg_X[4] << 18) | reg_X[5];
                words[3] = ((t_uint64) reg_X[6] << 18) | reg_X[7];
                words[4] = reg_A;
                words[5] = reg_Q;
                words[6] = setbits36(0, 0, 8, reg_E);
                words[7] = setbits36(0, 0, 27, reg_TR);
                words[7] = setbits36(words[7], 33, 3, reg_RALR);
                return store_yblock8(TPR.CA, words);
            }
            case opcode0_sta:
                return store_word(TPR.CA, reg_A);
            case opcode0_stac: {
                // BUG: check for illegal modifiers
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0 && word == 0)
                    ret = store_word(TPR.CA, reg_A);
                IR.zero = word == 0;
                return ret;
            }
            case opcode0_stacq: {
                // BUG: check for illegal modifiers
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    IR.zero = word == reg_Q;
                    if (word == reg_Q)
                        ret = store_word(TPR.CA, reg_A);
                }
                return ret;
            }
            case opcode0_staq: {
                // BUG: check for illegal modifiers
                t_uint64 word1, word2;
                uint y = TPR.CA - TPR.CA % 2;   // force even
                int ret = store_word(y, reg_A);
                if (ret == 0)
                    ret = store_word(y+1, reg_Q);
                return ret;
            }
            case opcode0_stba: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    int tag = ip->mods.single.tag;
                    for (int i = 0; i < 4; ++i) {
                        if ((tag & 040) != 0)
                            word = setbits36(word, i * 9, 9, getbits36(reg_A, i * 9, 9));
                        tag <<= 1;
                    }
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }
            case opcode0_stbq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    int tag = ip->mods.single.tag;
                    for (int i = 0; i < 4; ++i) {
                        if ((tag & 040) != 0)
                            word = setbits36(word, i * 9, 9, getbits36(reg_Q, i * 9, 9));
                        tag <<= 1;
                    }
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }
            case opcode0_stc1: {
                t_uint64 word;
                save_IR(&word);
                word = setbits36(word, 25, 1, initial_tally);
                word |= ((t_uint64)(PPR.IC + 1) << 18);
                // log_msg(DEBUG_MSG, "OPU::stct1", "saving %012llo to %#o\n", word, TPR.CA);
                return store_word(TPR.CA, word);
            }
            case opcode0_stc2: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word &= MASK18;
                    word |= ((t_uint64) (PPR.IC + 2) << 18);
                }
                return store_word(TPR.CA, word);
            }
            case opcode0_stca: {
                // no modifiers
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                if (ret == 0) {
                    for (int i = 0; i < 6; ++i) {
                        if ((ip->mods.single.tag & (1<<(5-i))) != 0)
                            word = setbits36(word, i * 6, 6, getbits36(reg_A, i * 6, 6));
                    }
                }
                ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_stcq: {
                // no modifiers
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                t_uint64 orig = word;
                if (ret == 0) {
                    for (int i = 0; i < 6; ++i)
                        if ((ip->mods.single.tag & (1 << (5-i))) != 0)
                            word = setbits36(word, i * 6, 6, getbits36(reg_Q, i * 6, 6));
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            case opcode0_stcd: {
                t_uint64 word0 = setbits36(0, 3, 15, PPR.PSR);
                word0 = setbits36(word0, 16, 3, PPR.PRR);
                word0 |= 043;
                t_uint64 word1 = setbits36(0, 0, 18, PPR.IC + 2);
                cancel_run(STOP_IBKPT);
                return store_pair(TPR.CA, word0, word1);
            }

            case opcode0_sti: {
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                if (ret == 0) {
                    t_uint64 ir;
                    uint tro = IR.tally_runout;
                    IR.tally_runout = saved_tro;
                    flag_t abs = IR.abs_mode;
#if 1
                    // The DH02-01 DPS8 ASM manual states that
                    // bit 31 (absolute mode) is cleared.  AL-39
                    // claims that the all 18 bits are stored, but
                    // the T&D tape insists that bit 31 be cleared.
                    IR.abs_mode = 0;
#endif
                    save_IR(&ir);
                    IR.tally_runout = tro;
                    IR.abs_mode = abs;
                    word = setbits36(word, 18, 18, ir);
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }
            case opcode0_stq:   // Store Q register
                return store_word(TPR.CA, reg_Q);

            // opcode0_stt unimplemented

            case opcode0_stx0:
            case opcode0_stx1:
            case opcode0_stx2:
            case opcode0_stx3:
            case opcode0_stx4:
            case opcode0_stx5:
            case opcode0_stx6:
            case opcode0_stx7: {
                int n = op & 07;
                t_uint64 word;
                log_msg(DEBUG_MSG, "OPU::stx*", "fetching word\n");
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = (word & MASK18) | ((t_uint64) reg_X[n] << 18);
                    log_msg(DEBUG_MSG, "OPU::stx*", "storing word\n");
                    ret = store_word(TPR.CA, word);
                }
                log_msg(DEBUG_MSG, "OPU::stx*", "done\n");
                return ret;
            }
            case opcode0_stz:   // store zero
                return store_word(TPR.CA, 0);
            case opcode0_sxl0:
            case opcode0_sxl1:
            case opcode0_sxl2:
            case opcode0_sxl3:
            case opcode0_sxl4:
            case opcode0_sxl5:
            case opcode0_sxl6:
            case opcode0_sxl7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = (word & ((t_uint64)MASK18<<18)) | reg_X[n];
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            case opcode0_alr: {
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                reg_A = lrotate36(reg_A, n);
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            }
            case opcode0_als: { // A reg left shift
                int n = TPR.CA & 0177;  // bits 11..17 of 18bit CA
                //log_msg(DEBUG_MSG, "OPU::als", "CA = %#o; bits 11..17 = %0o\n", TPR.CA, n);
                //log_msg(DEBUG_MSG, "OPU::als", "A = (%#llo << %d) ==> %#llo\n", reg_A, n, (reg_A << n) & MASK36);
                flag_t init_neg = bit36_is_neg(reg_A);
                reg_A = (reg_A << n) & MASK36;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_arl: { // A reg right logical shift
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                reg_A >>= n;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);   // possible for "shift" of zero bits
                return 0;
            }
            case opcode0_ars: {
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                uint fill = bit36_is_neg(reg_A);
                t_uint64 tmp = reg_A;
                reg_A >>= n;
                if (fill)
                    reg_A = setbits36(reg_A, 0, n, MASKBITS(n));
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                log_msg(DEBUG_MSG, "OPU::ars", "%012llo>>%d ==> %012llo\n", tmp, n, reg_A);
                return 0;
            }
            case opcode0_llr: {     // Long left rotate
                flag_t init_neg = bit36_is_neg(reg_A);
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                lrotate72(&reg_A, &reg_Q, n);
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                // IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_lls: {     // Long left shift
                flag_t init_neg = bit36_is_neg(reg_A);
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
#if 1
                t_uint64 asv, qsv;  // BUG: temp
                asv = reg_A; qsv = reg_Q;
                if (n >= 72)
                    n %= 72;
                if (n != 0) {
                    reg_A = ((reg_A << n) & MASK36) | (reg_Q >> (36 - n));
                    reg_Q <<= n;
                }
#endif
                n = TPR.CA & 0177;  // bits 11..17 of 18bit CA
                t_uint64 a = reg_A;
                t_uint64 q = reg_Q;
                reg_A = asv;
                reg_Q = qsv;
                if (n >= 72) {
                    log_msg(INFO_MSG, "OPU::lls", "Shift of %d bits.\n", n);
                    reg_A = 0;
                    reg_Q = 0;
                } else if (n != 0) {
                    if (n <= 36) {
                        reg_A = ((reg_A << n) & MASK36) | (reg_Q >> (36 - n));
                        reg_Q <<= n;
                    } else {
                        // reg_Q = setbits36(0, 0, 72-n, reg_A);
                        reg_A = (reg_Q << (n - 36)) & MASK36;
                        reg_Q = 0;
                    }
                }
                if (reg_A != a || reg_Q != q) {
                    log_msg(INFO_MSG, "OPU::lls", "BUG Fix: Shift of %012llo,%012llo by %d bits\n", asv, qsv, n);
                    log_msg(INFO_MSG, "OPU::lls", "Prior result was: %012llo,%012llo\n", a, q);
                    log_msg(INFO_MSG, "OPU::lls", "Fixed result is:  %012llo,%012llo\n", reg_A, reg_Q);
                }

                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }

            case opcode0_lrl: {     // long right logical
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                flag_t init_neg = bit36_is_neg(reg_A);
                long_right_shift(&reg_A, &reg_Q, n, 1);
                IR.zero= reg_A == 0 && reg_Q == 0;
                IR.neg = init_neg;
                return 0;
            }

            case opcode0_lrs: {     // Long right shift
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                flag_t init_neg = bit36_is_neg(reg_A);
                long_right_shift(&reg_A, &reg_Q, n, 0);
                IR.zero= reg_A == 0 && reg_Q == 0;
                IR.neg = init_neg;
                return 0;
            }

            case opcode0_qlr: {
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                reg_Q = lrotate36(reg_Q, n);
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return 0;
            }
            case opcode0_qls: { // Q reg left shift
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                flag_t init_neg = bit36_is_neg(reg_Q);
                reg_Q = (reg_Q << n) & MASK36;
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_qrl: {
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                t_uint64 qold = reg_Q;
                reg_Q >>= n;
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return 0;
            }
            case opcode0_qrs: { // Q Right Shift (with sign fill)
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                flag_t init_neg = bit36_is_neg(reg_Q);
                reg_Q >>= n;
                if (init_neg) {
                    reg_Q |= ~ (MASK36 >> n);
                    IR.zero = 0;
                    IR.neg = 1;
                } else {
                    IR.zero = reg_Q == 0;
                    IR.neg = 0;
                }
                return 0;
            }
            case opcode0_ada:
                return op_add(ip, &reg_A);

            case opcode0_adaq: {    // Add to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    ret = add72(word1, word2, &reg_A, &reg_Q, 0);
                }
                return ret;
            }

            case opcode0_adl: { // Add low to AQ
                t_uint64 word1, word2;
                int ret = fetch_op(ip, &word2);
                if (ret == 0) {
                    // BUG: ignoring 18bit math
                    word1 = bit36_is_neg(word2) ? MASK36 : 0;
                    ret = add72(word1, word2, &reg_A, &reg_Q, 0);
                }
                return ret;
            }

            case opcode0_adla: {    // Add logical to A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 a = reg_A;
                    reg_A += word;
                    if ((IR.carry = ((reg_A & MASK36) != 0)))
                        reg_A &= MASK36;
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
#if 0
                    if (do_18bit_math && TPR.is_value == 7 && a == 0) {
                        int is_neg = bit18_is_neg(reg_A);
                        if (is_neg != IR.neg) {
                            log_msg(WARN_MSG, "OPU::adla", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                            IR.neg = is_neg;
                            cancel_run(STOP_WARN);
                        }
                    }
#endif
                }
                return ret;
            }
            case opcode0_adlaq: {   // Add logical to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    // BUG: ignoring 18bit math
                    ret = add72(word1, word2, &reg_A, &reg_Q, 1);
                }
                return ret;
            }

            case opcode0_adlq: {    // Add logical to Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 q = reg_Q;
                    reg_Q += word;
                    if ((IR.carry = ((reg_Q & MASK36) != 0)))
                        reg_Q &= MASK36;
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
#if 0
                    if (do_18bit_math && TPR.is_value == 7 && q == 0) {
                        int is_neg = bit18_is_neg(reg_Q);
                        if (is_neg != IR.neg) {
                            log_msg(WARN_MSG, "OPU::adlq", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                            IR.neg = is_neg;
                            cancel_run(STOP_WARN);
                        }
                    }
#endif
                }
                return ret;
            }

            case opcode0_adlx0:     // Add logical to X[n]
            case opcode0_adlx1:
            case opcode0_adlx2:
            case opcode0_adlx3:
            case opcode0_adlx4:
            case opcode0_adlx5:
            case opcode0_adlx6:
            case opcode0_adlx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = getbits36(word, 0, 18) + reg_X[n];
                    if ((IR.carry = ((word & MASK18) != 0)))
                        word &= MASK18;
                    reg_X[n] = word;
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(word);
                }
                return ret;
            }
            case opcode0_adq:
                return op_add(ip, &reg_Q);
            case opcode0_adx0:  // Add to X[n]
            case opcode0_adx1:
            case opcode0_adx2:
            case opcode0_adx3:
            case opcode0_adx4:
            case opcode0_adx5:
            case opcode0_adx6:
            case opcode0_adx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0)
                    word >>= 18;
                    if ((ret = add18(word, reg_X[n], &word)) == 0)
                        reg_X[n] = word;
                return ret;
            }
            case opcode0_aos: { // Add one to Storage
                t_uint64 word = 1;
                int ret = op_add(ip, &word);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_asa: { // Add Stored to A reg
                t_uint64 word = reg_A;
                int ret = op_add(ip, &word);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_asq: { // Add Stored to Q
                t_uint64 word = reg_Q;
                int ret = op_add(ip, &word);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }


            case opcode0_asx0:
            case opcode0_asx1:
            case opcode0_asx2:
            case opcode0_asx3:
            case opcode0_asx4:
            case opcode0_asx5:
            case opcode0_asx6:
            case opcode0_asx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 w = word >> 18;
                    if ((ret = add18(w, reg_X[n], &w)) == 0) {
                        word = (w << 18) | (word & MASK18);
                        ret = store_word(TPR.CA, word);
                    }
                }
                return ret;
            }

            // case opcode0_awca unimplemented
            // case opcode0_awcq unimplemented

            case opcode0_sba: { // Subtract from A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
#if 0
                    word = negate36(word);
                    ret = add36(reg_A, word, &reg_A);
#endif
#if 1
                    ret = sub36(reg_A, word, &reg_A);
#else
            ret = sub36(reg_A, word, &reg_A);
                    // AL39, section two claims that subtraction uses 
                    // two's complement.   However, it's not that simple.
                    // Diagnostic tape t4d_b.2.tap expects that
                    // <max negative number> minus zero will generate
                    // a carry.   So, instead of adding in the two's
                    // complement, we'll first add the one's complement
                    // and then add one.  The diagnostic tape also expects
                    // that negative one minus zero will yield a carry.
                    t_uint64 w = word;
                    t_uint64 a = reg_A;
                    word = (~ word) & MASK36;
                    ret = add36(reg_A, word, &reg_A);
                    int carry = IR.carry;
                    log_msg(DEBUG_MSG, "OPU::sba", "%012llo - %012llo ==> adding %012llo yields %012llo with carry=%c.\n", a, w, word, reg_A, IR.carry ? 'Y' : 'N');
                    if (ret == 0) {
                        word = 1;
                        ret = add36(reg_A, word, &reg_A);
                        log_msg(DEBUG_MSG, "OPU::sba", "adding one yields %012llo with carry=%c.\n", reg_A, IR.carry ? 'Y' : 'N');
                        IR.carry |= carry;
#if 0
                        if (do_18bit_math && TPR.is_value == 7 && (a>>18) == 0) {
                            // arithmetic on "dl" constant
                            if (opt_debug || (a>>18) != 0 || (w >> 18) != 0 || (reg_A>>18) != 0)
                                log_msg(NOTIFY_MSG, "OPU::sba", "A = %012llo minus %06llo ,du operand yields %012llo at IC %06o \n", a, w, reg_A, PPR.IC);
#if 0
                            flag_t is_neg = bit18_is_neg(reg_A);
                            if (is_neg != IR.neg) {
                                log_msg(WARN_MSG, "OPU::sba", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                                IR.neg = is_neg;
                                // cancel_run(STOP_WARN);
                            }
#endif
                            if ((reg_A >> 18) != 0 && ! IR.carry) {
                                log_msg(WARN_MSG, "OPU::sba", "Carry may be wrong for ,dl operand.\n");
                                cancel_run(STOP_WARN);
                            }
                        }
#endif
                    }
#endif
                }
                return ret;
            }
            case opcode0_sbaq: {    // Subtract from AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    // BUG: Ignoring 18bit math
                    negate72(&word1, &word2);
                    ret = add72(word1, word2, &reg_A, &reg_Q, 0);
                }
                return ret;
            }

            case opcode0_sbla: {        // Subtract logical from A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    uint sign = reg_A >> 35;
                    reg_A = (reg_A - word) & MASK36;
                    uint rsign = reg_A >> 35;
                    IR.zero = reg_A == 0;
                    IR.neg = rsign;
                    IR.carry = sign != rsign;
                }
                return ret;
            }

            // sblaq unimplemented

            case opcode0_sblq: {    // Subtract logical from Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 q = reg_Q;
                    uint sign = reg_Q >> 35;
                    reg_Q = (reg_Q - word) & MASK36;
                    uint rsign = reg_Q >> 35;
                    IR.zero = reg_Q == 0;
                    IR.neg = rsign;
                    IR.carry = sign != rsign;
#if 0
                    if (do_18bit_math && TPR.is_value == 7 && (q>>18) == 0) {
                        int is_neg = bit18_is_neg(reg_Q);
                        if (is_neg != IR.neg) {
                            log_msg(WARN_MSG, "OPU::sba", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                            IR.neg = is_neg;
 
                        }
                    }
#endif
                }
                return ret;
            }
            case opcode0_sblx0:
            case opcode0_sblx1:
            case opcode0_sblx2:
            case opcode0_sblx3:
            case opcode0_sblx4:
            case opcode0_sblx5:
            case opcode0_sblx6:
            case opcode0_sblx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word >>= 18;
                    uint sign = reg_X[n] >> 17;
                    reg_X[n] = (reg_X[n] - word) & MASK18;
                    uint rsign = reg_X[n] >> 17;
                    IR.zero = reg_X[n] == 0;
                    IR.neg = rsign;
                    IR.carry = sign != rsign;
                }
                return ret;
            }
            case opcode0_sbq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
#if 0
                    word = negate36(word);
                    ret = add36(reg_Q, word, &reg_Q);
#else
                    // See comments at opcode0_sba.
                    t_uint64 q = reg_Q;
                    t_uint64 w = word;
                    word = (~ word) & MASK36;
                    ret = add36(reg_Q, word, &reg_Q);
                    int carry = IR.carry;
                    log_msg(DEBUG_MSG, "OPU::sbq", "%012llo - %012llo ==> adding %012llo yields %012llo with carry=%c.\n", q, w, word, reg_Q, IR.carry ? 'Y' : 'N');
                    if (ret == 0) {
                        word = 1;
                        ret = add36(reg_Q, word, &reg_Q);
                        log_msg(DEBUG_MSG, "OPU::sbq", "adding one yields %012llo with carry=%c.\n", reg_Q, IR.carry ? 'Y' : 'N');
                        IR.carry |= carry;
#if 0
                        if (do_18bit_math && TPR.is_value == 7 && (q>>18) == 0) {
                            // arithmetic on "dl" constant
                            if (opt_debug || (q>>18) != 0 || (w >> 18) != 0 || (reg_Q>>18) != 0)
                                log_msg(NOTIFY_MSG, "OPU::sbq", "A = %012llo minus %06llo ,du operand yields %012llo at IC %06o \n", q, w, reg_Q, PPR.IC);
                            flag_t is_neg = bit18_is_neg(reg_Q);
                            if (is_neg != IR.neg) {
                                log_msg(WARN_MSG, "OPU::sbq", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                                IR.neg = is_neg;
                                // cancel_run(STOP_WARN);
                            }
                            if ((reg_Q >> 18) != 0 && ! IR.carry) {
                                log_msg(WARN_MSG, "OPU::sbq", "Carry may be wrong for ,dl operand.\n");
                                cancel_run(STOP_WARN);
                            }
                        }
#endif
                    }
#endif
                }
                return ret;
            }
            case opcode0_sbx0:
            case opcode0_sbx1:
            case opcode0_sbx2:
            case opcode0_sbx3:
            case opcode0_sbx4:
            case opcode0_sbx5:
            case opcode0_sbx6:
            case opcode0_sbx7: {
                t_uint64 word;
                int n = op & 07;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 orig = word;
                    t_uint64 result;
                    word = negate18(word >> 18);
                    if ((ret = add18(word, reg_X[n], &result)) == 0) {
#if 0
                        char buf[40];
                        sprintf(buf, "OPU::sbx%d", n);
                        log_msg(DEBUG_MSG, buf, "X[%d] -= %#llo(%lld) ==> %#o+%#llo == %d+%d = %#llo (%lld)\n",
                            n, orig, orig,
                            reg_X[n], word, reg_X[n], sign18(word),
                            result, result);
                        log_msg(DEBUG_MSG, buf, "IR: zero=%d, neg=%d, carry=%d\n", IR.zero, IR.neg, IR.carry);
#endif
                        reg_X[n] = result;
                    }
                }
                return ret;
            }

            case opcode0_ssa: { // Subtract Stored from  A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    ret = sub36(reg_A, word, &word);
                    if (ret == 0)
                        ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            case opcode0_ssq: { // Subtract Stored from  Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
                    ret = sub36(reg_Q, word, &word);
                    -- opt_debug; -- cpu_dev.dctrl;
                    if (ret == 0)
                        ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            //ssx0 ..ssx7 unimplemented
            //swca unimplemented
            //swcq unimplemented

            // opcode_mpf unimplemented -- multiply fraction

            case opcode0_mpy: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_uint64 q = reg_Q;
                    mpy(sign36(word), sign36(reg_Q), &reg_A, &reg_Q);
                    IR.zero = reg_Q == 0 && reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                    log_msg(DEBUG_MSG, "OPU::mpy", "%#llo * %#llo ===> A=%#llo, Q=%#llo\n", word, q, reg_A, reg_Q);
                    log_msg(DEBUG_MSG, "OPU::mpy", "%lld * %lld ===> A=%lld, Q=%lld\n", word, q, reg_A, reg_Q);
                }
                return ret;
            }
            case opcode0_div: {
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    // t_int64 q = bit36_is_neg(reg_Q) ? negate36(reg_Q) : reg_Q;
                    // t_int64 w = bit36_is_neg(word) ? negate36(word) : word;
                    t_int64 q = sign36(reg_Q);
                    t_int64 w = sign36(word);
                    if (w == 0 || (reg_Q == ((t_uint64)1<<35) && w == -1)) {    // (1<<35) signed is -2**36
                        fault_gen(div_fault);
                        IR.neg = bit36_is_neg(reg_Q);
                        reg_Q = (IR.neg) ? (t_uint64) negate36(reg_Q) : reg_Q; // magnitude, absolute value
                        IR.zero = w == 0;
                        ret = 1;
                    } else {
                        log_msg(DEBUG_MSG, "OPU::div", "%#llo/%#llo => %#llo/%#llo)\n", reg_Q, word, q, w);
                        reg_Q = (q / w) & MASK36;
                        reg_A = (q % w) & MASK36;
                        IR.zero = reg_Q == 0;
                        IR.neg = bit36_is_neg(reg_Q);
                    }
                }
                return ret;
            }

            case opcode0_dvf: {         // divide fraction
                int ret = op_dvf(ip);
                return ret;
            }

            case opcode0_neg:
                if (reg_A == 0) {
                    IR.zero = 1;
                } else {
                    IR.zero = 0;
                    IR.overflow = reg_A == ((t_uint64)1<<35);
                    reg_A = negate36(reg_A);
                    IR.neg = bit36_is_neg(reg_A);
                    // BUG: Should we fault?  Maximum negative number can't be negated, but AL39 doesn't say to fault
                }
                return 0;
            case opcode0_negl:
                if (reg_A == 0 && reg_Q == 0) {
                    IR.zero = 1;
                } else {
                    IR.zero = 0;
                    IR.overflow = reg_A == ((t_uint64)1<<35) && reg_Q == 0;
                    negate72(&reg_A, &reg_Q);
                    IR.neg = bit36_is_neg(reg_A);
                }
                return 0;

            // opcode0_cmg unimplemented
            
            case opcode0_cmk: {
                // Note that the cleaned up AL39.pdf has a typo in the equation.
                // The original AL39-01C_multicsProcMan_Nov85.pdf is correct.
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = (~ reg_Q) & (reg_A ^ word) & MASK36;
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
            }

            case opcode0_cmpa: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    int neg1 = bit36_is_neg(reg_A);
                    int neg2 = bit36_is_neg(word);
                    if (neg1 == 0 && neg2 == 1) {
                        IR.zero = 0;
                        IR.neg = 0;
                        IR.carry = 0;
                    } else if (neg1 == neg2) {
                        if (reg_A > word) {
                            IR.zero = 0;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else if (reg_A == word) {
                            IR.zero = 1;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else {
                            IR.zero = 0;
                            IR.neg = 1;
                            IR.carry = 0;
                        }
                    } else {
                        IR.zero = 0;
                        IR.neg = 1;
                        IR.carry = 1;
                    }
                }
                if (opt_debug)
                    log_msg(DEBUG_MSG, "OPU::cmpa", "IR: zero=%d, negative=%d, carry=%d\n", IR.zero, IR.neg, IR.carry);
                return ret;
            }
            case opcode0_cmpaq: {
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    if (opt_debug) log_msg(DEBUG_MSG, "OPU::cmpaq", "AQ = {%012llo, %012llo}\n", reg_A, reg_Q);
                    if (opt_debug) log_msg(DEBUG_MSG, "OPU::cmpaq", "Y =  {%012llo, %012llo}\n", word1, word2);
                    int neg1 = bit36_is_neg(reg_A);
                    int neg2 = bit36_is_neg(word1);
                    if (neg1 == 0 && neg2 == 1) {
                        IR.zero = 0;
                        IR.neg = 0;
                        IR.carry = 0;
                    } else if (neg1 == neg2) {
                        if (reg_A > word1) {
                            IR.zero = 0;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else if (reg_A == word1) {
                            if (reg_Q > word2) {
                                IR.zero = 0;
                                IR.neg = 0;
                                IR.carry = 1;
                            } else if (reg_Q == word2) {
                                IR.zero = 1;
                                IR.neg = 0;
                                IR.carry = 1;
                            } else {
                                IR.zero = 0;
                                IR.neg = 1;
                                IR.carry = 1;
                            }
                        } else {
                            IR.zero = 0;
                            IR.neg = 1;
                            IR.carry = 0;
                        }
                    } else {
                        IR.zero = 0;
                        IR.neg = 1;
                        IR.carry = 1;
                    }
                }
                if (opt_debug) log_msg(DEBUG_MSG, "OPU::cmpaq", "Z=%d N=%d C=%d\n", IR.zero, IR.neg, IR.carry);
                return ret;
            }
            case opcode0_cmpq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    int neg1 = bit36_is_neg(reg_Q);
                    int neg2 = bit36_is_neg(word);
                    if (neg1 == 0 && neg2 == 1) {
                        IR.zero = 0;
                        IR.neg = 0;
                        IR.carry = 0;
                    } else if (neg1 == neg2) {
                        if (reg_Q > word) {
                            IR.zero = 0;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else if (reg_Q == word) {
                            IR.zero = 1;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else {
                            IR.zero = 0;
                            IR.neg = 1;
                            IR.carry = 0;
                        }
                    } else {
                        IR.zero = 0;
                        IR.neg = 1;
                        IR.carry = 1;
                    }
                }
                if (opt_debug)
                    log_msg(DEBUG_MSG, "OPU::cmpq", "IR: zero=%d, negative=%d, carry=%d\n", IR.zero, IR.neg, IR.carry);
                return ret;
            }
            case opcode0_cmpx0:
            case opcode0_cmpx1:
            case opcode0_cmpx2:
            case opcode0_cmpx3:
            case opcode0_cmpx4:
            case opcode0_cmpx5:
            case opcode0_cmpx6:
            case opcode0_cmpx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                word >>= 18;
                if (ret == 0) {
                    int neg1 = bit18_is_neg(reg_X[n]);
                    int neg2 = bit18_is_neg(word);
                    if (neg1 == 0 && neg2 == 1) {
                        IR.zero = 0;
                        IR.neg = 0;
                        IR.carry = 0;
                    } else if (neg1 == neg2) {
                        if (reg_X[n] > word) {
                            IR.zero = 0;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else if (reg_X[n] == word) {
                            IR.zero = 1;
                            IR.neg = 0;
                            IR.carry = 1;
                        } else {
                            IR.zero = 0;
                            IR.neg = 1;
                            IR.carry = 0;
                        }
                    } else {
                        IR.zero = 0;
                        IR.neg = 1;
                        IR.carry = 1;
                    }
                }
                return ret;
            }

            // opcode0_cwl unimplemented

            case opcode0_szn: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
#if 0
                    if (do_18bit_math && TPR.is_value == 7) {
                        // Why use szn with a constant?
                        log_msg(WARN_MSG, "OPU::szn", "Using ,dl operand.\n");
                        if ((word>>18) == 0) {
                            int is_neg = bit18_is_neg(reg_Q);
                            if (is_neg != IR.neg) {
                                log_msg(WARN_MSG, "OPU::szn", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                                IR.neg = is_neg;
                            }
                        }
                    }
#endif
                }
                return ret;
            }
            case opcode0_sznc: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    ret = store_word(TPR.CA, 0);
                    if (ret == 0) {
                        IR.zero = word == 0;
                        IR.neg = bit36_is_neg(word);
#if 0
                        if (do_18bit_math && TPR.is_value == 7) {
                            log_msg(WARN_MSG, "OPU::sznc", "Using ,dl operand.\n");
                            if ((word>>18) == 0) {
                                int is_neg = bit18_is_neg(reg_Q);
                                if (is_neg != IR.neg) {
                                    log_msg(WARN_MSG, "OPU::szn", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                                    IR.neg = is_neg;
                                }
                            }
                        }
#endif
                    }
                }
                return ret;
            }

            case opcode0_ana:
                return op_and(ip, &reg_A, NULL, &reg_A, NULL);
            case opcode0_anaq:  // y-pair to A & Q
                return op_and(ip, &reg_A, &reg_Q, &reg_A, &reg_Q);
            case opcode0_anq:
                return op_and(ip, &reg_Q, NULL, &reg_Q, NULL);
            case opcode0_ansa: {
                t_uint64 word;
                int ret = op_and(ip, &reg_A, NULL, &word, NULL);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_ansq: {
                t_uint64 word;
                int ret = op_and(ip, &reg_Q, NULL, &word, NULL);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }

            case opcode0_ansx0:
            case opcode0_ansx1:
            case opcode0_ansx2:
            case opcode0_ansx3:
            case opcode0_ansx4:
            case opcode0_ansx5:
            case opcode0_ansx6:
            case opcode0_ansx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = ((getbits36(word, 0, 18) & reg_X[n]) << 18) | getbits36(word,18,18);
                    ret = store_word(TPR.CA, word);
                    if (ret == 0) {
                        IR.zero = (word >> 18) == 0;
                        IR.neg = bit36_is_neg(word);
                    }
                }
                return ret;
            }
            
            case opcode0_anx0:
            case opcode0_anx1:
            case opcode0_anx2:
            case opcode0_anx3:
            case opcode0_anx4:
            case opcode0_anx5:
            case opcode0_anx6:
            case opcode0_anx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_X[n] &= getbits36(word, 0, 18) & MASK18;
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }

            case opcode0_ora: { // OR to A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_A |= word;
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_oraq: {    // OR to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    reg_A |= word1;
                    reg_Q |= word2;
                    IR.zero = reg_A == 0 && reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_orq: { // OR to Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_Q |= word;
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
                }
                return ret;
            }
            case opcode0_orsa: {    // OR to storage A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word |= reg_A;
                    ret = store_word(TPR.CA, word);
                    if (ret == 0) {
                        IR.zero = word == 0;
                        IR.neg = bit36_is_neg(word);
                    }
                }
                return ret;
            }
            case opcode0_orsq: {    // OR to storage from Q reg
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word |= reg_Q;
                    ret = store_word(TPR.CA, word);
                    if (ret == 0) {
                        IR.zero = word == 0;
                        IR.neg = bit36_is_neg(word);
                    }
                }
                return ret;
            }
            case opcode0_orsx0:     // OR to Storage X[n]
            case opcode0_orsx1:
            case opcode0_orsx2:
            case opcode0_orsx3:
            case opcode0_orsx4:
            case opcode0_orsx5:
            case opcode0_orsx6:
            case opcode0_orsx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = ((getbits36(word, 0, 18) | reg_X[n]) << 18) | getbits36(word,18,18);
                    ret = store_word(TPR.CA, word);
                    if (ret == 0) {
                        IR.zero = (word >> 18) == 0;
                        IR.neg = bit36_is_neg(word);
                    }
                }
                return ret;
            }
            case opcode0_orx0:      // Or to Index Register N
            case opcode0_orx1: 
            case opcode0_orx2: 
            case opcode0_orx3: 
            case opcode0_orx4: 
            case opcode0_orx5: 
            case opcode0_orx6: 
            case opcode0_orx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_X[n] |= getbits36(word, 0, 18);
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }

            case opcode0_era: { // XOR to A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_A ^= word;
                    reg_A &= MASK36;
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_eraq: {    // XOR to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0) {
                    reg_A ^= word1;
                    reg_Q ^= word2;
                    IR.zero = word1 == 0 && word2 == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_erq: { // XOR to Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_Q ^= word;
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
                }
                return ret;
            }
            case opcode0_ersa: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word ^= reg_A;
                    ret = store_word(TPR.CA, word);
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }
            case opcode0_ersq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word ^= reg_Q;
                    ret = store_word(TPR.CA, word);
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }
            case opcode0_ersx0:     // XOR to Index Register N
            case opcode0_ersx1:
            case opcode0_ersx2:
            case opcode0_ersx3:
            case opcode0_ersx4:
            case opcode0_ersx5:
            case opcode0_ersx6:
            case opcode0_ersx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    uint32 upper = getbits36(word, 0, 18) ^ reg_X[n];
                    word = ((t_uint64) upper << 18) | (word & MASK18);
                    ret = store_word(TPR.CA, word);
                    IR.zero = upper == 0;
                    IR.neg = bit36_is_neg(word);
                    //log_msg(NOTIFY_MSG, "OPU::opcode::ersx*", "AL39 requires us to set IR.neg oddly.\n", word);
                    // cancel_run(STOP_WARN);
                }
                return ret;
            }

            case opcode0_erx0:
            case opcode0_erx1:
            case opcode0_erx2:
            case opcode0_erx3:
            case opcode0_erx4:
            case opcode0_erx5:
            case opcode0_erx6:
            case opcode0_erx7: {
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_X[n] ^= getbits36(word, 0, 18);
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }

            case opcode0_cana: {    // Comparative AND with A reg
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word &= reg_A;      // results discarded except for IR bits
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }
            // opcode0_canaq unimplemented
            case opcode0_canq: {    // Comparative AND with Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word &= reg_Q;      // results discarded except for IR bits
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }
            case opcode0_canx0:     // Comparative AND with Index Register N
            case opcode0_canx1:
            case opcode0_canx2:
            case opcode0_canx3:
            case opcode0_canx4:
            case opcode0_canx5:
            case opcode0_canx6:
            case opcode0_canx7: {
                int n = op & 7;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word &= (reg_X[n] << 18);       // results discarded except for IR bits
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }

            // cnaa unimplemented
            // cnaaq unimplemented
            // cnaq unimplemented
            // cnax0 .. cnax7 unimplemented

            // dfld unimplemented

            case opcode0_fld: {
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_uint64 lo = getbits36(word, 0, 8);
                    t_uint64 hi = getbits36(word, 8, 28);
                    reg_E = lo;
                    reg_A = 0;  // zero bits 28..35
                    reg_A = setbits36(reg_A, 0, 28, hi);
                    reg_Q = 0;  // bits 36..71 of AQ pair
                    IR.zero = reg_A == 0 && reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }

            case opcode0_dfst: {    // Double-Precision Floating Store
                t_uint64 word1 = setbits36(0, 0, 8, reg_E);
                word1 = setbits36(word1, 8, 28, getbits36(reg_A, 0, 28));
                t_uint64 word2 = setbits36(0, 0, 8, getbits36(reg_A, 28, 8));
                word2 = setbits36(word2, 8, 28, getbits36(reg_A, 0, 28));
                return store_pair(TPR.CA, word1, word2);
            }

            // dfstr unimplemented

            case opcode0_fst: { // Floating Store
                t_uint64 word = setbits36(0, 0, 8, reg_E);
                word = setbits36(word, 8, 28, getbits36(reg_A, 0, 28));
                return store_word(TPR.CA, word);
            }

            // fstr unimplemented
            // dfad unimplemented
            // dufa unimplemented

            case opcode0_fad: {         // floating add
                int ret = op_ufa(ip, 0);
                if (ret == 0)
                    ret = instr_fno();
                return ret;
            }

            case opcode0_ufa:           // unnormalized floating add
                return op_ufa(ip, 0);


            // dfsb unimplemented -- double precision floating subtract
            // dufs unimplemented -- double precision unnormalized floating subtract

            case opcode0_fsb: { // floating subtract
                int ret = op_ufa(ip, 1);
                if (ret == 0)
                    ret = instr_fno();
                return ret;
            }

            // ufs unimplemented -- unnormalized floating subtract
            // dfmp unimplemented -- double-precision floating multiply
            // dufmp unimplemented -- double-precision unnormalized floating multiply

            case opcode0_fmp: {     // floating multiply
                int ret = op_ufm(ip);
                if (ret == 0)
                    ret = instr_fno();
                return ret;
            }

            case opcode0_ufm:       // unnormalized floating multiply
                return op_ufm(ip);

            // dfdi unimplemented -- double precision floating divide inverted
            // dfdv unimplemented -- double precision floating divide
            // fdi unimplemented -- floating divide inverted

            case opcode0_fdv: {     // floating divide
                t_uint64 word;
                int ret;
                if (fetch_op(ip, &word) != 0)
                    return 1;
                uint8 op_exp = getbits36(word, 0, 8);
                t_uint64 op_mant = getbits36(word, 8, 28);  // 36-8=28 bits
                op_mant <<= 8;  // fractions are left aligned & we have 28 bits
                t_uint64 reg_a = getbits36(reg_A, 0, 28);
                reg_a <<= 8;    // fractions are left aligned & we have 28 bits
                log_msg(INFO_MSG, "OPU::fdv", "A =  %012llo => mantissa %012llo, E = %d\n", reg_A, reg_a, reg_E);
                log_msg(INFO_MSG, "OPU::fdv", "op:  %12s    mantissa %012llo, exp %d\n", "", op_mant, op_exp);
                double op_val = multics_to_double(op_mant, 0, 0, 1);
                double a_val = multics_to_double(reg_a, 0, 0, 1);
                log_msg(INFO_MSG, "OPU::fdv", "Dividing %.4f*2^%d by %.4f*2^%d\n", a_val, reg_E, op_val, op_exp);
                log_msg(INFO_MSG, "OPU::fdv", "E.g., dividing %.6f / %.6f\n",
                    a_val * (1 << reg_E), op_val * (1 << op_exp));
                if (a_val * (1 << reg_E) < op_val * (1 << op_exp)) {
                    log_msg(WARN_MSG, "OPU::fdiv", "Auto breakpoint for small dividend.\n");
                    cancel_run(STOP_IBKPT);
                }
                reg_Q = 0;
                int a_is_neg = bit36_is_neg(reg_a);
                int op_is_neg = bit36_is_neg(op_mant);
                int exp = (int8) reg_E;
                if (op_mant == 0) {
                    // AL39 says to check after aligment, but the
                    // alignment loop would never end for zero divsior
                    reg_A = getbits36(reg_A, 0, 28);
                    reg_A <<= 8;    // Put the MSB bits back into place
                    IR.zero = op_mant == 0; // BUG, makes no sense
                    IR.neg = bit36_is_neg(reg_A);
                    if (IR.neg)
                        reg_A = negate36(reg_A);
                    fault_gen(div_fault);
                    return 1;
                } else {
                    // treat fractions as sign-magnitude for shifting
                    if (op_is_neg)
                        op_mant = negate36(op_mant);
                    if (a_is_neg)
                        reg_a = negate36(reg_a);
                    // Align mantissas as per AL39
                    for (;;) {
                        if (reg_a < op_mant)
                            break;
                        reg_a >>= 1;
                        ++ exp;
                    }
                    // Restore signs
                    if (op_is_neg)
                        op_mant = negate36(op_mant);
                    if (a_is_neg)
                        reg_a = negate36(reg_a);
                    a_val = multics_to_double(reg_a, 0, 0, 1);
                    log_msg(INFO_MSG, "OPU::fdv", "Aligned divisor mantissa is %012llo\n", reg_a);
                    log_msg(INFO_MSG, "OPU::fdv", "Aligned divisor is %.4f*2^%d aka %.6f\n", a_val, exp, a_val * (1 << exp));

                    exp -= op_exp;
                    if (exp < -128)
                        IR.exp_underflow = 1;
                    else if (exp > 127)
                        IR.exp_overflow = 1;
                    reg_E = exp;

                    // 72bit right shift of a in order to scale
                    t_uint64 low = setbits36(0, 0, 1, reg_a & 1);
                    reg_a >>= 1;
                    t_uint64 quot, rem;
                    div72(reg_a, low, op_mant, &quot, &rem);
                    reg_A = quot;
                    a_val = multics_to_double(reg_A, 0, 0, 1);
                    log_msg(INFO_MSG, "OPU::fdv", "Result is %.4f*2^%d aka %.6f\n", a_val, reg_E, a_val * (1 << reg_E));
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                if (op_is_neg || a_is_neg) {
                    log_msg(WARN_MSG, "OPU::fdiv", "Auto breakpoint for negative for negatives.\n");
                    cancel_run(STOP_IBKPT);
                }
                return 0;
            }

            // fneg unimplemented -- floating negate

            case opcode0_fno:       // floating normalize
                return instr_fno();

            // dfrd unimplemented -- double precision floating round
            // frd unimplemented -- floating round
            // dfcmg unimplemented -- double precision floating compare magnitude

            case opcode0_dfcmp: {       // double precision floating compare
                t_uint64 word0, word1;
                int ret;
                if (fetch_pair(TPR.CA, &word0, &word1) != 0)
                    return 1;
                if (opt_debug) {
                    log_msg(DEBUG_MSG, "OPU::dfcmp", "AQ = { %012llo, %012llo }, exp %d\n", reg_A, reg_Q, reg_E);
                    log_msg(DEBUG_MSG, "OPU::dfcmp", "op = { %012llo, %012llo }\n", word0, word1);
                }
                uint8 op_exp = getbits36(word0, 0, 8);
                t_uint64 op_mant_hi = getbits36(word0, 8, 28);  // 36-8=28 bits
                op_mant_hi <<= 8;
                op_mant_hi |= getbits36(word1, 0, 8);
                t_uint64 op_mant_lo = getbits36(word1, 8, 28) << 8;
                if (opt_debug)
                    log_msg(DEBUG_MSG, "OPU::dfcmp", "op:  { %012llo, %012llo }, exp %d\n", op_mant_hi, op_mant_lo, op_exp);
                t_uint64 reg_a = reg_A;
                t_uint64 reg_q = getbits36(reg_Q, 0, 28) << 8;  // need 64bit version of aq

                double op_val = multics_to_double(op_mant_hi, op_mant_lo, 0, 1);
                double aq_val72 = multics_to_double(reg_A, reg_Q, 0, 1);
                double aq_val64 = multics_to_double(reg_a, reg_q, 0, 1);
                log_msg(INFO_MSG, "OPU::dfcmp", "Comparing %.4f*2^%d to %.4f*2^%d\n", aq_val64, reg_E, op_val, op_exp);

#if 1
                int exp;    // debug
                int op_neg = bit36_is_neg(op_mant_hi);
                int aq_neg = bit36_is_neg(reg_a);
                equalize_mantissas72("OPU::dfcmp", &reg_a, &reg_q, reg_E, &op_mant_hi, &op_mant_lo, op_exp, &exp);
#else

                // treat fractions as sign-magnitude for shifting
                int op_neg = bit36_is_neg(op_mant_hi);
                if (op_neg)
                    negate72(&op_mant_hi, &op_mant_lo);
                int aq_neg = bit36_is_neg(reg_a);
                if (aq_neg)
                    negate72(&reg_a, &reg_q);
            
                // 72-bit right shift
                int exp;    // debug
                if ((int8) op_exp < (int8) reg_E) {
                    // operand has the smaller exponent
                    exp = (int8) reg_E;
                    int n = (int8) reg_E - (int8) op_exp;
                    long_right_shift(&op_mant_hi, &op_mant_lo, n, 1);
                } else {
                    // AQ has the smaller exponent
                    exp = (int8) op_exp;
                    int n = (int8) op_exp - (int8) reg_E;
                    long_right_shift(&reg_a, &reg_q, n, 1);
                }
#endif
                if (op_mant_hi == 0 && op_mant_lo == 0)
                    op_neg = 0;
                if (reg_a == 0 && reg_q == 0)
                    aq_neg = 0;
                IR.zero = aq_neg == op_neg && reg_a == op_mant_hi && reg_q == op_mant_lo;
                if (IR.zero)
                    IR.neg = 0;
                else if (aq_neg && ! op_neg)
                    IR.neg = 1;
                else if (!aq_neg && op_neg)
                    IR.neg = 0;
                else if (!aq_neg && ! op_neg)
                    IR.neg = reg_a < op_mant_hi || (reg_a == op_mant_hi && reg_q < op_mant_lo);
                else
                    IR.neg = reg_a > op_mant_hi || (reg_a == op_mant_hi && reg_q > op_mant_lo);

                op_val = multics_to_double(op_mant_hi, op_mant_lo, 0, 1);
                aq_val64 = multics_to_double(reg_a, reg_q, 0, 1);
                if (IR.zero) {
                    log_msg(NOTIFY_MSG, "OPU::dfcmp", "Result: equal -- exp is %d; mantissa %f versus %f\n", exp, aq_val64, op_val);
                    log_msg(NOTIFY_MSG, "OPU::dfcmp", "Auto breakpoint\n");
                    cancel_run(STOP_IBKPT);
                } else
                    log_msg(INFO_MSG, "OPU::dfcmp", "Result: Neg=%d -- exp is %d; mantissa %f versus %f\n", IR.neg, exp, aq_val64, op_val);
                return 0;
            }
            
            // fcmg unimplemented -- floating compare magnitude

            case opcode0_fcmp: {        // floating compare
                t_uint64 word;
                int ret;
                if (fetch_op(ip, &word) != 0)
                    return 1;
                uint8 op_exp = getbits36(word, 0, 8);
                t_uint64 op_mant = getbits36(word, 8, 28);  // 36-8=28 bits
                t_uint64 reg_a = getbits36(reg_A, 0, 28);
                log_msg(INFO_MSG, "OPU::fcmp", "A =  %012llo => mantissa %012llo, E = %d\n", reg_A, reg_a, reg_E);
                log_msg(INFO_MSG, "OPU::fcmp", "op:  %12s    mantissa %012llo, exp %d\n", "", op_mant, op_exp);
                double op_val = multics_to_double(op_mant << 8, 0, 0, 1);
                double a_val = multics_to_double(reg_a << 8, 0, 0, 1);
                log_msg(INFO_MSG, "OPU::fcmp", "Comparing %.4f*2^%d to %.4f*2^%d\n", a_val, reg_E, op_val, op_exp);
#if 1
                int exp;    // debug
                int op_neg = bit36_is_neg(op_mant);
                int a_neg = bit36_is_neg(reg_a);
                equalize_mantissas("OPU::fcmp", &reg_a, reg_E, &op_mant, op_exp, &exp);
#else

                // treat fractions as sign-magnitude for shifting
                int op_neg = bit36_is_neg(op_mant);
                if (op_neg)
                    op_mant = negate36(op_mant);
                int a_neg = bit36_is_neg(reg_a);
                if (a_neg)
                    reg_a = negate36(reg_a);
            
                // right shift
                int exp;    // debug
                if ((int8) op_exp < (int8) reg_E) {
                    // operand has the smaller exponent
                    exp = (int8) reg_E;
                    int n = (int8) reg_E - (int8) op_exp;
                    if (n >= 36)
                        op_mant = 0;
                    else
                        op_mant >>= n;
                } else {
                    // AQ has the smaller exponent
                    exp = (int8) op_exp;
                    int n = (int8) op_exp - (int8) reg_E;
                    if (n >= 36)
                        reg_a = 0;
                    else
                        reg_a >>= n;
                }
#endif
                if (op_mant == 0)
                    op_neg = 0;
                if (reg_a == 0)
                    a_neg = 0;
                IR.zero = a_neg == op_neg && reg_a == op_mant;
                if (IR.zero)
                    IR.neg = 0;
                else if (a_neg && ! op_neg)
                    IR.neg = 1;
                else if (!a_neg && op_neg)
                    IR.neg = 0;
                else if (!a_neg && ! op_neg)
                    IR.neg = reg_a < op_mant;
                else
                    IR.neg = reg_a > op_mant;

                op_val = multics_to_double(op_mant << 8, 0, 0, 1);
                a_val = multics_to_double(reg_a << 8, 0, 0, 1);
                if (IR.zero) {
                    log_msg(NOTIFY_MSG, "OPU::fcmp", "Result: equal -- exp is %d; mantissa %f versus %f\n", exp, a_val, op_val);
                    log_msg(NOTIFY_MSG, "OPU::fcmp", "Auto breakpoint\n");
                    cancel_run(STOP_IBKPT);
                } else
                    log_msg(INFO_MSG, "OPU::fcmp", "Result: Neg=%d -- exp is %d; mantissa %f versus %f\n", IR.neg, exp, a_val, op_val);
                log_msg(NOTIFY_MSG, "OPU::fcmp", "Untested!\n");
                return 0;
            }
            
            case opcode0_ade: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    int y = getbits36(word, 0, 8);
                    IR.zero = 0;
                    IR.neg = 0;
                    if (reg_E + y > 127)
                        IR.exp_overflow = 1;
                    else if (reg_E +y < -128)
                        IR.exp_underflow = 1;
                    reg_E = (reg_E + y) & 0xff;
                }
                return ret;
            }

            case opcode0_fszn: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    IR.zero = getbits36(word, 8, 28) == 0;
                    IR.neg = getbits36(word, 8, 1) == 1;
                }
                return ret;
            }

            case opcode0_lde: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    reg_E = getbits36(word, 0, 8);
                    IR.zero = 0;
                    IR.neg = 0;
                }
                return ret;
            }

            case opcode0_ste: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    word = setbits36(word, 0, 18, reg_E << 10);
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            case opcode0_call6: {
                if (get_addr_mode() == ABSOLUTE_mode) { 
                    log_msg(ERR_MSG, "OPU::call6", "Absolute mode not handled\n");
                    cancel_run(STOP_WARN);
                }
                if (TPR.TRR < PPR.PRR) {
                    AR_PR[7].PR.snr = (cpup->DSBR.stack << 3) | TPR.TRR;    // 12 bit stack, 3 bit ring number
                    log_msg(NOTIFY_MSG, "OPU::call6", "Inward call\n");
                    log_msg(NOTIFY_MSG, "OPU::call6", "PR[7] segment set to %04o||%o==> %05o\n", cpup->DSBR.stack, TPR.TRR, AR_PR[7].PR.snr);
                    log_msg(NOTIFY_MSG, "OPU::call6", "Auto breakpoint\n");
                    cancel_run(STOP_IBKPT);
                } else if (TPR.TRR == PPR.PRR)
                    AR_PR[7].PR.snr = AR_PR[6].PR.snr;
                else {
                    // Outward calls are declared illegal in Multics
                    log_msg(WARN_MSG, "OPU::call6", "Outward call\n");
                    cu.word1flags.ocall = 1;
                    fault_gen(acc_viol_fault);
                    return 1;
                }

                AR_PR[7].PR.rnr = TPR.TRR;
                if (TPR.TRR == 0) {
                    // BUG: Need to compare our mechanism for possibly paging in the SDW with chapter 8 of AL39
                    PPR.P = is_priv_mode();     // Get priv bit from the SDW for TPR.TSR
                } else
                    PPR.P = 0;
                AR_PR[7].wordno = 0;
                AR_PR[7].PR.bitno = 0;
                AR_PR[7].AR.charno = 0;
                AR_PR[7].AR.bitno = 0;

                PPR.PRR = TPR.TRR;
                PPR.PSR = TPR.TSR;
                PPR.IC = TPR.CA;
                cpu.trgo = 1;
                // log_msg(NOTIFY_MSG, "OPU::call6", "Auto breakpoint\n");
                // cancel_run(STOP_IBKPT);
                return 0;
            }
            
            case opcode0_ret: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    do_ldi_ret(word, 1);
                    PPR.IC = word >> 18;
                    cpu.trgo = 1;
                }
                return ret;
            }

            case opcode0_rtcd: {
                // BUG -- Need to modify APU to check for opcode==rtcd && POA flag
                log_msg(INFO_MSG, "OPU::rtcd", "Some access checks are not implemented.\n");    // TODO

                if (get_addr_mode() == ABSOLUTE_mode) { 
                    log_msg(ERR_MSG, "OPU::rtcd", "Absolute mode not handled\n");
                    cancel_run(STOP_WARN);
                }

                t_uint64 word0, word1;
                if (fetch_pair(TPR.CA, &word0, &word1) != 0) {
                    log_msg(WARN_MSG, "OPU::rtcd", "fetch y-pair: error or fault\n");
                    return 1;   // faulted
                }

                SDW_t *SDWp = get_sdw();        // Get SDW for TPR.TSR

                int priv;
                if (PPR.PRR == 0)
                    priv = is_priv_mode();      // Get priv bit from the SDW for TPR.TSR
                else
                    priv = 0;

                // AL39 says to check for fault while getting SDW -- however if a fault had occured, we
                // would have failed to fetch the y-pair.  Thus, we'd be putting garbage in the IC.
                if (! SDWp || fault_check_group(6)) {
                    log_msg(WARN_MSG, "OPU::rtcd", "Failed to get SDW and/or a fault occured even though operand fetch was successful.\n");
                    cancel_run(STOP_BUG);
                    return 1;
                }

                PPR.PSR = getbits36(word0, 3, 15);
                PPR.PRR = max3(getbits36(word0, 18, 3), TPR.TRR, SDWp->r1);
                PPR.IC = word1 >> 18;
                PPR.P = priv;
                if (priv == 0)
                    log_msg(DEBUG_MSG, "OPU::rtcd", "Flagging PPR with priv=0.\n");
                for (int i = 0; i < 8; ++i) {
                    AR_PR[i].PR.rnr = PPR.PRR;
                }

                //log_msg(NOTIFY_MSG, "OPU::rtcd", "Auto breakpoint\n");
                //cancel_run(STOP_IBKPT);
                cpu.trgo = 1;
                return 0;
            }
            
            // opcode0_teo unimplemented
            // opcode0_teu unimplemented
            
            case opcode0_tmi:
                if (IR.neg) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            // tmoz -- see opcode1_tmoz
            //
            case opcode0_tnc:
                if (! IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            case opcode0_tnz:
                if (! IR.zero) {
                    log_msg(DEBUG_MSG, "OPU::opcode::tnz", "transfer to %#o\n", TPR.CA);
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                } else
                    log_msg(DEBUG_MSG, "OPU::opcode::tnz", "no transfer (would have been to %#o)\n", TPR.CA);
                return 0;
            case opcode0_tov:
                if (IR.overflow) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                IR.overflow = 0;
                return 0;
            case opcode0_tpl:
                if (! IR.neg) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            // tpnz -- see opcode1_tpnz
            case opcode0_tra:
                PPR.IC = TPR.CA;
                PPR.PSR = TPR.TSR;
                cpu.trgo = 1;
                return 0;
            case opcode0_trc:
                if (IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            // trtf -- see opcode1_trtf
            // trtn -- see opcode1_trtn

            case opcode0_tsp0:
            case opcode0_tsp1:
            case opcode0_tsp2:
            case opcode0_tsp3:
            case opcode0_tsp4:
            case opcode0_tsp5:
            case opcode0_tsp6:
            case opcode0_tsp7: {
                int n = (op & 7) + ((op >> 6) & 4);
                AR_PR[n].PR.rnr = PPR.PRR;
                AR_PR[n].PR.snr = PPR.PSR;
                AR_PR[n].wordno = (PPR.IC + 1) & MASK18;
                AR_PR[n].PR.bitno = 0;
                AR_PR[n].AR.charno = 0;
                AR_PR[n].AR.bitno = 0;
                PPR.IC = TPR.CA;
                PPR.PSR = TPR.TSR;
                cpu.trgo = 1;
                return 0;
            }
            

            case opcode0_tss: {
                if (TPR.CA >= (BAR.bound << 9)) {
                    log_msg(NOTIFY_MSG, "OPU::tss", "Address %#o is out of BAR bounds of %#o.\n", TPR.CA, BAR.bound << 9);
                    fault_gen(store_fault);
                    // fault_reg.oob = 1;           // ERROR: fault_reg does not exist
                    return 1;
                }
                if (get_addr_mode() == BAR_mode) {
                    PPR.PSR = TPR.TSR;
                    PPR.IC = TPR.CA;
                } else {
                    log_msg(NOTIFY_MSG, "OPU::tss", "Switching to BAR mode.\n");
                    // PPR.IC = TPR.CA + (BAR.base << 9);   // Despite AL39, we need to let the CPU/APU add in the base
                    PPR.PSR = TPR.TSR;
                    PPR.IC = TPR.CA;
                    set_addr_mode(BAR_mode);
                }
                cpu.trgo = 1;
                return 0;
            }

            case opcode0_tsx0:  // transfer and set X[n]
            case opcode0_tsx1:
            case opcode0_tsx2:
            case opcode0_tsx3:
            case opcode0_tsx4:
            case opcode0_tsx5:
            case opcode0_tsx6:
            case opcode0_tsx7: {
                int n = op & 07;
                reg_X[n] = PPR.IC + 1;
                PPR.IC = TPR.CA;
                PPR.PSR = TPR.TSR;
                cpu.trgo = 1;
                return 0;
            }

            case opcode0_ttf:
                if (! IR.tally_runout) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;

            case opcode0_tze:
                if (IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;

            case opcode0_easp0:
                return do_easp(0);
            case opcode0_easp2:
                return do_easp(2);
            case opcode0_easp4:
                return do_easp(4);
            case opcode0_easp6:
                return do_easp(6);

            case opcode0_eawp0:
                return do_eawp(0);
            case opcode0_eawp2:
                return do_eawp(2);
            case opcode0_eawp4:
                return do_eawp(4);
            case opcode0_eawp6:
                return do_eawp(6);

            case opcode0_epbp1:
                return do_epbp(1);
            case opcode0_epbp3:
                return do_epbp(3);
            case opcode0_epbp5:
                return do_epbp(5);
            case opcode0_epbp7:
                return do_epbp(7);

            case opcode0_epp0:
                return do_epp(0);
            case opcode0_epp2:
                return do_epp(2);
            case opcode0_epp4:
                return do_epp(4);
            case opcode0_epp6:
                return do_epp(6);

            case opcode0_lpri: {
                addr_modes_t addr_mode;
                if ((addr_mode = get_addr_mode()) == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                t_uint64 words[16];
                int ret;
                if ((ret = fetch_yblock(TPR.CA, 1, 16, words)) == 0) {
                    SDW_t* sdwp = NULL;
                    if (addr_mode != ABSOLUTE_mode)
                        if ((sdwp = get_sdw()) == NULL) // Get SDW for current TPR.TSR
                            return 1;
                    for (int i = 0; i < 8; ++i) {
                        uint y_rnr = getbits36(words[2*i], 18, 3);
                        if (addr_mode == ABSOLUTE_mode)
                            AR_PR[i].PR.rnr = y_rnr;
                        else
                            AR_PR[i].PR.rnr = max3(y_rnr, sdwp->r1, TPR.TRR);
                        AR_PR[i].PR.snr = getbits36(words[2*i], 3, 15);
                        AR_PR[i].wordno = getbits36(words[2*i+1], 0, 18);
                        AR_PR[i].PR.bitno = getbits36(words[2*i+1], 21, 6); // 36-(72-57)
                        if ((int) AR_PR[i].PR.bitno < 0 || AR_PR[i].PR.bitno  > 35) {
                            log_msg(ERR_MSG, "OPU::lpri", "PR%d now has a bitno of %d\n", i, AR_PR[i].PR.bitno );
                            cancel_run(STOP_BUG);
                        }
                        AR_PR[i].AR.charno = AR_PR[i].PR.bitno / 9;
                        AR_PR[i].AR.bitno = AR_PR[i].PR.bitno % 9;
                        log_msg(DEBUG_MSG, "OPU::lpri", "PR[%d]: rnr=%o, snr=%o, wordno=%0o, bitno=%#o\n",
                            i, AR_PR[i].PR.rnr, AR_PR[i].PR.snr, AR_PR[i].wordno, AR_PR[i].PR.bitno);
                    }
                }
                return ret;
            }

            case opcode0_lprp0:
            case opcode0_lprp1:
            case opcode0_lprp2:
            case opcode0_lprp3:
            case opcode0_lprp4:
            case opcode0_lprp5:
            case opcode0_lprp6:
            case opcode0_lprp7: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int n = op & 07;
                AR_PR[n].PR.rnr = TPR.TRR;
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    if (getbits36(word, 0, 2) == 03) {
                        fault_gen(cmd_fault);
                        return 1;
                    }
                    AR_PR[n].PR.bitno = getbits36(word, 0, 6);
                    if ((int) AR_PR[n].PR.bitno < 0 || AR_PR[n].PR.bitno  > 35) {
                        log_msg(ERR_MSG, "OPU::lprp*", "PR%d now has a bitno of %d\n", n, AR_PR[n].PR.bitno );
                        cancel_run(STOP_BUG);
                    }
                    AR_PR[n].AR.charno = AR_PR[n].PR.bitno / 9;
                    AR_PR[n].AR.bitno = AR_PR[n].PR.bitno % 9;
                    if (getbits36(word, 6, 12) == 07777)
                        AR_PR[n].PR.snr = 070000;   // bits 0..2 of 15-bit register
                    else
                        AR_PR[n].PR.snr = 0;
                    AR_PR[n].PR.snr |= getbits36(word, 6, 12);
                    AR_PR[n].wordno = getbits36(word, 18, 18);
                    if (opt_debug) {
                        char cmd[20];
                        sprintf(cmd, "lprp%d", n);
                        log_msg(DEBUG_MSG, cmd, "PR[%d] loaded from value %012llo\n", n, word);
                    }
                }
                return ret;
            }

            case opcode0_spbp1:
                return do_spbp(1);
            case opcode0_spbp3:
                return do_spbp(3);
            case opcode0_spbp5:
                return do_spbp(5);
            case opcode0_spbp7:
                return do_spbp(7);

            case opcode0_spri: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                t_uint64 words[16];
                for (int i = 0; i < 8; ++i)
                    spri_to_words(i, words+2*i, words+2*i+1);
                return store_yblock16(TPR.CA, words);
            }

            case opcode0_spri0:
                return do_spri(0);
            case opcode0_spri2:
                return do_spri(2);
            case opcode0_spri4:
                return do_spri(4);
            case opcode0_spri6:
                return do_spri(6);

            case opcode0_sprp0:
            case opcode0_sprp1:
            case opcode0_sprp2:
            case opcode0_sprp3:
            case opcode0_sprp4:
            case opcode0_sprp5:
            case opcode0_sprp6:
            case opcode0_sprp7: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int n = op & 07;
                uint snr = AR_PR[n].PR.snr; // 15 bits (this instr only stores lower 12 bits)
                if ((snr & 070000) != 0 && snr != 077777) {
                    fault_gen(store_fault); // illegal pointer
                    return 1;
                }
                t_uint64 word = setbits36(0, 0, 6, AR_PR[n].PR.bitno);
                word = setbits36(word, 6, 12, (snr & 07777));
                word = setbits36(word, 18, 18, AR_PR[n].wordno);
                char cmd[20];
                sprintf(cmd, "OPU::sprp%d", n);
                log_msg(DEBUG_MSG, cmd, "Packed value %012llo from PR[%d] (snr=%o, wordno=%0o, bitno=%#o)\n",
                    word, n, AR_PR[n].PR.snr, AR_PR[n].wordno, AR_PR[n].PR.bitno);
                int ret = store_word(TPR.CA, word);
                return ret;
            }

            case opcode0_adwp0:
            case opcode0_adwp1:
            case opcode0_adwp2:
            case opcode0_adwp3:
            case opcode0_adwp4:
            case opcode0_adwp6:
            case opcode0_adwp7: {
                int n = (op & 03) + ((op >= opcode0_adwp4) ? 4 : 0);
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    AR_PR[n].wordno += word >> 18;
                    AR_PR[n].PR.bitno = 0;
                    AR_PR[n].AR.charno = 0;
                    AR_PR[n].AR.bitno = 0;
                }
                return ret;
            }

            case opcode0_epaq:
                reg_A = (TPR.TSR & MASKBITS(15)) << 18;
                reg_A |= TPR.TRR & MASKBITS(3);
                reg_Q = (TPR.CA & MASK18) << 18;
                reg_Q |= TPR.TBR & MASKBITS(6);
                return 0;

            case opcode0_rccl: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = scu_get_calendar(TPR.CA);
                // AL-39 was just trying to say that the clock is 52 bits, not that
                // we need to mask out the upper 20 bits of A.
                return ret;
            }

            // drl unimplemented

            case opcode0_xec: {
                // todo: fault if xec invokes xec
                // BUG: EIS multiword instructions may not be handled
                t_uint64 word0;
                instr_t IR;
                if (fetch_word(TPR.CA, &word0) != 0) {
                    log_msg(NOTIFY_MSG, "OPU::opcode::xec", "fetch instr: error or fault\n");
                    return 1;   // faulted
                }
#if 0
                // todo: combine with xed
                // todo: re-implement via setting flags and return to control_unit()
                // todo: handle rpd repeats
                decode_instr(&IR, word0);
                // extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "executing instr at %#o\n", TPR.CA);
                int ret;
                if ((ret = do_op(&IR)) != 0)
                    log_msg(NOTIFY_MSG, "OPU::opcode::xec", "fault or error executing instr\n");
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "finished\n");
                // -- opt_debug; -- cpu_dev.dctrl;
#else
                if (sim_brk_summ)
                    if (sim_brk_test(TPR.CA, SWMASK ('E'))) {
                        log_msg(NOTIFY_MSG, "opu::xec", "Breakpoint on target instruction pair at %#o\n", TPR.CA);
                        cancel_run(STOP_IBKPT);
                    }
                if (cpu.ic_odd) {
                    cu.xdo = 1;
                    cu.IRODD = word0;
                    cpu.irodd_invalid = 0;
                } else {
                    decode_instr(&cu.IR, word0);
                    cu.xde = 1;
                }
                log_msg(DEBUG_MSG, "opu::xec", "Flags are set for exec of instruction at %#o\n", TPR.CA);
#endif
                return 0;
            }

            case opcode0_xed: {
                // extern DEVICE cpu_dev; ++ opt_debug; if (! cpu_dev.dctrl) ++ cpu_dev.dctrl;
                // Load IR and IRODD, set flags, and return to the control unit
                // Much of the work here and in control_unit() is for handling edge cases like
                // xed running rpd or other edge cases but it's likely that real code always
                // uses a scu/tra pair.  Nope, see pl1_operators_$fx1_to_fl2.
                t_uint64 word0;
                t_uint64 word1;
                instr_t IR;
                uint y = TPR.CA - TPR.CA % 2;   // force even
                if (fetch_pair(y, &word0, &word1) != 0) {
                    log_msg(WARN_MSG, "OPU::opcode::xed", "fetch y-pair: error or fault\n");
                    return 1;   // faulted
                }
                decode_instr(&cu.IR, word0);
#if 0
                instr_t i_tmp;
                decode_instr(&i_tmp, word1);
                if (cu.IR.opcode != (opcode0_scu << 1) || i_tmp.opcode != (opcode0_tra << 1)) {
                    log_msg(WARN_MSG, "OPU::xed", "Target y-pair at %#o is not SCU/TRA -- found opcodes %03o(%d) and %03o(%d)\n", y, cu.IR.opcode >> 1, cu.IR.opcode & 1, i_tmp.opcode >> 1, i_tmp.opcode & 1);
                }
#endif
                cpu.ic_odd = 0;
                cu.IRODD = word1;
                cpu.irodd_invalid = 0;
                // cu.repeat_first = 1;     // allows us to execute the instruction even during a fault cycle; BUG: this is a hack
                cu.xde = 1;
                cu.xdo = 1;
                log_msg(NOTIFY_MSG, "opu::xed", "Flags are set for exec of ypair at %#o\n", y);
                if (sim_brk_summ)
                    if (sim_brk_test(y, SWMASK ('E')) || sim_brk_test(y+1, SWMASK ('E'))) {
                        log_msg(NOTIFY_MSG, "opu::xed", "Breakpoint on target instruction pair at %#o\n", y);
                        cancel_run(STOP_IBKPT);
                    }
#if 0
                if (switches.FLT_BASE == 2) {   // true for multics, false for T&D diag tape
                    log_msg(NOTIFY_MSG, "opu::xed", "Auto Breakpoint\n");
                    cancel_run(STOP_IBKPT);
                }
#endif
                return 0;
            }

            case opcode0_mme:
                fault_gen(mme1_fault);
                return 1;

            case opcode0_drl:
                fault_gen(drl_fault);
                return 1;

            case opcode0_mme2:
                if (get_addr_mode() == BAR_mode)
                    fault_gen(illproc_fault);
                else
                    fault_gen(mme2_fault);
                return 1;

            case opcode0_mme3:
                if (get_addr_mode() == BAR_mode)
                    fault_gen(illproc_fault);
                else
                    fault_gen(mme3_fault);
                return 1;

            case opcode0_mme4:
                if (get_addr_mode() == BAR_mode)
                    fault_gen(illproc_fault);
                else
                    fault_gen(mme4_fault);
                return 1;

            case opcode0_nop:
            case opcode0_puls1:
            case opcode0_puls2:
                // BUG: certain address forms may generate faults -- so addr_mod should gen faults
                return 0;

            // rpd unimplemented -- repeat double
            // rpl unimplemented -- repeat link
            
            case opcode0_rpt: {
                extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
                cu.rpts = 1;
                uint tally = (ip->addr >> 10);
                uint c = (ip->addr >> 7) & 1;
                uint term = ip->addr & 0177;
                cu.delta = ip->mods.single.tag;
                if (c)
                    reg_X[0] = ip->addr;    // Entire 18 bits
                cu.rpt = 1;
                cu.repeat_first = 1;
                // Setting cu.rpt will cause the instruction to be executed
                // until the termination is met.
                // See cpu.c for the rest of the handling.
                log_msg(INFO_MSG, "OPU", "RPT instruction found\n");
                return 0;
            }

            case opcode0_rpd: {
                extern DEVICE cpu_dev; ++ opt_debug; if (! cpu_dev.dctrl) ++ cpu_dev.dctrl;
                if (PPR.IC % 2 == 0) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                cu.rpts = 1;
                uint tally = (ip->addr >> 10);
                uint c = (ip->addr >> 7) & 1;
                uint term = ip->addr & 0177;
                cu.delta = ip->mods.single.tag;
                if (c)
                    reg_X[0] = ip->addr;    // Entire 18 bits
                cu.rd = 1;
                cu.repeat_first = 1;
                // Setting cu.rpts and cu.rd will cause the instruction to be executed
                // until the termination is met.
                // See cpu.c for the rest of the handling.
                log_msg(NOTIFY_MSG, "OPU", "RPD instruction found\n");
                cancel_run(STOP_IBKPT);
                return 0;
            }

            case opcode0_sbar: {
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                if (ret == 0) {
                    word = setbits36(word, 0, 9, BAR.base);
                    word = setbits36(word, 9, 9, BAR.bound);
                    ret = store_word(TPR.CA, word);
                }
                return ret;
            }

            // bcd unimplemented -- binary to binary-coded-decimal
            // gtb unimplemented -- gray to binary

            case opcode0_lbar: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    BAR.base = getbits36(word, 0, 9);
                    BAR.bound = getbits36(word, 9, 9);
                    log_msg(INFO_MSG, "OPU::lbar", "BAR: base = %09o => %018o, bound = %09o => %018o\n",
                        BAR.base, BAR.base << 9, BAR.bound, BAR.bound << 9);
                }
                return ret;
            }

            case opcode0_lcpr: {    // load central processor reg (priv)
                int ret = 0;
                t_uint64 word;
                switch (ip->mods.single.tag) {      // no addr modifications
                    case 2:
                        // Note that we completely ignore the CMR other
                        // loading and storing it -- we don't have nor
                        // need a cache.
                        ret = fetch_word(TPR.CA, &word);
                        if (! ret) {
                            CMR = word;
                            CMR = setbits36(CMR, 36-36, 15, 0);
                            CMR = setbits36(CMR, 51-36, 3, 0);
                            CMR = setbits36(CMR, 58-36, 1, 0);
                            CMR = setbits36(CMR, 60-36, 4, 0);
                        }
                        break;
                    case 4:
                        ret = fetch_word(TPR.CA, &word);
                        if (! ret) {
                            MR.word = word;
                            MR.mr_enable = word & 1; word &= ~1;
                            MR.strobe = word & 020; // word &= ~020;
                            MR.fault_reset = word &040; word &= ~040;
                            word &= MASK36;
                            if (word != 0) {
                                ret = 1;
                                log_msg(ERR_MSG, "OPU::opcode::lcpr", "Wrote %#llo to mode reg, but mode reg flags are ignored.\n", MR.word);
                            }
                        }
                        break;
                    case 3:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "history reg zero unimplemented.\n");
                        ret = 1;
                        break;
                    case 7:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "history reg setting unimplemented.\n");
                        ret = 1;
                        break;
                    default:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Bad tag %#o\n", ip->mods.single.tag);
                        ret = 1;
                }
                if (ret)
                    cancel_run(STOP_WARN);
                return ret;
            }
            case opcode0_ldbr: {
                if (get_addr_mode() != ABSOLUTE_mode) {
                    log_msg(WARN_MSG, "OPU::ldbr", "ldbr when not in absolute mode.\n");
                    fault_gen(illproc_fault);   // BUG: which fault?
                    return 1;
                }
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (! cu.PT_ON) {
                    log_msg(WARN_MSG, "OPU::ldbr", "PTWAM is not enabled\n");
                    // cancel_run(STOP_WARN);
                }
                if (! cu.SD_ON) {
                    log_msg(WARN_MSG, "OPU::ldbr", "SDWAM is not enabled\n");
                    // cancel_run(STOP_WARN);
                }
                for (int i = 0; i < 16; ++i) {
                    if (cu.SD_ON) {
                        cpup->SDWAM[i].assoc.is_full = 0;
                        cpup->SDWAM[i].assoc.use = i;
                    }
                    if (cu.PT_ON) {
                        cpup->PTWAM[i].assoc.is_full = 0;
                        cpup->PTWAM[i].assoc.use = i;
                    }
                }
                // BUG: If cache is enabled, reset all cache colume and level full flags
                cpup->DSBR.addr = getbits36(word1, 0, 24);
                cpup->DSBR.bound = getbits36(word2, 37-36, 14);
                cpup->DSBR.u = getbits36(word2, 55-36, 1);
                cpup->DSBR.stack = getbits36(word2, 60-36, 12);
                log_msg(INFO_MSG, "OPU::ldbr", "DSBR: addr=%#o, bound=%#o(%u), u=%d, stack=%#o\n",
                    cpup->DSBR.addr, cpup->DSBR.bound, cpup->DSBR.bound, cpup->DSBR.u, cpup->DSBR.stack);
                return 0;
            }
            case opcode0_ldt: { // load timer reg (priv)
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    log_msg(WARN_MSG, "OPU::ldt", "Privileged\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_uint64 bits = getbits36(word, 0, 27);
                    log_msg(DEBUG_MSG, "OPU::opcode::ldt", "Operand is %#llo => %#llo\n", word, bits);
                    reg_TR = bits;
                    activate_timer();
                }
                return ret;
            }
            // lptp unimplemented -- T&D Test & Diagnostic instruction
            // lptr unimplemented -- T&D Instruction
            // lra unimplemented
            // lsdp unimplemented -- T&D Instruction
            // lsdr unimplemented -- T&D Instruction

            case opcode0_rcu: {
                int ret;
                t_uint64 words[8];
                if ((ret = fetch_yblock8(TPR.CA, words)) != 0)
                    return ret;
                words2scu(words);
                log_msg(ERR_MSG, "OPU::rcu", "Partial implementation: Cannot fully restore control unit\n");
                cancel_run(STOP_BUG);
                return 1;
            }

            case opcode0_scpr: {    // Store CU reg
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                t_uint64 words[2];
                int ret = 1;
                int unimp = 1;
                switch (ip->mods.single.tag) {
                    case 0:
                        log_msg(WARN_MSG, "OPU::scpr", "APU history register not implemented\n");
                        cancel_run(STOP_BUG);
                        break;
                    case 1:
                        unimp = 0;
                        words[0] = FR;
                        words[1] = 0;
                        FR = 0;
                        log_msg(WARN_MSG, "OPU::scpr", "Fault register only partially implemented\n");
                        cancel_run(STOP_WARN);
                        break;
                    case 6:
                        unimp = 0;
                        words[0] = MR.word;
                        words[1] = CMR; // cache mode register
                        break;
                    case 010:
                    case 020:
                    case 040:
                        break;
                    default:
                        // illegal procedure fault
                        fault_gen(illproc_fault);
                        return 1;
                }
                if (unimp) {
                    log_msg(WARN_MSG, "OPU::scpr", "Unimplemented tag %#o\n", ip->mods.single.tag);
                    cancel_run(STOP_BUG);
                } else {
                    ret = store_pair(ip->addr, words[0], words[1]);
                    cancel_run(STOP_WARN);
                }
                return ret;
            }

            case opcode0_scu: {
                // AL39 says that CU data is not, in general, valid at any time
                // except when safe-stored by the first of the pair of the
                // instructions associated with the fault or interrupt.
                // On the other hand, the T&D tape expects that a ldi/scu pair
                // results in the updated IR and IC being stored.
                // The T&D tape also expects that the saved data will
                // appropriately reflect whether or not the CPU was in
                // appending mode at the time of a fault even though the
                // subsequent scu was executed in absolute mode.
                // An alternative to our implemention might be to save state 
                // fter every instruction except xed instructions.
                extern events_t events; // BUG: put in hdr file or hide behind an access function
                log_msg(WARN_MSG, "OPU::scu", "Not fully implemented\n");
                if (switches.FLT_BASE == 2) {
                    log_msg(WARN_MSG, "OPU::scu", "Auto breakpoint\n");
                    cancel_run(STOP_WARN);
                }
                return store_yblock8(TPR.CA, scu_data);
            }

            case opcode0_sdbr: {
                t_uint64 word0, word1;
                word0 = (t_uint64) cpup->DSBR.addr << 12;
                word1 = setbits36(0, 37-36, 14, cpup->DSBR.bound);
                word1 = setbits36(word1, 55-36, 1, cpup->DSBR.u);
                word1 = setbits36(word1, 60-36, 12, cpup->DSBR.stack);
                return store_pair(TPR.CA, word0, word1);
            }
            // sptp unimplemented -- T&D Instruction
            // sptr unimplemented -- T&D Instruction
            // ssdp unimplemented -- T&D Instruction
            // ssdr unimplemented -- T&D Instruction

            case opcode0_cams: {    // Clear Associative Memory Segments
                int ret = 0;
                int clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                if (enable == 2) {
                    cu.SD_ON = 1;
                    log_msg(INFO_MSG, "OPU::cams", "Enabling SDWAM\n");
                } else if (enable == 1) {
                    cu.SD_ON = 0;
                    log_msg(INFO_MSG, "OPU::cams", "Disabling SDWAM\n");
                } else if (enable == 0) {
                    log_msg(DEBUG_MSG, "OPU::cams", "Neither enable nor disable requested\n");
                } else {
                    log_msg(WARN_MSG, "OPU::cams", "Unknown enable/disable mode %06o=>%#o\n", TPR.CA, enable);
                    cancel_run(STOP_WARN);
                }
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    log_msg(ERR_MSG, "OPU::cams", "fault.\n");
                    fault_gen(illproc_fault);
                    cancel_run(STOP_WARN);
                    return 1;
                }
                int i;
                for (i = 0; i < 16; ++i) {
                    cpup->SDWAM[i].assoc.is_full = 0;
                    cpup->SDWAM[i].assoc.use = i;
                    if (clear) {
                        ret = 1;
                        log_msg(WARN_MSG, "OPU::cams", "Clear mode is unimplemented\n");
                        cancel_run(STOP_WARN);
                        // St the full/empty bits of all cache blocks to empty
                        // -- what are cache blocks?
                    }
                    // BUG: the enable flag in each register probably means something different
                    //if (enable == 2)
                    //  cpup->SDWAM[i].assoc.enabled = 1;
                    //else if (enable == 1)
                    //  cpup->SDWAM[i].assoc.enabled = 0;
                }
                return ret;
            }

            case opcode0_rmcm:  // read memory controller mask register
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                return scu_get_cpu_mask(TPR.CA);    // don't convert via appending

            case opcode0_rscr: { // priv
                // read SCU (system controller) register into AQ; See rsw for processor switches
                // Al-39 is wrong about the location of 'y'; see code for
                // privileged_mode_ut$rscr in privileged_mode_ut.alm.
                int ret = 0;
                uint y = TPR.CA >> 10;
                if (y > 7) {
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "CA of %#o has high bits set, yielding port # greater than 7: %d (%#o)n", y, y);
                    cancel_run(STOP_BUG);
                }
                uint ea = TPR.CA & 01770;
                log_msg((TPR.CA == 040) ? DEBUG_MSG : INFO_MSG, "OPU::opcode::rscr", "CA is %#o; port # is %d; function is %#o\n", TPR.CA, y, ea);
                // BUG/TODO: calls to scu_xxx below should probably have TPR.CA replaced by 'y' or y<<15 or y*bank_size
                t_bool show_q = 1;
                if (ea == 0) {
                    ; // SC mode reg
                    log_msg(INFO_MSG, "OPU::opcode::rscr", "mode register selected\n");
                    ret = scu_get_mode_register(TPR.CA);
                } else if (ea == 0010) {
                    ; // SC config reg
                    log_msg(INFO_MSG, "OPU::opcode::rscr", "sys config switches selected\n");
                    ret = scu_get_config_switches(TPR.CA);
                } else if (ea == 0020) {
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "port zero selected\n");
                    ret = scu_get_mask(TPR.CA, 0);
                } else if (ea == 0120)
                    ret = scu_get_mask(TPR.CA, 1);
                else if (ea == 0220)
                    ret = scu_get_mask(TPR.CA, 2);
                else if (ea == 0320)
                    ret = scu_get_mask(TPR.CA, 3);
                else if (ea == 0420)
                    ret = scu_get_mask(TPR.CA, 4);
                else if (ea == 0520)
                    ret = scu_get_mask(TPR.CA, 5);
                else if (ea == 0620)
                    ret = scu_get_mask(TPR.CA, 6);
                else if (ea == 0720) {
                    ret = scu_get_mask(TPR.CA, 7);
                } else if (ea == 0030) {
                    // interrupts
                    extern events_t events; // BUG: put in hdr file, or into a pre-SCU structure, or hide behind an access function
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "AL-39 doesn't specify if non-interrupt bits in regs A and Q should be cleared.\n");
                    reg_A = setbits36(reg_A, 0, 15, 0);
                    reg_Q = setbits36(reg_Q, 0, 15, 0);
                    if (events.int_pending) {
                        for (unsigned i = 0; i < ARRAY_SIZE(events.interrupts); ++i)
                            if (events.interrupts[i]) {
                                if (i <= 15)
                                    reg_A = setbits36(reg_A, i, 1, 1);
                                else
                                    reg_Q = setbits36(reg_Q, 15-i, 1, 1);
                            }
                    }
                    cancel_run(STOP_WARN);
                    ret = 1;
                } else if (ea == 0040) {
                    ret = scu_get_calendar(TPR.CA);
                    show_q = 0;
                } else if (ea == 0050) {
                    ret = scu_get_calendar(TPR.CA);
                    show_q = 0;
                } else if (ea == 0060 || ea == 0070) {
                    // get store unit mode reg
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "get store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "bad argument, CA %#o => function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "A = %llo\n", reg_A);
                if (show_q)
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "Q = %llo\n", reg_Q);
                return ret;
            }

            case opcode0_rsw: { // read processor switches (see rscr for scu switches)
                int low = TPR.CA & 07;
                switch(low) {
                    case 0: // unformatted; maintenance panel data switches
                        // GB61-01B reports which of the data switches should be turned on.
                        // It also discusses other switches, but these other switches are probably
                        // not considered to be part of the "data" switches.
                        reg_A = (t_uint64) 024000717200; // switches: 4, 6, 18, 19, 20, 23, 24, 25, 26, 28
                        break;
                    case 1: // Config switches for ports A, B, C, D
                        rsw_get_port_config(0);
                        log_msg(INFO_MSG, "OPU::opcode::rsw", "function xxx%o sets A=%012llo.\n", low, reg_A);
                        break;
                    case 2:
#if 0
                        // from start_cpu.pl1
                        reg_A = setbits36(0, 4, 2, 0);  // 0 for L68 or DPS
                        reg_A = setbits36(reg_A, 6, 7, switches.FLT_BASE);  // 7 MSB bits of 12bit addr
                        log_msg(INFO_MSG, "OPU::opcode::rsw", "Fault base in bits 6..13 is %#o=>%#o\n", switches.FLT_BASE, switches.FLT_BASE << 5);
                        reg_A = setbits36(reg_A, 19, 1, 0); // 0 for L68
                        reg_A = setbits36(reg_A, 27, 1, 0); // cache
                        reg_A = setbits36(reg_A, 28, 1, 0); // gcos mode extended memory option off
                        reg_A = setbits36(reg_A, 29, 4, 016);   // 1110b=>L68 re start_cpu.pl1
                        reg_A = setbits36(reg_A, 33, 3, switches.cpu_num);
#else
                        // from AL39
                        reg_A = setbits36(0, 0, 6, 0);  // 0..5 are zero for L68 or DPS
                        reg_A = setbits36(reg_A, 6, 7, switches.FLT_BASE);  // 7 MSB bits of 12bit addr
                        reg_A = setbits36(reg_A, 23, 11, 016);  // 1110b=>L68 re start_cpu.pl1
                        reg_A = setbits36(reg_A, 34, 2, switches.cpu_num);
#endif
                        log_msg(INFO_MSG, "OPU::opcode::rsw", "function xxx%o returns A=%012llo.\n", low, reg_A);
                        break;
                    case 3: // Config switches for ports E, F, G, H
                        rsw_get_port_config(4);
                        log_msg(INFO_MSG, "OPU::opcode::rsw", "function xxx%o sets A=%012llo.\n", low, reg_A);
                        break;
                    case 4:
                        reg_A = 0;  // see AL-39, configuration switch data; 0 is full memory with 4 word interlace
                        //for (int i = 0; i < 8; ++i)
                        //  reg_A = setbits36(reg_A, 13 + (i * 2), 2, 2);
                        log_msg(INFO_MSG, "OPU::opcode::rsw", "function xxx%o returns A=%012llo.\n", low, reg_A);
                        break;
                    default:
                        log_msg(WARN_MSG, "OPU::opcode::rsw", "function xxx%o not implemented.\n", low);
                        cancel_run(STOP_BUG);
                }
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            }

            case opcode0_cioc: { // priv
                // Connect I/O channel
                log_msg(DEBUG_MSG, "OPU", "CIOC\n");
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret;
                uint addr;
                if (get_addr_mode() == APPEND_mode) {
                    if ((ret = get_seg_addr(TPR.CA, 0, &addr)) != 0)
                        return ret;
                } else 
                    addr = TPR.CA;
                return scu_cioc(addr);  // we do convert via appending
            }
            case opcode0_smcm: { // priv
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                return scu_set_cpu_mask(TPR.CA);    // don't convert via appending
            }

            // smic unimplemented

            case opcode0_sscr: { // priv
                // set system controller register (to value in AQ)
                // Al-39 is wrong about the location of 'y'; see code for
                // privileged_mode_ut$rscr in privileged_mode_ut.alm.
                if (get_addr_mode() == BAR_mode) {
                    log_msg(ERR_MSG, "OPU::sscr", "fault: BAR mode.\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                if (! PPR.P) {
                    log_msg(ERR_MSG, "OPU::sscr", "fault: not privileged mode; segment %#o\n", TPR.TSR);
                    fault_gen(illproc_fault);
                    return 1;
                }
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "A = %llo\n", reg_A);
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "Q = %llo\n", reg_Q);
                int ret = 0;
                uint y = TPR.CA >> 10;
                if (y > 7) {
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "CA of %#o has high bits set, yielding port # greater than 7: %d (%#o)n", y, y);
                    cancel_run(STOP_BUG);
                }
                uint ea = TPR.CA & 01770;
                log_msg((TPR.CA == 040) ? DEBUG_MSG : INFO_MSG, "OPU::opcode::sscr", "CA is %#o; port # is %d; function is %#o\n", TPR.CA, y, ea);
                // BUG/TODO: calls to scu_xxx below should probably have TPR.CA replaced by 'y' or y<<15 or y*bank_size
                if (ea == 0) {
                    ; // SC mode reg
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "mode register selected\n");
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if (ea == 0010) {
                    ; // SC config reg
                    log_msg(INFO_MSG, "OPU::opcode::sscr", "sys config switches selected\n");
                    ret = scu_set_config_switches(TPR.CA);
                } else if (ea == 0020) {
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "port zero selected\n");
                    ret = scu_set_mask(TPR.CA, 0);
                } else if (ea == 0120)
                    ret = scu_set_mask(TPR.CA, 1);
                else if (ea == 0220)
                    ret = scu_set_mask(TPR.CA, 2);
                else if (ea == 0320)
                    ret = scu_set_mask(TPR.CA, 3);
                else if (ea == 0420)
                    ret = scu_set_mask(TPR.CA, 4);
                else if (ea == 0520)
                    ret = scu_set_mask(TPR.CA, 5);
                else if (ea == 0620)
                    ret = scu_set_mask(TPR.CA, 6);
                else if (ea == 0720) {
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "port seven selected\n");
                    ret = scu_set_mask(TPR.CA, 7);
                } else if (ea == 0030) {
                    // interrupts
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if (ea == 0040) {
                    // calendar
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if (ea == 0050) {
                    // calendar
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if (ea == 0060) {
                    // store unit mode reg
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if (ea == 0070) {
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented; CA == %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "bad argument, CA %#o, yielding function %#o\n", TPR.CA, ea);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                return ret;
            }

            case opcode0_absa: {    // priv
                addr_modes_t mode;
                if ((mode = get_addr_mode()) != APPEND_mode) {
                    if (mode == ABSOLUTE_mode) {
                        log_msg(WARN_MSG, "OPU::absa", "Absolute mode undefined\n");
                        reg_A = (t_uint64) TPR.CA << 12;    // results undefined, perform arbitrary action
                        cancel_run(STOP_WARN);
                        return 0;
                    }
                    if (mode == BAR_mode)
                        fault_gen(illproc_fault);
                    return 1;
                }
                if (opt_debug) log_msg(DEBUG_MSG, "OPU::absa", "Getting segment translation (DSBR.bound = %#o).\n", cpup->DSBR.bound);
                uint addr;
                int ret;
                if ((ret = get_seg_addr(TPR.CA, 0, &addr)) == 0)
                    reg_A = (t_uint64) addr << 12;  // upper 24 bits
                else
                    log_msg(WARN_MSG, "OPU::absa", "Unable to translate segment offset into absolute address.\n");
                if (opt_debug) {
                    if (addr == TPR.CA)
                        log_msg(DEBUG_MSG, "OPU::absa", "Using segment portion of PR register yields no change -- %#llo\n", reg_A);
                    else
                        log_msg(DEBUG_MSG, "OPU::absa", "Using segment portion of PR register yields %#llo instead of %#llo.\n", reg_A, (t_uint64) TPR.CA << 12);
                }
                return ret;
            }

            case opcode0_dis:
                // delay until interrupt set
                if (ip->inhibit) {
                    log_msg(WARN_MSG, "OPU::dis", "DIS with inhibit set\n");
                    cancel_run(STOP_IBKPT);
                } else {
                    log_msg(WARN_MSG, "OPU::dis", "DIS\n");
                }
                cpu.cycle = DIS_cycle;
                return 1;


            // limr ??
            // ldo ??
            // camp2 ??
            //

            default:
                log_msg(ERR_MSG, "OPU", "Unimplemented opcode %03o(0)\n", op);
                cancel_run(STOP_BUG);
                return 1;
        }
    } else {
        switch (op) {
            case opcode1_tmoz:
                if (IR.neg || IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            case opcode1_tpnz:
                if (! IR.neg && ! IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            case opcode1_trtf:
                if (! IR.truncation) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            case opcode1_trtn:
                if (IR.truncation) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;
            case opcode1_ttn:
                if (IR.tally_runout) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                    cpu.trgo = 1;
                }
                return 0;

            case opcode1_easp1:
                return do_easp(1);
            case opcode1_easp3:
                return do_easp(3);
            case opcode1_easp5:
                return do_easp(5);
            case opcode1_easp7:
                return do_easp(7);

            case opcode1_eawp1:
                return do_eawp(1);
            case opcode1_eawp3:
                return do_eawp(3);
            case opcode1_eawp5:
                return do_eawp(5);
            case opcode1_eawp7:
                return do_eawp(7);

            case opcode1_epbp0:
                return do_epbp(0);
            case opcode1_epbp2:
                return do_epbp(2);
            case opcode1_epbp4:
                return do_epbp(4);
            case opcode1_epbp6:
                return do_epbp(6);
    
            case opcode1_epp1:
                return do_epp(1);
            case opcode1_epp3:
                return do_epp(3);
            case opcode1_epp5:
                return do_epp(5);
            case opcode1_epp7:
                return do_epp(7);

            case opcode1_spbp0:
                return do_spbp(0);
            case opcode1_spbp2:
                return do_spbp(2);
            case opcode1_spbp4:
                return do_spbp(4);
            case opcode1_spbp6:
                return do_spbp(6);

            case opcode1_spri1:
                return do_spri(1);
            case opcode1_spri3:
                return do_spri(3);
            case opcode1_spri5:
                return do_spri(5);
            case opcode1_spri7:
                return do_spri(7);

            // opcode1_sra unimplemented
            // lptp unimplemented
            // lptr unimplemented
            // lra unimplemented
            // lsdr unimplemented
            // sptp unimplemented
            // sptr unimplemented
            // ssdr unimplemented

            case opcode1_camp: {    // Clear Associative Memory Pages
                if (get_addr_mode() == BAR_mode || ! PPR.P) {
                    log_msg(WARN_MSG, "OPU::camp", "faulting.\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                int sel_clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                if (enable == 2) {
                    cu.PT_ON = 1;
                    log_msg(INFO_MSG, "OPU::camp", "Enabling PTWAM\n");
                } else if (enable == 1) {
                    cu.PT_ON = 0;
                    log_msg(INFO_MSG, "OPU::camp", "Disabling PTWAM\n");
                } else if (enable == 0) {
                    // Don't change enabled/disabled status
                } else {
                    log_msg(WARN_MSG, "OPU::camp", "Unknown enable/disable mode %06o=>%#o\n", TPR.CA, enable);
                    cancel_run(STOP_WARN);
                }
                int i;
                for (i = 0; i < 16; ++i) {
                    cpup->PTWAM[i].assoc.is_full = 0;
                    cpup->PTWAM[i].assoc.use = i;
                    if (sel_clear) {
                        ret = 1;
                        log_msg(WARN_MSG, "OPU::camp", "Selective Clear mode is Unimplemented\n");
                        cancel_run(STOP_BUG);
                        // for any cache block for which upper 15 bits of the dir [are]
                        // entry equal PTWAM(i).ADDR0..13, 
                        //  set full/empty to empty
                        // -- what are cache blocks and dirs?
                    }
                    //if (enable == 2)
                    //  cpup->PTWAM[i].assoc.enabled = 1;
                    //else if (enable == 1)
                    //  cpup->PTWAM[i].assoc.enabled = 0;
                }
                return ret;
            }

            // aar0 .. aar7 unimplemented -- alphanumeric descriptor to address register
            // lar0 .. lar7 unimplemented
            // lareg unimplemented
            // lpl unimplemented
            // nar0 .. nar7  unimplemented
            // ara0 .. ara7 unimplemented
            // arn0 .. arn7 unimplemented
            // sar0 .. sar7 unimplemented
            // sareg unimplemented

            case opcode1_spl: {
                // BUG: Restart of EIS instructions not yet supported, so spl is a no-op
                t_uint64 words[8] = { 0, 0, 0, 0, 0, 0, 0, 0 };
                log_msg(WARN_MSG, "OPU::spl", "Not storing enough info to allow EIS decimal op restart.\n");
                cancel_run(STOP_BUG);
                return store_yblock8(TPR.CA, words);
            }

            // a4bd unimplemented -- add 4 bit displacement to addr register
            // a6bd
            case opcode1_a9bd:  // add 9 bit displacement to addr register
                log_msg(DEBUG_MSG, "OPU::a9bd", "APU does our work for us\n");
                return 0;

            case opcode1_abd:   // add bit displacement to addr register
                log_msg(DEBUG_MSG, "OPU::a9bd", "APU does our work for us\n");
                return 0;

            case opcode1_awd:
                log_msg(DEBUG_MSG, "OPU::awd", "APU does our work for us\n");
                return 0;

            // s4bd unimplemented
            // s6bd unimplemented

            case opcode1_s9bd:
                log_msg(DEBUG_MSG, "OPU::s9bd", "APU does our work for us\n");
                return 0;

            // sbd unimplemented
            // swd unimplemented

            case opcode1_cmpc: {
                // extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_cmpc(ip);
                // --opt_debug; --cpu_dev.dctrl;
                return ret;
            }

            // opcode1_scd unimplemented -- scan characters double
            // opcode1_scdr unimplemented -- scan characters double in reverse
            case opcode1_scm: { // scan with mask
                extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_scm(ip, 1);
                --opt_debug; --cpu_dev.dctrl;
                return ret;
            }
            
            case opcode1_scmr: {    // scan with mask in reverse
                extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_scm(ip, 0);
                --opt_debug; --cpu_dev.dctrl;
                return ret;
            }

            case opcode1_tct: {
                extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_tct(ip, 1);
                --opt_debug; --cpu_dev.dctrl;
                return ret;
            }
            case opcode1_tctr: {
                extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_tct(ip, 0);
                --opt_debug; --cpu_dev.dctrl;
                return ret;
            }
            case opcode1_mlr: {
                int ret = op_move_alphanum(ip, 1);
                return ret;
            }
            case opcode1_mrl: {
                int ret = op_move_alphanum(ip, 0);
                return ret;
            }

            // mve unimplemented -- move alphanumeric edited
            case opcode1_mvt: {
                int ret = op_mvt(ip);
                return ret;
            }
            // cmp0 .. cmp7 unimplemented -- compare numeric

            case opcode1_mvn:       // EIS: move numeric
                return op_mvn(ip);

            case opcode1_mvne:      // EIS: move numeric edited
                return op_mvne(ip);

            case opcode1_csl: {     // EIS: combine bit strings left
                extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
                int ret = op_csl(ip);
                -- opt_debug; -- cpu_dev.dctrl;
                return ret;
            }
            
            // csr unimplemented -- combine bit strings right

            case opcode1_cmpb:
                return op_cmpb(ip);

            // sztl unimplemented -- similar to csl?
            // sztr unimplemented -- similar to csr?

            case opcode1_btd:       // binary to decimal convert
                return op_btd(ip);

            case opcode1_dtb:   // decimal to binary convert
                return op_dtb(ip);

            // ad2d unimplemented -- add using two decimal operands
            // ad3d unimplemented -- add using three decimal operands

            // sb2d unimplemented -- subtract using two decimal operands; BUG: see comments by rmabee@comcast.net:
        /*
        On Aug 11, 3:14 am, rfm <rma...@comcast.net> wrote:
        > The EIS decimal subtract SB2D and DV2D do not agree on the order of
        > the inputs in the RTL:
        > SB2D: Y1 - Y2 -> Y2
        > DV2D: Y2 - Y1 -> Y2
        > I believe SB2D is misdocumented.
        
        I dug out a 1972 manual for the EIS add-on (to pre-Multics base),
        which confirms that SB2D is Y2 - Y1 -> Y2.
        */

            // sb3d unimplemented -- subtract using three decimal operands
            // mp2d unimplemented -- multiply using two decimal operands
            // mp2d unimplemented -- multiply using three decimal operands
            // dv2d unimplemented -- divide using two decimal operands; BUG: see comments by rmabee@comcast.net

            case opcode1_dv3d:      // divide using three decimal operands
                return op_dv3d(ip);

            default:
                log_msg(ERR_MSG, "OPU", "Unimplemented opcode %03o(1)\n", op);
                cancel_run(STOP_BUG);
                return 1;
        }
    }
    return 0;
}

// ============================================================================

void cu_safe_store()
{
    // Save current Control Unit Data in hidden temporary so a later SCU
    // instruction running in FAULT mode can save the state as it existed
    // at the time of the fault rather than as it exists at the time the scu
    // instruction is executed.
    log_msg(INFO_MSG, "CU", "Safe storing SCU data\n");
    scu2words(scu_data);
}

// ----------------------------------------------------------------------------

static void scu2words(t_uint64 *words)
{
    // BUG:  We don't track much of the data that should be tracked

    memset(words, 0, 8 * sizeof(*words));

    // See AL39 table 5-1
    words[0] = setbits36(0, 0, 3, PPR.PRR);
    words[0] = setbits36(words[0], 3, 15, PPR.PSR);
    words[0] = setbits36(words[0], 18, 1, PPR.P);
    // 19 "b" XSF
    // 20 "c" SDWAMN
    words[0] = setbits36(words[0], 21, 1, cu.SD_ON);
    // 22 "e" PTWAM
    words[0] = setbits36(words[0], 23, 1, cu.PT_ON);
    // 24..32 various
    // 33-35 FCT

    // words[1]

    words[2] = setbits36(0, 0, 3, TPR.TRR);
    words[2] = setbits36(words[2], 3, 15, TPR.TSR);
    words[2] = setbits36(words[2], 27, 3, switches.cpu_num);
    words[2] = setbits36(words[2], 30, 6, cu.delta);

    words[3] = 0;
    words[3] = setbits36(words[3], 30, 6, TPR.TBR);

    save_IR(&words[4]);
    words[4] = setbits36(words[4], 0, 18, PPR.IC);

    words[5] = setbits36(0, 0, 18, TPR.CA);
    words[5] = setbits36(words[5], 18, 1, cu.repeat_first);
    words[5] = setbits36(words[5], 19, 1, cu.rpt);
    // BUG: Not all of CU data exists and/or is saved
    words[5] = setbits36(words[5], 24, 1, cu.xde);
    words[5] = setbits36(words[5], 25, 1, cu.xdo);
    words[5] = setbits36(words[5], 30, 6, cu.CT_HOLD);

    encode_instr(&cu.IR, &words[6]);    // BUG: cu.IR isn't kept fully up-to-date

    words[7] = cu.IRODD;
}

// ----------------------------------------------------------------------------

static void words2scu(const t_uint64 *words)
{
    // BUG:  We don't track much of the data that should be tracked

    // See AL39 table 5-1

    // words[0]
    PPR.PRR = getbits36(words[0], 0, 3);
    PPR.PSR = getbits36(words[0], 3, 15);
    PPR.P = getbits36(words[0], 18, 1);

    // words[1] - none of these loaded by rcu
 
    // words[2]
    TPR.TRR = getbits36(words[2], 0, 3);
    TPR.TSR = getbits36(words[2], 3, 15);
    switches.cpu_num = getbits36(words[2], 27, 3);  // FIXME: Really?
    cu.delta = getbits36(words[2], 30, 6);

    // words[3]
    TPR.TBR = getbits36(words[3], 30, 6);

    // words[4]
    PPR.IC = getbits36(words[4], 0, 18);
    load_IR(&IR, words[4]); // FIXME: do we need to suppress any bits?

    // words[5]
    cu.repeat_first = getbits36(words[5], 18, 1);
    cu.rpt = getbits36(words[5], 19, 1);
    cu.xde = getbits36(words[5], 24, 1);
    cu.xdo = getbits36(words[5], 25, 1);
    cu.CT_HOLD = getbits36(words[5], 30, 6);

    decode_instr(&cu.IR, words[6]);    // BUG: cu.IR isn't kept fully up-to-date

    cu.IRODD = words[7];
}

// ============================================================================

static int op_add(instr_t *ip, t_uint64 *dest)
{
    int ret;
    t_uint64 word;
    if ((ret = fetch_op(ip, &word)) != 0)
        return ret;
    t_uint64 w = word;
    t_uint64 d = *dest;
    ret = add36(word, *dest, dest);
    return ret;
}

// ----------------------------------------------------------------------------

static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest)
{
    uint sign1 = a >> 35;
    uint sign2 = b >> 35;
    t_uint64 result = a + b;
    if ((result >> 36) != 0) {
        // BUG: generates inappropriate? carry when adding a negative to a positive number
        IR.carry = 1;
        //log_msg(WARN_MSG, "OPU::add36", "%012llo (%lld) + %012llo (%lld) ==> carry on result %012llo (%lld => %lld)\n",
        //  a, sign36(a), b, sign36(b), result, sign36(result), sign36(result & MASK36));
        result &= MASK36;
    } else {
        IR.carry = 0;
    }
    uint signr = result >> 35;
    IR.zero = result == 0;
    IR.neg = signr;
    if (sign1 == sign2 && signr != sign1) {
        if(opt_debug) log_msg(DEBUG_MSG, "OPU::add36", "%012llo (%lld) + %012llo (%lld) ==> overflow on result %012llo (%lld)\n",
            a, sign36(a), b, sign36(b), result, sign36(result));
        IR.overflow = 1;
        if (IR.overflow_mask == 0) {
            fault_gen(overflow_fault);
            return 1;
        }
    }

    *dest = result;

    // see comments at opcode0_sba
#if 0
    if (do_18bit_math && TPR.is_value == 7) {
        // arithmetic on "dl" constant
        if ((a>>18) == 0 && (b>>18) == 0) {
            if (opt_debug || (result>>18) != 0)
                log_msg(NOTIFY_MSG, "OPU::add36", "%012llo + %012llo = %012llo for ,dl at IC %06o\n", a, b, result, PPR.IC);
            flag_t is_neg = bit18_is_neg(result);
            if (is_neg != IR.neg) {
                // problems: 017127, 017207, 017267, 017347
                //if (PPR.IC >= 0013767 && PPR.IC <= 017047)
                if ((PPR.IC >= 0013767 && PPR.IC <= 017047) || PPR.IC == 017127)
                    log_msg(WARN_MSG, "OPU::add36", "**Special case: NOT changing IR.neg to %d for ,dl operand at IC %#o.\n", is_neg, PPR.IC);
                else {
                    log_msg(WARN_MSG, "OPU::add36", "Changing IR.neg to %d for ,dl operand at IC %#o\n", is_neg, PPR.IC);
                    IR.neg = is_neg;
                }
                if (get_addr_mode() != ABSOLUTE_mode)
                    cancel_run(STOP_WARN);
            }
            if ((result >> 18) != 0 && ! IR.carry) {
                log_msg(WARN_MSG, "OPU::add36", "Carry is probably wrong for ,dl operand.\n");
                cancel_run(STOP_WARN);
            }
        }
    }
#endif

    return 0;
}

// ----------------------------------------------------------------------------

/*
 * sub36()
 *
 * NOTES
 *    AL39, section two claims that subtraction uses two's complement.  However,
 *    it's not that simple.  Diagnostic tape t4d_b.2.tap expects that
 *    <max negative number> minus zero will generate a carry.  The diagnostic
 *    tape also expects that negative one minus zero will yield a carry.
 *
 * IMPLEMENTATION
 *    So, instead of adding in the two's complement, we'll first add the one's
 *    complement and then add one.
 *
 *  ************
 *  All the following is noise -- valid but unexpected behavior of carry bit...
 *  ************
 *
 * NOTE
 *    Subtracting a number from itself will yield zero with an overflow for
 *    either approach -- adding in the 1's complement then adding one or 
 *    when directly adding in the two's complement
 *
 * HOWEVER:
 * Subtracting a small positive number from a larger positive number
 * generates a carry!
 *        004000000000
 *      - 000100000000
 *        ------------
 *                        004000000000
 *                      + 777677777777 (one's complement of 2nd arg)
 *                        ------------
 *                        003677777777 (carry is yes)
 *                      +            1
 *                        ------------
 *                        003700000000 (no flags)
 *        ------------
 *        003700000000 (carry is yes -- really a borrow)
 *
 * WORSE:
 * Subtracting a small number from itself yields carry & overflow:
 *        000100000000
 *      - 000100000000
 *        ------------
 *                        000100000000
 *                      + 777677777777 (one's complement of 2nd arg)
 *                        ------------
 *                        777777777777 (carry is no)
 *                      +            1
 *                        ------------
 *                        000000000000 (zero, carry, overflow?)
 *       ------------
 *       000000000000 (zero, carry, overflow?)
 *
 * NEED:
 *        100000000000
 *      - 000000000000
 *        ------------
 *        100000000000 (carry)
 * NEED:
 *        377777777777
 *      - 000000000000
 *        ------------
 *        377777777777 (carry)
 *
 * WHAT IF add 1s complement first? (OK)
 *        100000000000
 *      - 000000000000
 *        ------------
 *                        100000000000 (max negative)
 *                      +            1
 *                        ------------
 *                        100000000001 (no flags, almost max neg)
 *                      + 777777777777 (one's complement of 2nd arg)
 *                        ------------
 *                        100000000000 (carry out)
 *
 * WHAT IF add 1s complement first? (maybe OK)
 *        004000000000
 *      - 000100000000
 *        ------------
 *                        004000000000
 *                      +            1
 *                        ------------
 *                        004000000001 (no flags)
 *                      + 777677777777 (one's complement of 2nd arg)
 *                        ------------
 *                        003700000000 (carry)
 *        ------------
 *        003700000000 (carry -- really a borrow)
 *
 * WHAT IF add 1s complement first? (no go)
 *        000100000000
 *      - 000100000000
 *        ------------
 *                        000100000000
 *                      +            1
 *                        ------------
 *                        000100000001
 *                      + 777677777777 (one's complement of 2nd arg)
 *                        ------------
 *                        000100000000 (oveflow or carry?)
 *
 * WHAT IF set flags during 2s complement? (OK?)
 *        100000000000
 *      - 000000000000
 *                        000000000000 2nd arg
 *                        777777777777 one's complement
 *                      +            1
 *                        ------------
 *                        000000000000 (carry)
 *                      + 100000000000
 *                        ------------
 *                      + 100000000000 (no flags)
 *        ------------
 *        100000000000 (carry)
 *
 * WHAT IF set flags during 2s complement? (maybe ok)
 *        004000000000
 *      - 000100000000
 *        ------------
 *                        000100000000 2nd arg
 *                        777677777777 (one's complement of 2nd arg)
 *                      +            1
 *                        ------------
 *                        777700000000 (no flags)
 *                      + 004000000000
 *                        ------------
 *                        003700000000 (carry)
 *        ------------
 *        003700000000 (carry is yes -- really a borrow)
 *
 * WHAT IF set flags during 2s complement? (maybe ok)
 * Subtracting a small number from itself
 *        000100000000
 *      - 000100000000
 *        ------------
 *                        777677777777 (one's complement of 2nd arg)
 *                      +            1
 *                        777700000000 (no flags)
 *                      + 000100000000
 *                        ------------
 *                        000000000000 (zero, carry, overflow?)
 *       ------------
 *       000000000000 (zero, carry, overflow?)
 *
 *
 * WHAT IF we don't allow the 2s complement ops to set flags?
 * -- we would not get flags when subtracting zero...
 *
 * TODO: log the snippits of code from the diag tape...
 */

static int sub36(t_uint64 a, t_uint64 b, t_uint64 *dest)
{
    t_uint64 bsv = b;
    t_uint64 asv = a;

#if 0
    /*
     * Add one, then add in the one's complement of the second arg
     */

    ...

#else
    /*
     * Add in the one's complement of the second arg and then add one.
     */

    b = (~ b) & MASK36;
    int ret = add36(a, b, &a);

    int carry = IR.carry;
    if (opt_debug) {
        log_msg(DEBUG_MSG, "OPU::subtract", "%012llo - %012llo\n", asv, bsv);
        log_msg(DEBUG_MSG, "OPU::subtract", "%012llo + %012llo yields %012llo with carry=%c, overflow=%c.\n",
            asv, b, a, IR.carry ? 'Y' : 'N', IR.overflow ? 'Y' : 'N');
    }

    if (ret == 0) {
        b = 1;
        ret = add36(a, b, &a);
        log_msg(DEBUG_MSG, "OPU::subtract", "adding one yields %012llo with carry=%c, overflow=%c.\n", a, IR.carry ? 'Y' : 'N', IR.overflow ? 'Y' : 'N');
        IR.carry |= carry;
        if (ret == 0)
            *dest = a;
    } // ret == 0

    return ret;
#endif
}

// ----------------------------------------------------------------------------

static int add18(t_uint64 a, t_uint64 b, t_uint64 *dest)
{
    
    a &= MASK18;
    b &= MASK18;
    uint sign1 = a >> 17;
    uint sign2 = b >> 17;
    uint32 result = a + b;
    if ((result >> 18) != 0) {
        IR.carry = 1;
        result &= MASK18;
    } else {
        IR.carry = 0;
    }
    IR.zero = (result == 0);
    uint signr = result >> 17;
    IR.neg = signr;
#if  0
example: 060 plus negative one
0777777
   + 60
1000057
#endif
    if (sign1 == sign2 && signr != sign1) {
        IR.overflow = 1;
        if (IR.overflow_mask == 0) {
            // BUG: Should we overwrite dest on overflow fault?
            fault_gen(overflow_fault);
            return 1;
        }
    }

    *dest = result;
    return 0;
}


// ----------------------------------------------------------------------------

int add72(t_uint64 ahi, t_uint64 alow, t_uint64* dest1, t_uint64* dest2, int is_unsigned)
{
    t_uint64 word1, word2;

    uint sign1 = ahi >> 35;
    uint sign2 = *dest1 >> 35;

    t_uint64 lo = alow + *dest2;
    int lo_carry = (lo >> 36);
    lo &= MASK36;

    t_uint64 hi = ahi + *dest1;
    if (lo_carry)
        ++ hi;
    IR.carry = (hi >> 36) != 0;
    if (IR.carry)
        hi &= MASK36;

    IR.zero = lo == 0 && hi == 0;
    uint signr = hi >> 35;
    IR.neg = signr;
    if (! is_unsigned)
        if (sign1 == sign2 && signr != sign1) {
            IR.overflow = 1;
            if (IR.overflow_mask == 0) {
                fault_gen(overflow_fault);
                return 1;
            }
        }

    *dest1 = hi;
    *dest2 = lo;
    return 0;
}

// ============================================================================

static int op_and(instr_t *ip, t_uint64 *op1, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2)
{
    // Use a y-pair if op2 non null
    int ret;

    t_uint64 word1, word2;
    if (op2 == NULL) {
        if ((ret = fetch_op(ip, &word1)) != 0)
            return ret;
        if (opt_debug)
            log_msg(DEBUG_MSG, "OPU::and", "%012llo & %012llo\n", *op1, word1);
    } else {
        if ((ret = fetch_pair(TPR.CA, &word1, &word2)) != 0)    // BUG: fetch_op not needed?
            return ret;
    }
    *dest1 = word1 & *op1;
    if (op2 != NULL && dest2 != NULL)
        *dest2 = word2 & *op2;

    IR.zero = (*dest1 == 0) && (dest2 == NULL || *dest2 == 0);
    IR.neg = bit36_is_neg(*dest1);  // dest1 presumed to hold bit zero
    return 0;
}

// ============================================================================

static inline t_uint64 lrotate36(t_uint64 x, unsigned n)
{
    if (n >= 36)
        n %= 36;
    return ((x << n) & MASK36) | (x >> (36-n));
}

static inline void lrotate72(t_uint64* ap, t_uint64* bp, unsigned n)
{
    if (n >= 72)
        n %= 72;
    if (n == 0)
        return;

    t_uint64 a = *ap;
    t_uint64 b = *bp;
    if (n <= 36) {
        t_uint64 aout = a >> (36 - n);
        t_uint64 bout = b >> (36 - n);
        a = ((a << n) & MASK36) | bout;
        b = ((b << n) & MASK36) | aout;
    } else {
        t_uint64 aout_hi = a >> (72 - n);
        t_uint64 aout_lo = a & ~(~(t_uint64)0 << (n - 36));
        t_uint64 bout_hi = b >> (72 - n);
        t_uint64 bout_lo = b & ~(~(t_uint64)0 << (n - 36));
        a = (bout_lo << (n - 36)) | aout_hi;
        b = (aout_lo << (n - 36)) | bout_hi;
    }
            
    *ap = a;
    *bp = b;
}

// ============================================================================

static inline uint min(uint a, uint b)
{
    return (a < b) ? a : b;
}

static inline uint max(uint a, uint b)
{
    return (a > b) ? a : b;
}

static inline uint max3(uint a, uint b, uint c)
{
    return (a > b) ?
        ((a > c) ? a : c) :
        ((b > c) ? b : c);
}

// ============================================================================

static int do_epp(int epp)
{
    if (get_addr_mode() == BAR_mode) {
        fault_gen(illproc_fault);   // BUG: which sub-fault?
        return 1;
    }

    AR_PR[epp].PR.rnr = TPR.TRR;
    AR_PR[epp].PR.snr = TPR.TSR;
    AR_PR[epp].wordno = TPR.CA & MASK18;
    AR_PR[epp].PR.bitno = TPR.TBR;
    if ((int) AR_PR[epp].PR.bitno < 0 || AR_PR[epp].PR.bitno > 35) {
        log_msg(ERR_MSG, "OPU::epp*", "PR%d now has a bitno of %d\n", epp, AR_PR[epp].PR.bitno );
        cancel_run(STOP_BUG);
    }
    AR_PR[epp].AR.charno = AR_PR[epp].PR.bitno / 9;
    AR_PR[epp].AR.bitno = AR_PR[epp].PR.bitno % 9;
    char buf[20];
    sprintf(buf, "OPU::epp%d", epp);;
    log_msg(DEBUG_MSG, buf, "PR[%o]=TPR -- rnr=%#o, snr=%#o, wordno=%o, bitno=%o\n", epp, AR_PR[epp].PR.rnr, AR_PR[epp].PR.snr, AR_PR[epp].wordno, AR_PR[epp].PR.bitno);
    return 0;
}

// ============================================================================

static int do_epbp(int n)
{
    if (get_addr_mode() == BAR_mode) {
        fault_gen(illproc_fault);   // BUG: which sub-fault?
        return 1;
    }

    AR_PR[n].PR.rnr = TPR.TRR;
    AR_PR[n].PR.snr = TPR.TSR;
    AR_PR[n].wordno = 0;
    AR_PR[n].PR.bitno = 0;
    AR_PR[n].AR.charno = 0;
    AR_PR[n].AR.bitno = 0;
    char buf[20];
    sprintf(buf, "OPU::epbp%d", n);
    log_msg(DEBUG_MSG, buf, "PR[%o]=TPR$0 -- rnr=%#o, snr=%#o, wordno=zero, bitno=zero\n", n, AR_PR[n].PR.rnr, AR_PR[n].PR.snr);

    return 0;
}


// ============================================================================

static int do_spbp(int n)
{
    t_uint64 word0 = setbits36(0, 3, 15, AR_PR[n].PR.snr);
    word0 = setbits36(word0, 18, 3, AR_PR[n].PR.rnr);
    word0 |= 043;
    return store_pair(TPR.CA, word0, 0);
}

// ============================================================================

static int do_easp(int n)
{
    if (get_addr_mode() == BAR_mode) {
        fault_gen(illproc_fault);
        return 1;
    }
    AR_PR[n].PR.snr = TPR.CA;
    return 0;
}

// ============================================================================

static int do_eawp(int n)
{
    if (get_addr_mode() == BAR_mode) {
        fault_gen(illproc_fault);
        return 1;
    }
    AR_PR[n].wordno = TPR.CA;
    AR_PR[n].PR.bitno = TPR.TBR;
    if ((int) AR_PR[n].PR.bitno < 0 || AR_PR[n].PR.bitno > 35) {
        log_msg(ERR_MSG, "OPU::eawp", "PR%d now has a bitno of %d\n", n, AR_PR[n].PR.bitno );
        cancel_run(STOP_BUG);
    }
    AR_PR[n].AR.charno = AR_PR[n].PR.bitno / 9;
    AR_PR[n].AR.bitno = AR_PR[n].PR.bitno % 9;
    return 0;
}

// ============================================================================

static void spri_to_words(int reg, t_uint64* word0p, t_uint64 *word1p)
{
    t_uint64 word0, word1;
    word0 = setbits36(0, 3, 15, AR_PR[reg].PR.snr);
    word0 = setbits36(word0, 18, 3, AR_PR[reg].PR.rnr);
    word0 |= 043;

    word1 = setbits36(0, 0, 18, AR_PR[reg].wordno);
    word1 = setbits36(word1, 21, 6, AR_PR[reg].PR.bitno); // 36-(72-57)
    *word0p = word0;
    *word1p = word1;
}

// ============================================================================

static int do_spri(int n)
{
    if (get_addr_mode() == BAR_mode) {
        fault_gen(illproc_fault);
        return 1;
    }
    t_uint64 word0, word1;
    spri_to_words(n, &word0, &word1);
    if(opt_debug) log_msg(DEBUG_MSG, "OPU::spri*", "Saving PR[%d]: snr=%#o, rnr=%o, wordno=%#o, bitno=%#o\n", n, AR_PR[n].PR.snr, AR_PR[n].PR.rnr, AR_PR[n].wordno, AR_PR[n].PR.bitno);
    // log_msg(NOTIFY_MSG, "OPU::spri*", "Saving PR[%d]: { snr=%#o, rnr=%o, wordno=%#o, bitno=%#o} as {%012llo, %012llo}\n", n, AR_PR[n].PR.snr, AR_PR[n].PR.rnr, AR_PR[n].wordno, AR_PR[n].PR.bitno, word0, word1);
    return store_pair(TPR.CA, word0, word1);
}

// ============================================================================

static int op_dvf(const instr_t* ip)
{
    // dvf -- divide fraction; 71-bit signed fractional dividend  is
    // divided by a 36-bit fractional divisor

    t_uint64 word;
    int ret = fetch_op(ip, &word);
    if (ret != 0)
        return ret;
    return instr_dvf(word);
}

// ============================================================================

static int op_ufa(const instr_t* ip, flag_t subtract)
{
    t_uint64 word;
    int ret = fetch_op(ip, &word);
    if (ret != 0)
        return ret;
    ret = instr_ufas(word, subtract);
    return ret;
}

// ============================================================================

static int op_ufm(const instr_t* ip)
{
    t_uint64 word;
    int ret = fetch_op(ip, &word);
    if (ret != 0)
        return ret;
    ret = instr_ufm(word);
    return ret;
}

// ============================================================================

static void do_ldi_ret(t_uint64 word, int is_ret)
{
    uint par = IR.parity_mask;
    uint nbar = IR.not_bar_mode;
    uint abs = IR.abs_mode;
    uint mir = IR.mid_instr_intr_fault;
    load_IR(&IR, word);
    if (is_ret) {
        // Allow turning off, but undo a change that turns on
        if (!nbar) IR.not_bar_mode = 0;
        if (!abs) IR.abs_mode = 0;
    } else {
        // Undo changes
        IR.not_bar_mode = nbar;
        IR.abs_mode = abs;
    }
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != ABSOLUTE_mode) {   // BUG: check priv mode
        // Undo changes
        IR.parity_mask = par;
        IR.mid_instr_intr_fault = mir;
    }
    if (opt_debug) {
        t_uint64 ir;
        save_IR(&ir);
        log_msg(DEBUG_MSG, "OPU::ldi", "IR: %s\n", bin2text(ir, 18));
    }
}

// ============================================================================

static void long_right_shift(t_uint64 *hip, t_uint64 *lop, int n, int is_logical)
{
    const char *moi = (is_logical) ? "OPU::lrl" : "OPU::lrs";

    flag_t init_neg = bit36_is_neg(*hip);
    if (opt_debug) log_msg(DEBUG_MSG, moi, "Shift %012llo,%012llo of %d bits.\n", *hip, *lop, n);

    if (n >= 72) {
        log_msg(NOTIFY_MSG, moi, "Shift of %d bits.\n", n);
        if (!is_logical && init_neg) {
            *hip = MASK36;
            *lop = MASK36;
        } else {
            *hip = 0;
            *lop = 0;
        }
    } else if (n != 0) {
        if (n <= 36) {
            *lop >>= n;
            *lop = setbits36(*lop, 0, n, *hip);
            *hip >>= n;
            if (!is_logical && init_neg)
                *hip = setbits36(*hip, 0, min(n, 36), ~0);
        } else {
            *lop = *hip >> (n - 36);
            if (!is_logical && init_neg && n != 36)
                *lop = setbits36(*lop, 0, n-36, ~0);
            *hip = (!is_logical && init_neg) ? MASK36 : 0;
        }
        // *hip >>= n;  gcc 4.2.4 didn't complain under c99 but flakes if shift moves more bits than source has
    }
    if (opt_debug) log_msg(DEBUG_MSG, moi, "Debug: Result:  %012llo,%012llo\n", *hip, *lop);
}

// ============================================================================

static void rsw_get_port_config(int low_port)
    // low_port should be zero for ports {A,B,C,D} or 4 for ports {E,F,G,H}
{

    extern cpu_ports_t cpu_ports;

    // ADR:3, c, d, e, MEM:3
    // ADR: addr assignment; c: port enabled; d: sys init enabled; interlace enabled; coded mem size xxxb
    reg_A = 0;
    for (int port = low_port; port < low_port + 4; ++port) {
        int enabled = cpu_ports.ports[port] >= 0;   // port enabled?
#if 0
        int adr = 0; // Only one SCU, so ADR is 000b for lowest memory
#else
        int adr = enabled ? cpu_ports.ports[port] & 7 : 0;  // 3 bit port assignment
        // rsw_util$port_info claims base addr is: port-assignement * size/1024
#endif
        int byte = adr << 6;
        byte |= enabled << 5;   // port enabled?
        byte |= enabled << 4;   // sys init enabled? -- yes because we only have one CPU
        byte |= enabled << 3;   // interlace enabled
        if (enabled) {
            // BUG/TODO: we always claim the max possible memory size
            // Note that AL-39 is incorrect.  The table of sizes in section three under
            // the description of configuration switch data is obsolete (after an apparently
            // required hardware FCO).   A value of seven used to mean the maximum possible
            // memory before MR10.0 and the FCO.  Now 7 means 256K while 2 means 4MW.  See
            // comments in rsw_util.pl1.
            byte |= 2;  // 4MW -- AL39 is incorrect; see dps_mem_size_table in rsw_util.pl1
        }
        if (enabled)
            log_msg(INFO_MSG, "OPU::rsw", "CPU port '%c' is enabled for SCU port %d.\n", 'A' + port, adr);
        else
            log_msg(INFO_MSG, "OPU::rsw", "CPU port '%c' is not enabled.\n", 'A' + port);
        reg_A = (reg_A << 9) | byte;
    }
}

// ============================================================================

// Returns -1, 0, or 1

#if 0
static int cmp_fract(t_uint64 x, t_uint64 y)
{
    if (x == y)
        return 0;

    flag_t x_neg = bit36_is_neg(x);
    flag_t y_neg = bit36_is_neg(y);

    if (x_neg)  {
        if (y_neg) {
            // The larger absolute value is the more negative fraction
            return (x > y) ? -1 : 1;
        } else
            return -1;
    } else {
        if (y_neg)
            return 1;
        return (x > y) ? 1 : -1;
    }
}
#endif

// ============================================================================

static void equalize_mantissas(
    const char* op,
    t_uint64 *a_mantp, uint8 a_exp,
    t_uint64 *b_mantp, uint8 b_exp,
    int* new_exp)
{
    t_uint64 a_mant = *a_mantp;
    t_uint64 b_mant = *b_mantp;

    log_msg(INFO_MSG, op, "1st mantissa %012llo, E = %d\n", a_mant, a_exp);
    log_msg(INFO_MSG, op, "2nd mantissa %012llo, E = %d\n", b_mant, b_exp);

    double a_val = multics_to_double(a_mant << 8, 0, 0, 1);
    double b_val = multics_to_double(b_mant << 8, 0, 0, 1);
    log_msg(INFO_MSG, op, "1st arg: %.4f*2^%d, 2nd arg: %.4f*2^%d\n", a_val, a_exp, b_val, b_exp);
    
    // treat fractions as sign-magnitude for shifting
    int b_neg = bit36_is_neg(b_mant);
    if (b_neg)
        b_mant = negate36(b_mant);
    int a_neg = bit36_is_neg(a_mant);
    if (a_neg)
        a_mant = negate36(a_mant);

    // right shift
    int exp;    // debug
    if ((int8) b_exp < (int8) a_exp) {
        // b has the smaller exponent
        exp = (int8) a_exp;
        int n = (int8) a_exp - (int8) b_exp;
        if (n >= 36)
            b_mant = 0;
        else
            b_mant >>= n;
    } else {
        // a has the smaller exponent
        exp = (int8) b_exp;
        int n = (int8) b_exp - (int8) a_exp;
        if (n >= 36)
            a_mant = 0;
        else
            a_mant >>= n;
    }
    if (b_mant == 0)
        b_neg = 0;
    if (a_mant == 0)
        a_neg = 0;

#if 0
    // fcmp
    IR.zero = a_neg == b_neg && a_mant == b_mant;
    if (IR.zero)
        IR.neg = 0;
    else if (a_neg && ! b_neg)
        IR.neg = 1;
    else if (!a_neg && b_neg)
        IR.neg = 0;
    else if (!a_neg && ! b_neg)
        IR.neg = a_mant < b_mant;
    else
        IR.neg = a_mant > b_mant;
#endif
    
    b_val = multics_to_double(b_mant << 8, 0, 0, 1);
    a_val = multics_to_double(a_mant << 8, 0, 0, 1);

    *new_exp = exp;
    *a_mantp = a_mant;
    *b_mantp = b_mant;

    if (a_neg == b_neg && a_mant == b_mant)
        log_msg(INFO_MSG, op, "Align: Exp is %d; both mantissas %f\n", exp, a_val);
    else
        log_msg(INFO_MSG, op, "Align: Exp is %d; 1st mantissa %f, 2nd mantissa %f\n", exp, a_val, b_val);

}

// ============================================================================

static void equalize_mantissas72(
    const char* op,
    t_uint64* a_mant_hip, t_uint64* a_mant_lop, uint8 a_exp,
    t_uint64* b_mant_hip, t_uint64* b_mant_lop, uint8 b_exp,
    int* new_exp)
{
    t_uint64 a_mant_hi = *a_mant_hip;
    t_uint64 a_mant_lo = *a_mant_lop;
    t_uint64 b_mant_hi = *b_mant_hip;
    t_uint64 b_mant_lo = *b_mant_lop;

    log_msg(INFO_MSG, op, "1st mantissa { %012llo, %012llo }, exp = %d\n", a_mant_hi, a_mant_lo, a_exp);
    log_msg(INFO_MSG, op, "2nd mantissa { %012llo, %012llo }, exp = %d\n", b_mant_hi, b_mant_lo, b_exp);

    double a_val = multics_to_double(a_mant_hi << 8, 0, 0, 1);
    double b_val = multics_to_double(b_mant_hi << 8, 0, 0, 1);
    log_msg(INFO_MSG, op, "1st arg: %.4f*2^%d, 2nd arg: %.4f*2^%d\n", a_val, a_exp, b_val, b_exp);
    
    // treat fractions as sign-magnitude for shifting
    int b_neg = bit36_is_neg(b_mant_hi);
    if (b_neg)
        b_mant_hi = negate36(b_mant_hi);
    int a_neg = bit36_is_neg(a_mant_hi);
    if (a_neg)
        a_mant_hi = negate36(a_mant_hi);

    // 72-bit right shift
    int exp;    // debug
    if ((int8) b_exp < (int8) a_exp) {
        // b has the smaller exponent
        exp = (int8) a_exp;
        int n = (int8) a_exp - (int8) b_exp;
        long_right_shift(&b_mant_hi, &b_mant_lo, n, 1);
    } else {
        // a has the smaller exponent
        exp = (int8) b_exp;
        int n = (int8) b_exp - (int8) a_exp;
        long_right_shift(&a_mant_hi, &a_mant_lo, n, 1);
    }

    if (b_mant_hi == 0 && b_mant_lo == 0)
        b_neg = 0;
    if (a_mant_hi == 0 && a_mant_lo == 0)
        a_neg = 0;

    b_val = multics_to_double(b_mant_hi, b_mant_lo, 0, 1);
    a_val = multics_to_double(a_mant_hi, a_mant_lo, 0, 1);

    *new_exp = exp;
    *a_mant_hip = a_mant_hi;
    *a_mant_lop = a_mant_lo;
    *b_mant_hip = b_mant_hi;
    *b_mant_lop = b_mant_lo;

    if (a_neg == b_neg && a_mant_hi == b_mant_hi && a_mant_lo == b_mant_lo)
        log_msg(INFO_MSG, op, "Align: Exp is %d; both mantissas %f\n", exp, a_val);
    else
        log_msg(INFO_MSG, op, "Align: Exp is %d; 1st mantissa %f, 2nd mantissa %f\n", exp, a_val, b_val);

}

// ============================================================================
