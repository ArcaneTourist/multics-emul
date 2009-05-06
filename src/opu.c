/*
    opu.c -- implements instruction execution.

    Called by routines in hw6180_cpu.c.

    Some of the simpler mathematical and logical functions are implemented
    here, but more complicated math functions are provided in math.h.
    Makes heavy use of the addressing functions in apu_basic.c and
    opu_eis_mw.c

    BUG: Unclear if we need to sometimes set IR based on 18 bit values when
    an 18-bit ",dl" operand is used.   This is partially implemented when
    constant do_18bit_math is set to true.

*/

#include "hw6180.h"
#include <ctype.h>  // for isprint

int do_18bit_math;  // diag tape seems to want this, probably inappropriately

#define XED_NEW 1

// ============================================================================

extern uint32 sim_brk_summ;

static inline t_uint64 lrotate36(t_uint64 x, unsigned n);
static inline void lrotate72(t_uint64* ap, t_uint64* bp, unsigned n);
static inline int32 negate18(t_uint64 x);
static inline t_int64 negate36(t_uint64 x);
static inline int negate72(t_uint64* a, t_uint64* b);
// static int32 sign18(t_uint64 x);
static t_int64 sign36(t_uint64 x);
static inline uint min(uint a, uint b);
static inline uint max(uint a, uint b);
static inline uint max3(uint a, uint b, uint c);

static int do_op(instr_t *ip);
static void scu2words(t_uint64 *words);
static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);
static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add18(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add72(t_uint64 a, t_uint64 b, t_uint64* dest1, t_uint64* dest2, int is_unsigned);
static int do_epp(int epp);
static int do_eawp(int n);
static int do_epbp(int n);
static int do_easp(int n);
static int do_an_op(instr_t *ip);   // todo: hack, fold into do_op
static void spri_to_words(int reg, t_uint64* word0p, t_uint64 *word1p);
static int do_spri(int n);
static int op_mlr(const instr_t* ip);
static int op_tct(const instr_t* ip, int fwd);
static int op_mvt(const instr_t* ip);
static int op_cmpc(const instr_t* ip);
// static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs);    // BUG: temp
static int do_spbp(int n);
static int op_csl(const instr_t* ip);
static int op_scm(const instr_t* ip);

static uint saved_tro;

// BUG: move externs to hdr file
extern switches_t switches;
extern int scu_cioc(t_uint64 addr);
extern int scu_set_mask(t_uint64 addr, int port);
extern int scu_get_mask(t_uint64 addr, int port);
extern int scu_set_cpu_mask(t_uint64 addr);
extern int scu_get_calendar(t_uint64 addr);
extern int activate_timer();

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
#if 0
        if (cpu.orig_mode_BUG != get_addr_mode())
            log_msg(NOTIFY_MSG, "OPU::fetch-op", "addr mode is changed, but operand is a constant.\n");
#endif
        *wordp = TPR.value;
        return 0;
    }
#if 0
    if (cpu.orig_mode_BUG != get_addr_mode())
        log_msg(NOTIFY_MSG, "OPU::fetch-op", "fetching in changed mode\n");
#endif
    return fetch_word(TPR.CA, wordp);
}


// ============================================================================

static int do_op(instr_t *ip)
{
    // Wrapper for do_an_op() with detection of change in address mode (for debugging).

    //do_18bit_math = (switches.FLT_BASE != 2); // diag tape seems to want this, probably inappropriately
    do_18bit_math = 0;

    addr_modes_t orig_mode = get_addr_mode();
#if 0
    cpu.orig_mode_BUG = orig_mode;
#endif
    uint orig_ic = PPR.IC;
    int ret = do_an_op(ip);
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
            log_msg(WARN_MSG, "OPU", "Unknown instruction 0172(1) allegedly 'ldo'.  Ignoring instruction.\n");
            cancel_run(STOP_WARN);
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
                if (TPR.TSR ==  0427) {
                    // BUG: horrid hack to allow: eppbp =its(-2,2),*
                    log_msg(WARN_MSG, "OPU", "Disabling faults for special case instr.\n");
                    fault_gen_no_fault = 1;
                    addr_mod(ip);       // note that ip == &cu.IR
                    fault_gen_no_fault = 0;
                } else {
                    addr_mod(ip);       // note that ip == &cu.IR
#if 0
                    if (cpu.orig_mode_BUG != get_addr_mode())
                        log_msg(NOTIFY_MSG, "OPU::do-an-op", "Back from addr_mod()\n");
#endif
                }
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
    } else {
        switch (op) {
            case opcode1_a4bd + 0:  // BUG: this construct just used to allow counting how many opcodes are implemented
            case opcode1_a6bd + 0:
            case opcode1_a9bd + 0:
                addr_mod_eis_addr_reg(ip);
                break;
            case opcode1_epp1 + 0:
            case opcode1_epp3 + 0:
            case opcode1_epp5 + 0:
            case opcode1_epp7 + 0:
                if (TPR.TSR ==  0427) {
                    // BUG: horrid hack to allow: eppbp =its(-2,2),*
                    log_msg(WARN_MSG, "OPU", "Disabling faults for special case instr.\n");
                    fault_gen_no_fault = 1;
                    addr_mod(ip);       // note that ip == &cu.IR
#if 0
                    if (cpu.orig_mode_BUG != get_addr_mode())
                        log_msg(NOTIFY_MSG, "OPU::do-an-op", "Back from addr_mod()\n");
#endif
                    fault_gen_no_fault = 0;
                } else
                    addr_mod(ip);       // note that ip == &cu.IR
#if 0
                    if (cpu.orig_mode_BUG != get_addr_mode())
                        log_msg(NOTIFY_MSG, "OPU::do-an-op", "Back from addr_mod()\n");
#endif
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
#if 0
                if (cpu.orig_mode_BUG != get_addr_mode())
                    log_msg(NOTIFY_MSG, "OPU::do-an-op", "Back from addr_mod()\n");
#endif
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

            // opcode0_lcx0 .. lcx7 unimplemented

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
                if (ret == 0) {
                    uint par = IR.parity_mask;
                    uint nbar = IR.not_bar_mode;
                    uint abs = IR.abs_mode;
                    load_IR(&IR, word);
                    IR.not_bar_mode = nbar;
                    IR.abs_mode = abs;
                    addr_modes_t addr_mode = get_addr_mode();
                    if (addr_mode != ABSOLUTE_mode) {   // BUG: check priv mode
                        IR.parity_mask = par;
                        IR.mid_instr_intr_fault = 0;
                    }
                    if (opt_debug) {
                        t_uint64 ir;
                        save_IR(&ir);
                        log_msg(DEBUG_MSG, "OPU::ldi", "IR: %s\n", bin2text(ir, 18));
                    }
                }
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
            // stba unimplemented
            // stbq unimplemented
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

            // opcode0_stcd unimplemented
            case opcode0_sti: {
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                if (ret == 0) {
                    t_uint64 ir;
                    uint tro = IR.tally_runout;
                    IR.tally_runout = saved_tro;
                    save_IR(&ir);
                    IR.tally_runout = tro;
                    word = setbits36(word, 18, 18, ir);
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
                int init_neg = bit36_is_neg(reg_A);
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
                int init_neg = bit36_is_neg(reg_A);
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                lrotate72(&reg_A, &reg_Q, n);
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_lls: {     // Long left shift
                int init_neg = bit36_is_neg(reg_A);
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
                    log_msg(NOTIFY_MSG, "OPU::lls", "Shift of %d bits.\n", n);
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
                    log_msg(NOTIFY_MSG, "OPU::lls", "BUG Fix: Shift of %012llo,%012llo by %d bits\n", asv, qsv, n);
                    log_msg(NOTIFY_MSG, "OPU::lls", "Prior result was: %012llo,%012llo\n", a, q);
                    log_msg(NOTIFY_MSG, "OPU::lls", "Fixed result is:  %012llo,%012llo\n", reg_A, reg_Q);
                }

                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }

            case opcode0_lrl: {     // long right logical
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                int init_neg = bit36_is_neg(reg_A);
                log_msg(NOTIFY_MSG, "OPU::lrl", "Debug: Shift AQ %012llo,%012llo of %d bits.\n", reg_A, reg_Q, n);  // BUG: temp
                if (n >= 72) {
                    log_msg(NOTIFY_MSG, "OPU::lrl", "Shift of %d bits.\n", n);
                    if (init_neg) {
                        reg_A = MASK36;
                        reg_Q = MASK36;
                    } else {
                        reg_A = 0;
                        reg_Q = 0;
                    }
                } else if (n != 0) {
                    if (n <= 36) {
                        reg_Q >>= n;
                        reg_Q = setbits36(reg_Q, 0, n, reg_A);
                    } else {
                        reg_Q = reg_A >> (n - 36);
                    }
                    reg_A >>= n;
                }
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = init_neg;
                log_msg(NOTIFY_MSG, "OPU::lrl", "Debug: Result:  %012llo,%012llo\n", reg_A, reg_Q); // BUG: temp
                return 0;
            }

            case opcode0_lrs: {     // Long right shift
                unsigned n = TPR.CA & 0177; // bits 11..17 of 18bit CA
                int init_neg = bit36_is_neg(reg_A);
                // log_msg(NOTIFY_MSG, "OPU::lrs", "Debug: Shift AQ %012llo,%012llo of %d bits.\n", reg_A, reg_Q, n);
                if (n >= 72) {
                    log_msg(NOTIFY_MSG, "OPU::lrs", "Shift of %d bits.\n", n);
                    if (init_neg) {
                        reg_A = MASK36;
                        reg_Q = MASK36;
                    } else {
                        reg_A = 0;
                        reg_Q = 0;
                    }
                } else if (n != 0) {
                    if (n <= 36) {
                        reg_Q >>= n;
                        reg_Q = setbits36(reg_Q, 0, n, reg_A);
                    } else {
                        reg_Q = reg_A >> (n - 36);
                        if (init_neg && n != 36)
                            reg_Q = setbits36(reg_Q, 0, n-36, ~0);
                    }
                    reg_A >>= n;
                    if (init_neg)
                        reg_A = setbits36(reg_A, 0, min(n, 36), ~0);
                }
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = init_neg;
                // log_msg(NOTIFY_MSG, "OPU::lrs", "Debug: Result:  %012llo,%012llo\n", reg_A, reg_Q);
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
                int init_neg = bit36_is_neg(reg_Q);
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
                int init_neg = bit36_is_neg(reg_Q);
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

            // adaq unimplemented

            case opcode0_adl: { // Add low to AQ
                t_uint64 word1, word2;
                int ret = fetch_word(TPR.CA, &word2);
                if (ret == 0) {
                    // BUG: ignoring 18bit math
                    word1 = bit36_is_neg(word2) ? MASK36 : 0;
                    ret = add72(word1, word2, &reg_A, &reg_Q, 0);
                }
                return ret;
            }

            case opcode0_adla: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    t_uint64 a = reg_A;
                    reg_A += word;
                    if ((IR.carry = reg_A & MASK36) != 0)
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
                    if ((IR.carry = reg_Q & MASK36) != 0)
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
                    if ((IR.carry = word & MASK18) != 0)
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
#else
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
                    }
#endif
                }
                return ret;
            }
            case opcode0_sbaq: {    // Subtract from AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);
                if (ret == 0)
                    // BUG: Ignoring 18bit math
                    if ((ret = negate72(&word1, &word2)) == 0)
                        ret = add72(word1, word2, &reg_A, &reg_Q, 0);
                return ret;
            }
            // sbla unimplemented
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
            //ssa unimplemented
            //ssq unimplemented
            //ssx0 ..ssx7 unimplemented
            //swca unimplemented
            //swcq unimplemented

            // opcode_mpf unimplemented -- multiple fraction

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
                        reg_Q = (IR.neg) ? negate36(reg_Q) : reg_Q; // magnitude, absolute value
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

            // opcode0_divf unimplemented -- divide fraction

            case opcode0_neg:
                if (reg_A == 0) {
                    IR.zero = 1;
                } else {
                    IR.zero = 0;
                    reg_A = negate36(reg_A);
                    IR.neg = bit36_is_neg(reg_A);
                    IR.overflow = reg_A == ((t_uint64)1<<35);
                    // BUG: Should we fault?  Maximum negative number can't be negated, but AL39 doesn't say to fault
                }
                return 0;
            // opcode0_negl unimplemented
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
            // ansq unimplemented

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
                    IR.zero = word == 0;
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
                    IR.zero = word == 0;
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
            // opcode0_erx0 .. erx7 unimplemented

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
                    reg_E = setbits36(reg_E, 0, 8, lo);
                    reg_A = 0;  // zero bits 28..35
                    reg_A = setbits36(reg_A, 0, 28, hi);
                    reg_Q = 0;  // bits 36..71 of AQ pair
                    IR.zero = reg_A == 0 && reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }

            // dfst unimplemented
            // dfstr unimplemented
            // fst unimplemented
            // fstr unimplemented
            // dfad unimplemented
            // dufa unimplemented
            // fad unimplemented -- floating add
            // ufa unimplemented -- unnormalized floating add
            // dfsb unimplemented -- double precision floating subtract
            // dufs unimplemented -- double precision unnormalized floating subtract
            // fsb unimplemented -- floating subtract
            // ufs unimplemented -- unnormalized floating subtract
            // dfmp unimplemented -- double-precision floating multiply
            // dufmp unimplemented -- double-precision unnormalized floating multiply
            // fmp unimplemented -- floating multiply
            // ufm unimplemented -- unnormalized floating multiply
            // dfdi unimplemented -- double precision floating divide inverted
            // dfdv unimplemented -- double precision floating divide
            // fdi unimplemented -- floating divide inverted
            // fdv unimplemented -- floating divide
            // fneg unimplemented -- floating negate
            // fno unimplemented -- floating normalize
            // dfrd unimplemented -- double precision floating round
            // frd unimplemented -- floating round
            // dfcmg unimplemented -- double precision floating compare magnitude
            // dfcmp unimplemented -- double precision floating compare
            // fcmg unimplemented -- floating compare magnitude
            // cmp unimplemented -- floating compare
            
            // ade unimplemented
            // fszn unimplemented
            // lde unimplemented
            // ste unimplemented
            
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
            
            // opcode0_ret unimplemented

            case opcode0_rtcd: {
                // BUG -- Need to modify APU to check for opcode==rtcd && POA flag
                log_msg(NOTIFY_MSG, "OPU::rtcd", "Some access checks are not implemented.\n");

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
                // would not failed to fetch the y-pair.  Thus, we'd be putting garbage in the IC.
                if (! SDWp || fault_check_group(6)) {
                    log_msg(WARN_MSG, "OPU::rtcd", "Failed to get SDW and/or a fault occured even though operand fetch was successful.\n");
                    cancel_run(STOP_BUG);
                    return 1;
                }

                PPR.PSR = getbits36(word0, 3, 15);
                PPR.PRR = max3(getbits36(word0, 18, 3), TPR.TRR, SDWp->r1);
                PPR.IC = word1 >> 18;
                PPR.P = priv;
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
                int n = (op & 03) + (op >= opcode0_adwp4) ? 4 : 0;
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

            // rccl unimplemented
            // drl unimplemented

            case opcode0_xec: {
                // todo: combine with xec
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xec invokes xec
                // todo: handle rpd repeats
                // BUG: EIS multiword instructions may not be handled
                t_uint64 word0;
                instr_t IR;
                if (fetch_instr(TPR.CA, &IR) != 0) {
                    log_msg(NOTIFY_MSG, "OPU::opcode::xec", "fetch instr: error or fault\n");
                    return 1;   // faulted
                }
                // extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "executing instr at %#o\n", TPR.CA);
                int ret;
                if ((ret = do_op(&IR)) != 0)
                    log_msg(NOTIFY_MSG, "OPU::opcode::xec", "fault or error executing instr\n");
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "finished\n");
                // -- opt_debug; -- cpu_dev.dctrl;
                return ret;
            }
            case opcode0_xed: {
#if XED_NEW
                // extern DEVICE cpu_dev; ++ opt_debug; if (! cpu_dev.dctrl) ++ cpu_dev.dctrl;
                // Load IR and IRODD, set flags, and return to the control unit
                // Much of the work here and in control_unit() is for handling edge cases like
                // xed running rpd or other edge cases but it's likely that real code always
                // uses a scu/tra pair.
                t_uint64 word0;
                t_uint64 word1;
                instr_t IR;
                uint y = TPR.CA - TPR.CA % 2;   // force even
                if (fetch_pair(y, &word0, &word1) != 0) {
                    log_msg(WARN_MSG, "OPU::opcode::xed", "fetch y-pair: error or fault\n");
                    return 1;   // faulted
                }
                decode_instr(&cu.IR, word0);
                instr_t i_tmp;
                decode_instr(&i_tmp, word1);
                if (cu.IR.opcode != (opcode0_scu << 1) || i_tmp.opcode != (opcode0_tra << 1)) {
                    log_msg(WARN_MSG, "OPU::xed", "Target y-pair at %#o is not SCU/TRA -- found opcodes %03o(%d) and %03o(%d)\n", y, cu.IR.opcode >> 1, cu.IR.opcode & 1, i_tmp.opcode >> 1, i_tmp.opcode & 1);
                }
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
                if (switches.FLT_BASE == 2) {   // true for multics, false for T&D diag tape
                    log_msg(NOTIFY_MSG, "opu::xed", "Auto Breakpoint\n");
                    cancel_run(STOP_IBKPT);
                }
                return 0;
#else
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xed invokes xed
                // todo: handle rpd repeats
                /* NOTES
                    CU has flags for
                        XDE, XDO -- exec even or odd instr for xed
                        IC -- exec odd instr of current pair
                        WI -- wait for instr fetch
                        INS-FETCH
                    CU DATA
                        RPT, RD, RPL
                */
                // Maybe load IR and IRODD, set flags, and return
                // CU would have to do execution even though cycle is set to fault
                // and check post-fault stuff
                // How the heck are the instr-pair for the XED restartable?
                // Answer must be via scu/rcu
                t_uint64 word0;
                t_uint64 word1;
                instr_t IR;
#if 0
                uint64 y;
                if (decode_ypair_addr(ip, &y)) {
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "decode addr: error or fault\n");
                    return 1;   // faulted
                }
#else
                uint y = TPR.CA - TPR.CA % 2;   // force even
#endif
                // -----------
                if (fetch_instr(y, &IR) != 0) {
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "fetch even: error or fault\n");
                    return 1;   // faulted
                }
                log_msg(DEBUG_MSG, "OPU::opcode::xed", "executing even instr at %#llo\n", y);
                if (do_op(&IR) != 0) {
                    log_msg(WARN_MSG, "OPU::opcode::xed", "fault or error executing even instr\n");
                    // BUG: no way to get to the second instr
                    return 1;
                }
                // -----------
                if (cpu.trgo) {
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "transfer instr executed, not doing odd instr\n");
                } else {
                    ++ y;
                    if (fetch_instr(y, &IR) != 0) {
                        log_msg(DEBUG_MSG, "OPU::opcode::xed", "fetch odd: error or fault\n");
                        return 1;   // faulted
                    }
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "executing odd instr at %#llo\n", y);
                    if (do_op(&IR) != 0) {
                        log_msg(WARN_MSG, "OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
#endif
                log_msg(DEBUG_MSG, "OPU::opcode::xed", "finished\n");
                break;
            }

            case opcode0_mme:
                fault_gen(mme1_fault);
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
                cu.rpts = 1;
                // AL39, page 209
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
                log_msg(NOTIFY_MSG, "OPU", "RPT instruction found\n");
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
                    log_msg(NOTIFY_MSG, "OPU::lbar", "BAR: base = %09o => %018o, bound = %09o => %018o\n",
                        BAR.base, BAR.base << 9, BAR.bound, BAR.bound << 9);
                }
                return ret;
            }

            case opcode0_lcpr: {    // load central processor reg (priv)
                int ret = 0;
                t_uint64 word;
                switch (ip->mods.single.tag) {      // no addr modifications
                    case 2:
                        ret = fetch_word(TPR.CA, &word);
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Not writing %#llo to cache mode reg.\n", word);
                        if (word != 0)
                            ret = 1;
                        break;
                    case 4:
                        ret = fetch_word(TPR.CA, &word);
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Not writing %#llo to mode reg.\n", word);
                        if (word != 0)
                            ret = 1;
                        break;
                    case 3:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "history reg zero unimplemented.\n");
                        break;
                    case 7:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "history reg setting unimplemented.\n");
                        ret = 1;
                        break;
                    default:
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Bad tag %#o\n", ip->mods.single.tag);
                        ret = 1;
                }
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
                log_msg(NOTIFY_MSG, "OPU::ldbr", "DSBR: addr=%#o, bound=%#o(%u), u=%d, stack=%#o\n",
                    cpup->DSBR.addr, cpup->DSBR.bound, cpup->DSBR.bound, cpup->DSBR.u, cpup->DSBR.stack);
                return 0;
            }
            case opcode0_ldt: { // load timer reg (priv)
                if (get_addr_mode() != ABSOLUTE_mode && ! is_priv_mode()) {
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
            // lptp unimplemented
            // lptr unimplemented
            // lra unimplemented
            // lsdp unimplemented
            // lsdr unimplemented
            // rcu unimplemented -- restore control unit
            // scpr unimplemented -- store cp register

            // scu unimplemented -- store control unit
            case opcode0_scu: {
                // AL39 says that CU data is not, in general, valid at any time except
                // when safe-stored by the first of the pair of the instructions
                // associated with the fault or interrupt.
                // On the other hand, the T&D tape expects that a ldi/scu pair results
                // in the updated IR and IC being stored.
                // The T&D tape also expects that the saved data will appropriately
                // reflect whether or not the CPU was in appending mode at the time
                // of a fault even though the subsequent scu was executed in absolute
                // mode.
                // An alternative to our implemention might be to save state after
                // every instruction except xed instructions.
                extern events_t events; // BUG: put in hdr file or hide behind an access function
                log_msg(WARN_MSG, "OPU::scu", "Not fully implemented\n");
                if (switches.FLT_BASE == 2) {
                    log_msg(WARN_MSG, "OPU::scu", "Auto breakpoint\n");
                    cancel_run(STOP_WARN);
                }
                if (! events.xed)
                    scu2words(scu_data);
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
            // sptp unimplemented
            // sptr unimplemented
            // ssdp unimplemented
            // ssdr unimplemented

            case opcode0_cams: {    // Clear Associative Memory Segments
                int ret = 0;
                int clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                if (enable == 2) {
                    cu.SD_ON = 1;
                    log_msg(NOTIFY_MSG, "OPU::cams", "Enabling SDWAM\n");
                } else if (enable == 1) {
                    cu.SD_ON = 0;
                    log_msg(NOTIFY_MSG, "OPU::cams", "Disabling SDWAM\n");
                } else if (enable == 0) {
                    log_msg(DEBUG_MSG, "OPU::cams", "Neither enable nor disable requested\n");
                } else {
                    log_msg(WARN_MSG, "OPU::cams", "Unknown enable/disable mode %06o=>%#o\n", TPR.CA, enable);
                    cancel_run(STOP_WARN);
                }
                // AL-39 says this instr can't be used in absolute mode, but the bootload tape does
                // exactly that.  Perhaps it's only enabling and disabling during appending mode that
                // is actually illegal.  There has to be some means of invalidating the cache and switching
                // into absolute mode to do so may have been deemed clumsy.
                if (enable == 1 || enable == 2)
                    if (get_addr_mode() != ABSOLUTE_mode) {
                        log_msg(ERR_MSG, "OPU::cams", "cams executed when mode is not absolute.\n");
                        // log_msg(ERR_MSG, "OPU::cams", "This should be a fault, but we're ignoring it...\n");
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

            // rmcm unimplemented -- read memory controller mask register

            case opcode0_rscr: { // priv
                // read system controller register (to AQ)
                int ret = 0;
                uint y = (TPR.CA >> 16) & 3;    // get bits one and two of 18bit CA
                uint ea = y << 15;              // and set just those bits in ea
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "CA is %#o (y0%03ox)\n", TPR.CA, (TPR.CA >> 3) & 077);
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "EA is 0%04o\n", ea);
                t_bool show_q = 1;
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    log_msg(NOTIFY_MSG, "OPU::opcode::rscr", "mode register selected\n");
                    log_msg(ERR_MSG, "OPU::opcode::rscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "sys config switches unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0020) {
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "port zero selected\n");
                    ret = scu_get_mask(TPR.CA, 0);
                } else if ((TPR.CA & ~7) == ea + 0120)
                    ret = scu_get_mask(TPR.CA, 1);
                else if ((TPR.CA & ~7) == ea + 0220)
                    ret = scu_get_mask(TPR.CA, 2);
                else if ((TPR.CA & ~7) == ea + 0320)
                    ret = scu_get_mask(TPR.CA, 3);
                else if ((TPR.CA & ~7) == ea + 0420)
                    ret = scu_get_mask(TPR.CA, 4);
                else if ((TPR.CA & ~7) == ea + 0520)
                    ret = scu_get_mask(TPR.CA, 5);
                else if ((TPR.CA & ~7) == ea + 0620)
                    ret = scu_get_mask(TPR.CA, 6);
                else if ((TPR.CA & ~7) == ea + 0720) {
                    ret = scu_get_mask(TPR.CA, 7);
                } else if ((TPR.CA & ~7) == ea + 0030) {
                    // interrupts
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "interrupts unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    ret = scu_get_calendar(TPR.CA);
                    show_q = 0;
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    ret = scu_get_calendar(TPR.CA);
                    show_q = 0;
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    log_msg(WARN_MSG, "OPU::opcode::rscr", "store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "bad argument, CA %#o\n", TPR.CA);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "A = %llo\n", reg_A);
                if (show_q)
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "Q = %llo\n", reg_Q);
                return ret;
            }

            case opcode0_rsw: { // read switches
                int low = TPR.CA & 07;
                switch(low) {
                    case 0: // unformatted; maintenance panel data switches
                        // GB61-01B reports which of the data switches should be turned on.
                        // It also discusses other switches, but these other switches are probably
                        // not considered to be part of the "data" switches.
                        reg_A = (t_uint64) 024000717200; // switches: 4, 6, 18, 19, 20, 23, 24, 25, 26, 28
                        break;
                    case 2:
#if 0
                        // from start_cpu.pl1
                        reg_A = setbits36(0, 4, 2, 0);  // 0 for L68 or DPS
                        reg_A = setbits36(reg_A, 6, 7, switches.FLT_BASE);  // 7 MSB bits of 12bit addr
                        log_msg(NOTIFY_MSG, "OPU::opcode::rsw", "Fault base in bits 6..13 is %#o=>%#o\n", switches.FLT_BASE, switches.FLT_BASE << 5);
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
                        log_msg(NOTIFY_MSG, "OPU::opcode::rsw", "function xxx%o returns A=%012llo.\n", low, reg_A);
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
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                return scu_set_cpu_mask(TPR.CA);    // don't convert via appending
            }
            // smic unimplemented
            case opcode0_sscr: { // priv
                // set system controller register (to value in AQ)
                if (get_addr_mode() != ABSOLUTE_mode && ! is_priv_mode()) {
                    log_msg(ERR_MSG, "OPU::sscr", "Not in absolute mode\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "A = %llo\n", reg_A);
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "Q = %llo\n", reg_Q);
                int ret = 0;
                uint y = (TPR.CA >> 16) & 3;    // 18bit CA
                uint ea = y << 15;
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "EA is 0%04o\n", ea);
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "CA is 0%04o (0%03o=>0%03o)\n", TPR.CA, (TPR.CA >> 3), TPR.CA & ~7);
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "mode register selected\n");
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0020) {
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "port zero selected\n");
                    ret = scu_set_mask(TPR.CA, 0);
                } else if ((TPR.CA & ~7) == ea + 0120)
                    ret = scu_set_mask(TPR.CA, 1);
                else if ((TPR.CA & ~7) == ea + 0220)
                    ret = scu_set_mask(TPR.CA, 2);
                else if ((TPR.CA & ~7) == ea + 0320)
                    ret = scu_set_mask(TPR.CA, 3);
                else if ((TPR.CA & ~7) == ea + 0420)
                    ret = scu_set_mask(TPR.CA, 4);
                else if ((TPR.CA & ~7) == ea + 0520)
                    ret = scu_set_mask(TPR.CA, 5);
                else if ((TPR.CA & ~7) == ea + 0620)
                    ret = scu_set_mask(TPR.CA, 6);
                else if ((TPR.CA & ~7) == ea + 0720) {
                    log_msg(DEBUG_MSG, "OPU::opcode::sscr", "port seven selected\n");
                    ret = scu_set_mask(TPR.CA, 7);
                } else if ((TPR.CA & ~7) == ea + 0030) {
                    // interrupts
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    // calendar
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    // calendar
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "bad argument, CA %#o\n", TPR.CA);
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
                uint addr;
                int ret;
#if 0
                if (cpu.orig_mode_BUG == ABSOLUTE_mode) {
                        (void) get_seg_addr(TPR.CA, 0, &addr);
                        reg_A = (t_uint64) TPR.CA << 12;    // upper 24 bits
                        if (addr == TPR.CA)
                            log_msg(NOTIFY_MSG, "OPU::absa", "Ignored segment portion of PR register irrelevent.  Result %#llo\n", reg_A);
                        else
                            log_msg(NOTIFY_MSG, "OPU::absa", "Ignoring segment portion of PR register -- Using %#llo instead of %#llo.\n", reg_A, (t_uint64) addr << 12);
                        ret = 0;
                } else
#endif
                {
                        if (opt_debug) log_msg(DEBUG_MSG, "OPU::absa", "Getting segment translation (DSBR.bound = %#o).\n", cpup->DSBR.bound);
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
                }
                return ret;
            }

            case opcode0_dis:
                // delay until interrupt set
                if (1 || ip->inhibit) {
                    if (ip->inhibit) {
                        log_msg(WARN_MSG, "OPU::dis", "DIS with inhibit set -- CPU now dead (but you can issue SIMH go)\n");
                        cancel_run(STOP_BUG);
                    } else {
                        log_msg(WARN_MSG, "OPU::dis", "DIS unimplemented; simply continue when ready\n");
                        cancel_run(STOP_IBKPT);
                    }
                    return 1;
                } else {
                    log_msg(WARN_MSG, "OPU::dis", "DIS unimplemented.   Continuing after history dump.\n");
                    cmd_dump_history(-1, NULL);
                    return 0;
                }


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
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                int sel_clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                if (enable == 2) {
                    cu.PT_ON = 1;
                    log_msg(NOTIFY_MSG, "OPU::camp", "Enabling PTWAM\n");
                } else if (enable == 1) {
                    cu.PT_ON = 0;
                    log_msg(NOTIFY_MSG, "OPU::camp", "Disabling PTWAM\n");
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
                return store_yblock8(TPR.CA, words);
            }

            // a4bd unimplemented -- add 4 bit displacement to addr register
            // a6bd
            case opcode1_a9bd:
                log_msg(DEBUG_MSG, "OPU::a9bd", "APU does our work for us\n");
                return 0;

            // abd unimplemented -- add bit displacement to addr register
            // awd unimplemented
            // s4bd unimplemented
            // s6bd unimplemented
            // s9bd unimplemented
            // sbd unimplemented
            // swd unimplemented

            case opcode1_cmpc:
                return op_cmpc(ip);

            // opcode1_scd unimplemented -- scan characters double
            // opcode1_scdr unimplemented -- scan characters double in reverse
            case opcode1_scm: { // scan with mask
                // extern DEVICE cpu_dev; ++opt_debug; ++ cpu_dev.dctrl;
                int ret = op_scm(ip);
                //--opt_debug; --cpu_dev.dctrl;
                return ret;
            }
            
            // opcode1_scmr unimplemented -- scan with mask in reverse

            case opcode1_tct: {
                int ret = op_tct(ip, 1);
                return ret;
            }
            case opcode1_tctr: {
                //int ret = op_unimplemented_mw(ip, op, opname, 3);
                int ret = op_tct(ip, 0);
                return ret;
            }
            case opcode1_mlr: {
                int ret = op_mlr(ip);
                return ret;
            }
            // mrl unimplemented
            // mve unimplemented -- move alphanumeric edited
            case opcode1_mvt: {
                int ret = op_mvt(ip);
                return ret;
            }
            // cmp0 .. cmp7 unimplemented -- compare numeric
            // mvn unimplemented -- move numeric
            // mvne unimplemented -- move numeric edited

            case opcode1_csl: {     // combine bit strings left
                int ret = op_csl(ip);
                return ret;
            }
            
            // csr unimplemented -- combine bit strings right
            // cmpb unimplemented -- compare bit strings
            // sztl unimplemented -- similar to csl?
            // sztr unimplemented -- similar to csr?
            // btd unimplemented --  binary to decimal convert
            // dtb unimplemented --  decimal to binary convert
            // ad2d unimplemented -- add using two decimal operands
            // ad3d unimplemented -- add using three decimal operands
            // sb2d unimplemented -- subtract using two decimal operands
            // sb3d unimplemented -- subtract using three decimal operands
            // mp2d unimplemented -- multiply using two decimal operands
            // mp2d unimplemented -- multiply using three decimal operands
            // dv2d unimplemented -- divide using two decimal operands
            // dv3d unimplemented -- divide using three decimal operands

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
    // Save current Control Unit Data in hidden temporary so a later SCU instruction running
    // in FAULT mode can save the state as it existed at the time of the fault rather than
    // as it exists at the time the scu instruction is executed.
    scu2words(scu_data);
}


static void scu2words(t_uint64 *words)
{
    memset(words, 0, 8 * sizeof(*words));

    words[0] = setbits36(0, 0, 3, PPR.PRR);
    words[0] = setbits36(words[0], 3, 15, PPR.PSR);
    words[0] = setbits36(words[0], 18, 1, PPR.P);
    words[0] = setbits36(words[0], 21, 1, cu.SD_ON);
    words[0] = setbits36(words[0], 23, 1, cu.PT_ON);
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
    words[5] = setbits36(words[5], 24, 1, cu.xdo);
    words[5] = setbits36(words[5], 30, 6, cu.CT_HOLD);
    encode_instr(&cu.IR, &words[6]);    // BUG: cu.IR isn't kept fully up-to-date
    words[7] = cu.IRODD;
}


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
    //uint wrong_s = result >> 35;
    //t_uint64 wrong_r = result;
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
    //if (signr != wrong_s) {
    //  log_msg(WARN_MSG, "OPU::add36", "Prior version had incorrect sign %#o instead of %o -- %012llo (%lld) + %012llo (%lld) ==> %012llo ==> %012llo (%lld => %lld)\n",
    //      wrong_s, signr, a, sign36(a), b, sign36(b), wrong_r, result, sign36(result), sign36(result & MASK36));
    //}
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

    return 0;
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

static int add72(t_uint64 a, t_uint64 b, t_uint64* dest1, t_uint64* dest2, int is_unsigned)
{
    t_uint64 word1, word2;

    uint sign1 = a >> 35;
    uint sign2 = *dest1 >> 35;

    t_uint64 lo = b + *dest2;
    int lo_carry = (lo >> 35);
    lo &= MASK36;

    t_uint64 hi = a + *dest1;
    if (lo_carry)
        ++ hi;
    IR.carry = (hi >> 36) != 0;
    if (IR.carry)
        hi &= MASK36;

    IR.zero = lo == 0 && hi == 0;
    int signr = hi >> 35;
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

static inline int32 negate18(t_uint64 x)
{
    // overflow not detected
    if (bit18_is_neg(x))
        return ((~x & MASK18) + 1) & MASK18;    // todo: only one mask needed?
    else
        return (- x) & MASK18;
}

static inline t_int64 negate36(t_uint64 x)
{
    // overflow not detected
    if (bit36_is_neg(x))
        return ((~x & MASK36) + 1) & MASK36;    // todo: only one mask needed?
    else
        return (- x) & MASK36;
}

static inline int negate72(t_uint64* a, t_uint64* b)
{
    // overflow not detected
    *a = (~ *a) & MASK36;
    *b = (~ *b) & MASK36;
    ++ *b;
    if (*b & MASK36) {
        *b &= MASK36;
        ++ *a;
        *a = *a & MASK36;
    }
    return 0;
}

// ============================================================================

#if 0
static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        // log_msg(DEBUG_MSG, "OPU::sign18", "%#llo => %#o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}
#endif

static t_int64 sign36(t_uint64 x)
{
    if (bit36_is_neg(x)) {
        t_int64 r = - (((t_int64)1<<36) - (x&MASK36));
        return r;
    }
    else
        return x;
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
    AR_PR[epp].AR.charno = AR_PR[epp].PR.bitno / 9;
    AR_PR[epp].AR.bitno = AR_PR[epp].PR.bitno % 9;
    char buf[20];
    sprintf(buf, "OPU::epp%d", epp);;
    log_msg(DEBUG_MSG, buf, "PR[%o]=TPR -- rnr=%#o, snr=%#o, wordno=%o, bitno=%o\n", epp, AR_PR[epp].PR.rnr, AR_PR[epp].PR.snr, AR_PR[epp].wordno, AR_PR[epp].PR.bitno);
    return 0;
}

// ============================================================================

extern flag_t fault_gen_no_fault;   // BUG

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


# if 0
static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs)
{
    // This function was only useful for turning unimplemented EIS muli-word instructions
    // into no-ops while debugging other issues.
    char moi[20];
    sprintf(moi, "OPU::%s", opname);

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&mf2, &desc2));
    if (nargs == 3) {
        t_uint64 word3;
        if (fetch_word(PPR.IC + 3, &word3) != 0)
            return 1;
        log_msg(DEBUG_MSG, moi, "word3 = %012llo\n", word3);
    }

    PPR.IC += nargs + 1;
    cpu.irodd_invalid = 1;
    log_msg(WARN_MSG, moi, "EIS multi-word opcode %s unimplemented.\n", opname);
    cancel_run(STOP_BUG);
    return 1;
}
#endif

// ============================================================================

static int op_mlr(const instr_t* ip)
{
    const char* moi = "OPU::mlr";

    // BUG: Detection of GBCD overpunch is not done

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&mf2, &desc2));

    int ret = 0;

    while (desc2.n > 0) {
        uint nib;
        if (desc1.n == 0)
            nib = fill & MASKBITS(desc2.nbits);
        else
            if (get_eis_an(&ip->mods.mf1, &desc1, &nib) != 0) { // must fetch when needed
                ret = 1;
                break;
            }
        if (put_eis_an(&mf2, &desc2, nib) != 0) {   // must fetch/store when needed
            ret = 2;
            break;
        }
    }
    if (ret == 0) {
        IR.truncation = desc1.n != 0;
        if (IR.truncation && t) {
            fault_gen(overflow_fault);  // truncation
            ret = 1;
        }
    }
    // write unsaved data (if any)
    if (ret < 2)
        if (save_eis_an(&mf2, &desc2) != 0)
            ret = 2;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");

    PPR.IC += 3;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret != 0;
}

// ============================================================================

static int get_table_char(uint addr, uint index, uint *charp)
{
    // index &= MASKBITS(9);
    addr += index / 4;      // 9 bit entries
    t_uint64 word;
    if (fetch_abs_word(addr, &word) != 0) {         // BUG: assumes entire table fits in same page
        // log_msg(WARN_MSG, "OPU::get-table-char", "Error fetching from addr %#o\n", addr);
        return 1;
    }
    *charp = getbits36(word, (index % 4) * 9, 9);
    return 0;
}

static int op_tct(const instr_t* ip, int fwd)
{
    const char* moi = (fwd) ? "OPU::tct" : "OPU::tctr";

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, NULL, &word2, NULL, &word3) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    uint addr2, addr3;
    if (get_eis_indir_addr(word2, &addr2) != 0) {
        log_msg(NOTIFY_MSG, moi, "Problem reading address operand.\n");
        return 1;
    }
    if (get_eis_indir_addr(word3, &addr3) != 0) {
        log_msg(NOTIFY_MSG, moi, "Problem reading address operand.\n");
        return 1;
    }
    log_msg(DEBUG_MSG, moi, "table addr: %#o\n", addr2);
    log_msg(DEBUG_MSG, moi, "result addr: %#o\n", addr3);

    uint n = desc1.n;
    while (desc1.n > 0) {
        log_msg(DEBUG_MSG, moi, "Remaining length %d\n", desc1.n);
        uint m;
        if (fwd) {
            if (get_eis_an(&ip->mods.mf1, &desc1, &m) != 0)
                return 1;
        } else
            if (get_eis_an_rev(&ip->mods.mf1, &desc1, &m) != 0) {
                //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
                return 1;
            }
        uint t;
        if (get_table_char(addr2, m, &t) != 0) {
            log_msg(WARN_MSG, moi, "Unable to read table\n");
            //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
            return 1;
        }
        if (t != 0) {
            int idx = (fwd) ? n - desc1.n - 1 : desc1.n;    // reverse scan also stores i-1 not N1-i !
            int i = n - desc1.n - 1;
            log_msg(DEBUG_MSG, moi, "Index %d: found non-zero table entry %#o for table index %#o, from offset %d in the str.\n", i, t, m, idx);
            t_uint64 word = (t_uint64) t << 27;
            word = setbits36(word, 12, 24, i);
            if (store_abs_word(addr3, word) != 0) {
                //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
                return 1;
            }
            IR.tally_runout = 0;
            PPR.IC += 4;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
            return 0;
        }
    }

    //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
    t_uint64 word = setbits36(0, 12, 24, n);
    if (store_abs_word(addr3, word) != 0)
        return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_IBKPT);

    IR.tally_runout = 1;
    log_msg(DEBUG_MSG, moi, "No non-zero entry found; setting TRO.\n");
    log_msg(NOTIFY_MSG, moi, "finished.\n");
    PPR.IC += 4;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return 0;
}

// ============================================================================

static int op_mvt(const instr_t* ip)
{
    const char* moi = "OPU::mvt";

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, &word3) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);
    uint addr3;
    if (get_eis_indir_addr(word3, &addr3) != 0)
        return 1;

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&mf2, &desc2));
    log_msg(DEBUG_MSG, moi, "translation table at %#llo => %#o\n", word3, addr3);

    int ret = 0;

    // uint n = desc1.n;
    while (desc2.n > 0) {
        uint m;
        if (desc1.n == 0)
            m = fill & MASKBITS(desc2.nbits);
        else
            if (get_eis_an(&ip->mods.mf1, &desc1, &m) != 0) {
                ret = 1;
                break;
            }
        uint t;
        if (get_table_char(addr3, m, &t) != 0) {
            log_msg(WARN_MSG, moi, "Unable to read table\n");
            ret = 1;
            break;
        }
        if (put_eis_an(&mf2, &desc2, t) != 0) {
            ret = 1;
            break;
        }
    }

    if (ret == 0) {
        IR.truncation = desc1.n != 0;
        if (IR.truncation && t) {
            fault_gen(overflow_fault);  // truncation
            ret = 1;
        }
    }
    // write unsaved data (if any)
    if (ret == 0)
        if (save_eis_an(&mf2, &desc2) != 0)
            return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    PPR.IC += 4;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret;
}

// ============================================================================

static int op_cmpc(const instr_t* ip)
{
    const char* moi = "OPU::cmpc";

    uint fill = ip->addr >> 9;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    word2 = setbits36(word2, 21, 2, desc1.ta);  // force (ignored) type of mf2 to match mf1
    parse_eis_alpha_desc(word2, &mf2, &desc2);

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&mf2, &desc2));

    int ret = 0;

    uint nib1, nib2;
    while (desc1.n > 0 || desc2.n > 0) {
        if (desc1.n == 0)
            nib1 = fill & MASKBITS(desc1.nbits);
        else
            if (get_eis_an(&ip->mods.mf1, &desc1, &nib1) != 0) {    // must fetch when needed
                ret = 1;
                break;
            }
        if (desc2.n == 0)
            nib2 = fill & MASKBITS(desc2.nbits);
        else
            if (get_eis_an(&mf2, &desc2, &nib2) != 0) { // must fetch when needed
                ret = 1;
                break;
            }
        if (nib1 < nib2) {
            IR.zero = 0;
            IR.carry = 0;
            break;
        } else if (nib1 > nib2) {
            IR.zero = 0;
            IR.carry = 1;
            break;
        }
    }
    if (ret == 0 && nib1 == nib2) {
        IR.zero = 1;
        IR.carry = 1;
    }

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    PPR.IC += 3;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret;
}

// ============================================================================

static int op_scm(const instr_t* ip)
{
    const char* moi = "OPU::scm";

    cpu.irodd_invalid = 1;

    uint mask = ip->addr >> 9;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(NOTIFY_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, &word3) != 0)
        return 1;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    word2 = setbits36(word2, 21, 2, desc1.ta);  // force (ignored) type of desc2 to match desc1
    //word2 = setbits36(word2, 24, 12, 1);  // force (ignored) count of desc2 to one
    parse_eis_alpha_desc(word2, &mf2, &desc2);
    desc2.n = 1;
    // eis_mf_t mf3 = { 0, 0, 1, 0};
    uint y3;
    if (get_eis_indir_addr(word3, &y3) != 0)
            return 1;
    //uint y3 = word3 >> 18;
    //uint y3_indir = (word3 & 0100) != 0;
    //uint y3_reg = word3 & 017;
    
    log_msg(NOTIFY_MSG, moi, "mask = %03o\n", mask);
    log_msg(NOTIFY_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&ip->mods.mf1, &desc1));
    log_msg(NOTIFY_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&mf2, &desc2));
    log_msg(NOTIFY_MSG, moi, "y3: %06o\n", y3);

    int ret = 0;

    uint test_nib;
    log_msg(DEBUG_MSG, moi, "Getting test char\n");
    if (get_eis_an(&mf2, &desc2, &test_nib) != 0) {
        return 1;
    }
    if (isprint(test_nib))
        log_msg(NOTIFY_MSG, moi, "test char: %03o '%c'\n", test_nib, test_nib);
    else
        log_msg(NOTIFY_MSG, moi, "test char: %03o\n", test_nib);
    uint n = desc1.n;
    uint i;
    for (i = 0; i < n; ++i) {
        uint nib;
        if (get_eis_an(&ip->mods.mf1, &desc1, &nib) != 0) { // must fetch when needed
            ret = 1;
            break;
        }
        uint z = ~mask & (nib ^ test_nib);
        log_msg(NOTIFY_MSG, moi, "compare nibble %#o(%+d): value %03o yields %#o\n", i, i, nib, z);
        if (z == 0)
            break;
    }
    if (ret == 0) {
        IR.tally_runout = i == n;
        // t_uint64 word = setbits36(0, 12, 24, i);
        t_uint64 word = i;
        log_msg(NOTIFY_MSG, moi, "TRO=%d; writing %#o(%+d) to abs addr %#o\n", IR.tally_runout, i, i, y3);
        ret = store_abs_word(y3, word);
    }

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_IBKPT);
    PPR.IC += 4;        // BUG: check other eis mw instr to make sure IC update is after op fetches (affects ic addr mode and prob restart)
    return ret;
}

// ============================================================================

static int op_csl(const instr_t* ip)
{
    // Combine bit strings left

    const char* moi = "OPU::csl";

    uint fill = ip->addr >> 17;
    uint bolr = (ip->addr >> 9) & MASKBITS(4);
    flag_t t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(NOTIFY_MSG, moi, "mf2 = %s\n", mf2text(&mf2));
    char *ops[16] = { "clear", "and", "x&!y", "x", "!x&y", "y", "xor", "or", "!or", "!xor", "!y", "!x&y", "!x", "x|!y", "nand", "set" };
    log_msg(NOTIFY_MSG, moi, "bool oper: %#o =b%d%d%d%d (%s), fill: %d\n", bolr, (bolr>>3)&1, (bolr>>2)&1, (bolr>>1)&1, bolr&1, ops[bolr], fill);

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_bit_desc_t desc1;
    parse_eis_bit_desc(word1, &ip->mods.mf1, &desc1);
    eis_bit_desc_t desc2;
    parse_eis_bit_desc(word2, &mf2, &desc2);

    log_msg(NOTIFY_MSG, moi, "desc1: %s\n", eis_bit_desc_to_text(&desc1));
    log_msg(NOTIFY_MSG, moi, "desc2: %s\n", eis_bit_desc_to_text(&desc2));

    int ret = 0;

    flag_t all_zero = 1;
    while (desc2.n > 0) {
        flag_t bit1, bit2;
        if (desc1.n == 0)
            bit1 = fill;
        else
            if (get_eis_bit(&ip->mods.mf1, &desc1, &bit1) != 0) {   // must fetch when needed
                ret = 1;
                break;
            }
        if (retr_eis_bit(&mf2, &desc2, &bit2) != 0) {   // must fetch when needed
            ret = 1;
            break;
        }
        flag_t r = (bolr >> (3 - ((bit1 << 1) | bit2))) & 1;    // like indexing into a truth table
        log_msg(NOTIFY_MSG, moi, "nbits1=%d, nbits2=%d; %d op(%#o) %d => %d\n", desc1.n, desc2.n, bit1, bolr, bit2, r);
        if (r == bit2) {
            // Do an ordinary "get" to advance the ptr
            if (get_eis_bit(&mf2, &desc2, &bit2) != 0) {
                ret = 1;
                break;
            }
        } else
            if (put_eis_bit(&mf2, &desc2, r) != 0) {
                ret = 1;
                break;
            }
        if (r)
            all_zero = 0;
    }
    if (ret == 0)
        // write unsaved data (if any)
        if (save_eis_bit(&mf2, &desc2) != 0)
            ret = 1;
    if (ret == 0) {
        IR.zero = all_zero;
        IR.truncation = desc1.n > 0;
        if (IR.truncation && t) {
            fault_gen(overflow_fault);  // truncation
            ret = 1;
        }
    }

    PPR.IC += 3;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret;
}
