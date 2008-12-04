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

const int do_18bit_math = 0;

// ============================================================================

static inline t_uint64 lrotate36(t_uint64 x, unsigned n);
static inline void lrotate72(t_uint64* ap, t_uint64* bp, unsigned n);
static inline int32 negate18(t_uint64 x);
static inline t_int64 negate36(t_uint64 x);
static inline int negate72(t_uint64* a, t_uint64* b);
static inline uint min(uint a, uint b);
static inline uint max3(uint a, uint b, uint c);

static int do_op(instr_t *ip);
static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);
static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add18(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add72(t_uint64 a, t_uint64 b, t_uint64* dest1, t_uint64* dest2, int is_unsigned);
static int32 sign18(t_uint64 x);
static t_int64 sign36(t_uint64 x);
static int do_epp(int epp);
static int do_epbp(int n);
static int do_easp(int n);
static int do_an_op(instr_t *ip);   // todo: hack, fold into do_op
static void spri_to_words(int reg, t_uint64* word0p, t_uint64 *word1p);
static int do_spri(int n);
static int op_mlr(const instr_t* ip);
static int op_tct(const instr_t* ip, int fwd);
static int op_mvt(const instr_t* ip);

static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs);   // BUG: temp

static uint saved_tro;

// BUG: move externs to hdr file
extern switches_t switches;
extern int scu_cioc(t_uint64 addr);
extern int scu_set_mask(t_uint64 addr, int port);
extern int scu_get_mask(t_uint64 addr, int port);
extern int scu_set_cpu_mask(t_uint64 addr);
extern int scu_get_calendar(t_uint64 addr);
extern int activate_timer();

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
    addr_modes_t orig_mode = get_addr_mode();
    uint orig_ic = PPR.IC;
    int ret = do_an_op(ip);
    addr_modes_t mode = get_addr_mode();
    if (orig_mode != mode) {
        if (orig_ic == PPR.IC) {
            set_addr_mode(orig_mode);
            log_msg(DEBUG_MSG, "OPU", "Resetting addr mode for sequential instr\n");
        } else {
            log_msg(NOTIFY_MSG, "OPU", "Address mode has been changed with a transfer instr.  Was 0%o, now 0%o\n", orig_ic, PPR.IC);
            cancel_run(STOP_IBKPT);
        }
    }
    return ret;
}


static int do_an_op(instr_t *ip)
{
    // Returns non-zero on error or non-group-7  fault
    // BUG: check for illegal modifiers

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
        log_msg(WARN_MSG, "OPU", "Illegal opcode 0%0o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    } else {
        if (opt_debug) log_msg(DEBUG_MSG, "OPU", "Opcode 0%0o(%d) -- %s\n", op, bit27, instr2text(ip));
    }
    
    // Check instr type for format before addr_mod
    // Todo: check efficiency of lookup table versus switch table
    // Also consider placing calls to addr_mod() in next switch table
    flag_t initial_tally = IR.tally_runout;
    if (ip->is_eis_multiword) {
        log_msg(DEBUG_MSG, "OPU", "Skipping addr_mod() for EIS instr.\n");
    } else if (bit27 == 0) {
#if 1
        addr_mod(ip);       // note that ip == &cu.IR
#else
        switch (op) {
            case opcode0_rpd:
            case opcode0_rpl:
            case opcode0_rpt:
                // special instr format
                log_msg(DEBUG_MSG, "OPU", "Skipping addr_mod() for special case instr.\n");
                cancel_run(STOP_WARN);
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
#endif
    } else {
        switch (op) {
            case opcode1_a4bd:
            case opcode1_a6bd:
            case opcode1_a9bd:
                addr_mod_eis_addr_reg(ip);
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
    }
    
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

            // opcode0_lca unimplemented
            // opcode0_lcaq unimplemented
            // opcode0_lcq unimplemented
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
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
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
                    load_IR(word);
                    IR.not_bar_mode = nbar;
                    IR.abs_mode = abs;
                    addr_modes_t addr_mode = get_addr_mode();
                    if (addr_mode != ABSOLUTE_mode) {   // BUG: check priv mode
                        IR.parity_mask = par;
                        IR.mid_instr_intr_fault = 0;
                    }
                }
                return ret;
            }
            case opcode0_ldq: { // load Q reg
                int ret = fetch_op(ip, &reg_Q);
                if (ret == 0) {
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_ldqc: {    // load Q reg & clear
                int ret = fetch_op(ip, &reg_Q);
                if (ret == 0) {
                    ret = store_word(TPR.CA, 0);
                }
                // BUG: Should ldqc test C(A) before zeroing it?
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
                    log_msg(DEBUG_MSG, "OPU::instr::ldx*", "X[%d]: Loaded 0%Lo => 0%Lo (0%o aka 0%o)\n", n, word, word >> 18, reg_X[n], reg_X[n] & MASK18);
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
                    reg_E = getbits36(words[6], 0, 7);
                    // reg_TR not loaded
                    // reg_RALR not loaded
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
                words[6] = setbits36(0, 0, 7, reg_E);
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
                // log_msg(DEBUG_MSG, "OPU::stct1", "saving %012Lo to 0%o\n", word, TPR.CA);
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
            // opcode0_stcq unimplemented
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
                //log_msg(DEBUG_MSG, "OPU::als", "CA = 0%o; bits 11..17 = %0o\n", TPR.CA, n);
                //log_msg(DEBUG_MSG, "OPU::als", "A = (%0Lo << %d) ==> %0Lo\n", reg_A, n, (reg_A << n) & MASK36);
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
                log_msg(DEBUG_MSG, "OPU::ars", "%012Lo>>%d ==> %012Lo\n", tmp, n, reg_A);
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
                if (n >= 72)
                    n %= 72;
                if (n != 0) {
                    reg_A = ((reg_A << n) & MASK36) | (reg_Q >> (36 - n));
                    reg_Q <<= n;
                }
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
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
#if 0
                // BUG: ERROR: Why isn't this appropriate?
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    int n = getbits36(word, 11, 7);
                    reg_Q >>= n;
                    IR.zero = reg_Q == 0;
                    IR.neg = bit36_is_neg(reg_Q);
                }
                return ret;
#endif
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
            // adl unimplemented
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
                    if (do_18bit_math && TPR.is_value == 7 && a == 0) {
                        int is_neg = bit18_is_neg(reg_A);
                        if (is_neg != IR.neg) {
                            log_msg(WARN_MSG, "OPU::adla", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                            IR.neg = is_neg;
                            cancel_run(STOP_WARN);
                        }
                    }
                }
                return ret;
            }
            case opcode0_adlaq: {   // Add logical to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
                if (ret == 0) {
                    // BUG: ignoring 18bit math
                    ret = add72(word1, word2, &reg_A, &reg_Q, 1);
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
            // case opcode0_asq unimplemented

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
                    log_msg(DEBUG_MSG, "OPU::sba", "%012Lo - %012Lo ==> adding %012Lo yields %012Lo with carry=%c.\n", a, w, word, reg_A, IR.carry ? 'Y' : 'N');
                    if (ret == 0) {
                        word = 1;
                        ret = add36(reg_A, word, &reg_A);
                        log_msg(DEBUG_MSG, "OPU::sba", "adding one yields %012Lo with carry=%c.\n", reg_A, IR.carry ? 'Y' : 'N');
                        IR.carry |= carry;
                        if (do_18bit_math && TPR.is_value == 7 && (a>>18) == 0) {
                            // arithmetic on "dl" constant
                            if (opt_debug || (a>>18) != 0 || (w >> 18) != 0 || (reg_A>>18) != 0)
                                log_msg(NOTIFY_MSG, "OPU::sba", "A = %012Lo minus %06Lo ,du operand yields %012Lo at IC %06o \n", a, w, reg_A, PPR.IC);
                            flag_t is_neg = bit18_is_neg(reg_A);
                            if (is_neg != IR.neg) {
                                log_msg(WARN_MSG, "OPU::sba", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                                IR.neg = is_neg;
                                // cancel_run(STOP_WARN);
                            }
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
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
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
                    if (do_18bit_math && TPR.is_value == 7 && (q>>18) == 0) {
                        int is_neg = bit18_is_neg(reg_Q);
                        if (is_neg != IR.neg) {
                            log_msg(WARN_MSG, "OPU::sba", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                            IR.neg = is_neg;
 
                        }
                    }
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
                    log_msg(DEBUG_MSG, "OPU::sbq", "%012Lo - %012Lo ==> adding %012Lo yields %012Lo with carry=%c.\n", q, w, word, reg_Q, IR.carry ? 'Y' : 'N');
                    if (ret == 0) {
                        word = 1;
                        ret = add36(reg_Q, word, &reg_Q);
                        log_msg(DEBUG_MSG, "OPU::sbq", "adding one yields %012Lo with carry=%c.\n", reg_Q, IR.carry ? 'Y' : 'N');
                        IR.carry |= carry;
                        if (do_18bit_math && TPR.is_value == 7 && (q>>18) == 0) {
                            // arithmetic on "dl" constant
                            if (opt_debug || (q>>18) != 0 || (w >> 18) != 0 || (reg_Q>>18) != 0)
                                log_msg(NOTIFY_MSG, "OPU::sbq", "A = %012Lo minus %06Lo ,du operand yields %012Lo at IC %06o \n", q, w, reg_Q, PPR.IC);
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
                        log_msg(DEBUG_MSG, buf, "X[%d] -= 0%Lo(%Ld) ==> 0%o+0%Lo == %d+%d = 0%Lo (%Ld)\n",
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
                    log_msg(DEBUG_MSG, "OPU::mpy", "0%Lo * 0%Lo ===> A=0%Lo, Q=0%Lo\n", word, q, reg_A, reg_Q);
                    log_msg(DEBUG_MSG, "OPU::mpy", "%Ld * %Ld ===> A=%Ld, Q=%Ld\n", word, q, reg_A, reg_Q);
                }
                return ret;
            }
            case opcode0_div: {
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_int64 q = bit36_is_neg(reg_Q) ? negate36(reg_Q) : reg_Q;
                    t_int64 w = bit36_is_neg(word) ? negate36(word) : word;
                    if (w == 0 || (q == ((t_uint64)1<<35) && w == -1)) {    // (1<<35) signed is -2**36
                        fault_gen(div_fault);
                        IR.neg = bit36_is_neg(reg_Q);
                        reg_Q = (IR.neg) ? - q  : reg_Q;    // magnitude, absolute value
                        IR.zero = w == 0;
                        ret = 1;
                    } else {
                        log_msg(DEBUG_MSG, "OPU::div", "0%Lo/0%Lo => 0%Lo/0%Lo)\n", reg_Q, word, q, w);
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
            // opcode0_cmk unimplemented

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
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
                if (ret == 0) {
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
                                IR.zero = 1;
                            } else if (reg_Q == word2) {
                                IR.zero = 1;
                                IR.neg = 0;
                                IR.zero = 1;
                            } else {
                                IR.zero = 0;
                                IR.neg = 1;
                                IR.zero = 1;
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
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
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
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
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
            // _ersq unimplemented
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
            
            // call6 unimplemented -- call using pr6 and pr7
            // opcode0_ret unimplemented
            // opcode0_rtcd unimplemented
            
            // opcode0_teo unimplemented
            // opcode0_teu unimplemented
            
            case opcode0_tmi:
                if (IR.neg) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            // tmoz -- see opcode1_tmoz
            //
            case opcode0_tnc:
                if (! IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode0_tnz:
                if (! IR.zero) {
                    log_msg(DEBUG_MSG, "OPU::opcode::tnz", "transfer to %0o\n", TPR.CA);
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                } else
                    log_msg(DEBUG_MSG, "OPU::opcode::tnz", "no transfer (would have been to %0o)\n", TPR.CA);
                return 0;
            case opcode0_tov:
                if (IR.overflow) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                IR.overflow = 0;
                return 0;
            case opcode0_tpl:
                if (! IR.neg) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            // tpnz -- see opcode1_tpnz
            case opcode0_tra:
                PPR.IC = TPR.CA;
                PPR.PSR = TPR.TSR;
                return 0;
            case opcode0_trc:
                if (IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            // trtf -- see opcode1_trtf
            // trtn -- see opcode1_trtn
            // opcode0_tsp0 .. tsp7 unimplemented
            // opcode0_tss unimplemented

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
                return 0;
            }
            // ttf unimplemented
            case opcode0_tze:
                if (IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
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

            // eawp0 ..eawp7 unimplemented

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
                        log_msg(DEBUG_MSG, "OPU::lpri", "PR[%d]: rnr=%o, snr=%o, wordno=%0o, bitno=0%o\n",
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
                        AR_PR[n].PR.snr = 070000;   // bits 0..2 if 15-bit register
                    else
                        AR_PR[n].PR.snr = 0;
                    AR_PR[n].PR.snr |= getbits36(word, 6, 12) & MASK18;
                    AR_PR[n].PR.snr |= getbits36(word, 6, 12) & MASK18;
                    AR_PR[n].wordno = getbits36(word, 18, 18);
                    if (opt_debug) {
                        char cmd[20];
                        sprintf(cmd, "lprp%d", n);
                        log_msg(DEBUG_MSG, cmd, "PR[%d] loaded from value %012o\n", n, word);
                    }
                }
                return ret;
            }

            // spbp0 .. spbp7 unimplemented

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
                log_msg(DEBUG_MSG, cmd, "Packed value %012Lo from PR[%d] (snr=%o, wordno=%0o, bitno=0%o)\n",
                    word, n, AR_PR[n].PR.snr, AR_PR[n].wordno, AR_PR[n].PR.bitno);
                int ret = store_word(TPR.CA, word);
                return ret;
            }

            // adwp0 .. adwp7 unimplemented
            // epaq unimplemented

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
                    log_msg(DEBUG_MSG, "OPU::opcode::xec", "fetch instr: error or fault\n");
                    return 1;   // faulted
                }
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "executing instr at 0%o\n", TPR.CA);
                if (do_op(&IR) != 0) {
                    log_msg(DEBUG_MSG, "OPU::opcode::xec", "fault or error executing instr\n");
                    return 1;
                }
                log_msg(DEBUG_MSG, "OPU::opcode::xec", "finished\n");
                return 0;
            }
            case opcode0_xed: {
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xed invokes xed
                // todo: handle rpd repeats
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
                log_msg(DEBUG_MSG, "OPU::opcode::xed", "executing even instr at 0%Lo\n", y);
                if (do_op(&IR) != 0) {
                    log_msg(WARN_MSG, "OPU::opcode::xed", "fault or error executing even instr\n");
                    return 1;
                }
                // -----------
                if (cpu.xfr) {
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "transfer instr executed, not doing odd instr\n");
                } else {
                    ++ y;
                    if (fetch_instr(y, &IR) != 0) {
                        log_msg(DEBUG_MSG, "OPU::opcode::xed", "fetch odd: error or fault\n");
                        return 1;   // faulted
                    }
                    log_msg(DEBUG_MSG, "OPU::opcode::xed", "executing odd instr at 0%Lo\n", y);
                    if (do_op(&IR) != 0) {
                        log_msg(WARN_MSG, "OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
                log_msg(DEBUG_MSG, "OPU::opcode::xed", "finished\n");
                break;
            }

            // mme unimplemented
            // mme2 unimplemented
            // mme3 unimplemented
            // mme4 unimplemented

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

            // opcode0_sbar unimplemented
            // bcd unimplemented -- binary to binary-coded-decimal
            // gtb unimplemented -- gray to binary
            // lbar unimplemented

            case opcode0_lcpr: {    // load central processor reg (priv)
                int ret = 0;
                t_uint64 word;
                switch (ip->mods.single.tag) {      // no addr modifications
                    case 2:
                        ret = fetch_word(TPR.CA, &word);
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Not writing 0%Lo to cache mode reg.\n", word);
                        if (word != 0)
                            ret = 1;
                        break;
                    case 4:
                        ret = fetch_word(TPR.CA, &word);
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Not writing 0%Lo to mode reg.\n", word);
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
                        log_msg(ERR_MSG, "OPU::opcode::lcpr", "Bad tag 0%o\n", ip->mods.single.tag);
                        ret = 1;
                }
                cancel_run(STOP_WARN);
                return ret;
            }
            case opcode0_ldbr: {
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);   // BUG: which fault?
                    return 1;
                }
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
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
                        SDWAM[i].assoc.is_full = 0;
                        SDWAM[i].assoc.use = i;
                    }
                    if (cu.PT_ON) {
                        PTWAM[i].assoc.is_full = 0;
                        PTWAM[i].assoc.use = i;
                    }
                }
                // BUG: If cache is enabled, reset all cache colume and level full flags
                DSBR.addr = getbits36(word1, 0, 24);
                DSBR.bound = getbits36(word2, 37-36, 14);
                DSBR.u = getbits36(word2, 55-36, 1);
                DSBR.stack = getbits36(word2, 60-36, 12);
                log_msg(DEBUG_MSG, "OPU::ldbr", "DSBR: addr=0%o, bound=0%o(%u), u=%d, stack=0%o\n",
                    DSBR.addr, DSBR.bound, DSBR.bound, DSBR.u, DSBR.stack);
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
                    log_msg(DEBUG_MSG, "OPU::opcode::ldt", "Operand is 0%Lo => 0%Lo\n", word, bits);
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

            case opcode0_sdbr: {
                t_uint64 word0, word1;
                word0 = (t_uint64) DSBR.addr << 12;
                word1 = setbits36(0, 37-36, 14, DSBR.bound);
                word1 = setbits36(word1, 55-36, 1, DSBR.u);
                word1 = setbits36(word1, 60-36, 12, DSBR.stack);
                return store_pair(TPR.CA, word0, word1);
            }
            // sptp unimplemented
            // sptr unimplemented
            // ssdp unimplemented
            // ssdr unimplemented

            case opcode0_cams: {    // Clear Associative Memory Segments
                if (get_addr_mode() != ABSOLUTE_mode) {
                    log_msg(ERR_MSG, "OPU::cams", "cams executed when mode is not absolute.\n");
                    log_msg(ERR_MSG, "OPU::cams", "This should be a fault, but we're ignoring it...\n");
                    // fault_gen(illproc_fault);
                    // return 1;
                    // cancel_run(STOP_WARN);
                    // return 0;
                }
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
                    log_msg(NOTIFY_MSG, "OPU::cams", "Neither enable nor disable requested\n");
                } else {
                    log_msg(WARN_MSG, "OPU::cams", "Unknown enable/disable mode %06o=>0%o\n", TPR.CA, enable);
                    cancel_run(STOP_WARN);
                }
                int i;
                for (i = 0; i < 16; ++i) {
                    SDWAM[i].assoc.is_full = 0;
                    SDWAM[i].assoc.use = i;
                    if (clear) {
                        ret = 1;
                        log_msg(WARN_MSG, "OPU::cams", "Clear mode is unimplemented\n");
                        cancel_run(STOP_WARN);
                        // St the full/empty bits of all cache blocks to empty
                        // -- what are cache blocks?
                    }
                    // BUG: the enable flag in each register probably means something different
                    //if (enable == 2)
                    //  SDWAM[i].assoc.enabled = 1;
                    //else if (enable == 1)
                    //  SDWAM[i].assoc.enabled = 0;
                }
                return ret;
            }

            // rmcm unimplemented -- read memory controller mask register

            case opcode0_rscr: { // priv
                // read system controller register (to AQ)
                int ret = 0;
                uint y = (TPR.CA >> 16) & 3;    // get bits one and two of 18bit CA
                uint ea = y << 15;              // and set just those bits in ea
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "CA is 0%0o (y0%03ox)\n", TPR.CA, (TPR.CA >> 3) & 077);
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
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "bad argument, CA 0%o\n", TPR.CA);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                log_msg(DEBUG_MSG, "OPU::opcode::rscr", "A = %Lo\n", reg_A);
                if (show_q)
                    log_msg(DEBUG_MSG, "OPU::opcode::rscr", "Q = %Lo\n", reg_Q);
                return ret;
            }

            case opcode0_rsw: { // read switches
                int low = TPR.CA & 07;
                switch(low) {
                    case 0: // unformatted; maintenance panel data switches
                        // GB61-01B says that DPS8 systems have a display instead of a panel.
                        // Various switches should be on including some of 4..28.  But
                        // how do these match up to bit positions?
#if 1                       
                        reg_A = 0;
#else
                        reg_A = 0;
                        reg_A = setbits36(reg_A, 0, 2, 03); // enable match?
                        reg_A = setbits36(reg_A, 2+4, 1, 1);    // data switches
                        reg_A = setbits36(reg_A, 2+6, 1, 1);    // data switches
                        reg_A = setbits36(reg_A, 2+18, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+19, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+20, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+23, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+24, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+25, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+26, 1, 1);   // data switches
                        reg_A = setbits36(reg_A, 2+28, 1, 1);   // data switches
                        // 31, auto?, off
                        // 32, cycle?, off
                        reg_A = setbits36(reg_A, 33, 1, 1); // execute pb
                        // 34, init?, off
                        // 35, execute?, off
#endif
                        log_msg(WARN_MSG, "OPU::opcode::rsw", "function xxx%o is undocumented.\n", low);
                        cancel_run(STOP_BUG);
                        break;
                    case 2:
                        reg_A = setbits36(0, 4, 2, 0);  // 0 for L68 or DPS
                        reg_A = setbits36(0, 6, 7, switches.FLT_BASE);
                        reg_A = setbits36(reg_A, 19, 1, 0); // 0 for L68
                        reg_A = setbits36(reg_A, 27, 1, 0); // cache
                        reg_A = setbits36(reg_A, 28, 1, 0); // gcos mode extended memory option off
                        reg_A = setbits36(reg_A, 29, 4, 016);   // 1110b=>L68 re start_cpu.pl1
                        reg_A = setbits36(reg_A, 33, 3, switches.cpu_num);
                        log_msg(NOTIFY_MSG, "OPU::opcode::rsw", "function xxx%o returns A=%012Lo.\n", low, reg_A);
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
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "A = %Lo\n", reg_A);
                log_msg(DEBUG_MSG, "OPU::opcode::sscr", "Q = %Lo\n", reg_Q);
                int ret = 0;
                // uint y = getbits36(TPR.CA, 0, 2);        // BUG: CA is 18 bits, not 36
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
                    log_msg(WARN_MSG, "OPU::opcode::sscr", "bad argument, CA 0%o\n", TPR.CA);
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
                if ((ret = get_seg_addr(TPR.CA, 0, &addr)) == 0)
                    reg_A = (t_uint64) addr << 12;  // upper 24 bits
                return ret;
            }

            case opcode0_dis:
                // delay until interrupt set
                if (1) {
                    log_msg(WARN_MSG, "OPU::dis", "DIS unimplemented; simply continue when ready\n");
                    cancel_run(STOP_IBKPT);
                    return 1;
                } else {
                    log_msg(WARN_MSG, "OPU::dis", "DIS unimplemented.   Continuing after history dump.\n");
                    cmd_dump_history();
                    return 0;
                }


            // limr ??
            // ldo ??
            // camp2 ??
            //

            default:
                log_msg(ERR_MSG, "OPU", "Unimplemented opcode 0%0o(0)\n", op);
                cancel_run(STOP_BUG);
                return 1;
        }
    } else {
        switch (op) {
            case opcode1_tmoz:
                if (IR.neg || IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode1_tpnz:
                if (! IR.neg && ! IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode1_trtf:
                if (! IR.truncation) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode1_trtn:
                if (IR.truncation) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode1_ttn:
                if (IR.tally_runout) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
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

            // eawp0 ..eawp7 unimplemented

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

            // spbp0 .. spbp7 unimplemented

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
                    log_msg(WARN_MSG, "OPU::camp", "Unknown enable/disable mode %06o=>0%o\n", TPR.CA, enable);
                    cancel_run(STOP_WARN);
                }
                int i;
                for (i = 0; i < 16; ++i) {
                    PTWAM[i].assoc.is_full = 0;
                    PTWAM[i].assoc.use = i;
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
                    //  PTWAM[i].assoc.enabled = 1;
                    //else if (enable == 1)
                    //  PTWAM[i].assoc.enabled = 0;
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
                log_msg(NOTIFY_MSG, "OPU::spl", "Not storing enough info to allow EIS decimal op restart.\n");
                return store_yblock8(TPR.CA, words);
            }

            case opcode0_spri0:
                return do_spri(0);
            case opcode0_spri2:
                return do_spri(2);
            case opcode0_spri4:

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

            case opcode1_cmpc: {
                return op_unimplemented_mw(ip, op, opname, 2);
            }
            // opcode1_scd unimplemented -- scan characters double
            // opcode1_scdr unimplemented -- scan characters double in reverse
            // opcode1_scm unimplemented -- scan with mask
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
            // csl unimplemented -- combine bit strings left
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
                log_msg(ERR_MSG, "OPU", "Unimplemented opcode 0%0o(1)\n", op);
                cancel_run(STOP_BUG);
                return 1;
        }
    }
    return 0;
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
    //uint wrong_s = result >> 35;
    //t_uint64 wrong_r = result;
    if ((result >> 36) != 0) {
        // BUG: generates inappropriate? carry when adding a negative to a positive number
        IR.carry = 1;
        //log_msg(WARN_MSG, "OPU::add36", "0%012Lo (%Ld) + 0%012Lo (%Ld) ==> carry on result 0%012Lo (%Ld => %Ld)\n",
        //  a, sign36(a), b, sign36(b), result, sign36(result), sign36(result & MASK36));
        result &= MASK36;
    } else {
        IR.carry = 0;
    }
    uint signr = result >> 35;
    //if (signr != wrong_s) {
    //  log_msg(WARN_MSG, "OPU::add36", "Prior version had incorrect sign 0%o instead of %o -- 0%012Lo (%Ld) + 0%012Lo (%Ld) ==> 0%12Lo ==> 0%012Lo (%Ld => %Ld)\n",
    //      wrong_s, signr, a, sign36(a), b, sign36(b), wrong_r, result, sign36(result), sign36(result & MASK36));
    //}
    IR.zero = result == 0;
    IR.neg = signr;
    if (sign1 == sign2 && signr != sign1) {
        if(opt_debug) log_msg(DEBUG_MSG, "OPU::add36", "0%012Lo (%Ld) + 0%012Lo (%Ld) ==> overflow on result 0%12Lo (%Ld)\n",
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
                log_msg(NOTIFY_MSG, "OPU::adder36", "%012o + %012o = %012o for ,dl modified at IC %06o\n", a, b, result, PPR.IC);
            flag_t is_neg = bit18_is_neg(result);
            if (is_neg != IR.neg) {
                log_msg(WARN_MSG, "OPU::addr36", "Changing IR.neg to %d for ,dl operand.\n", is_neg);
                IR.neg = is_neg;
                cancel_run(STOP_WARN);
            }
            if ((result >> 18) != 0 && ! IR.carry) {
                log_msg(WARN_MSG, "OPU::addr", "Carry is probably wrong for ,dl operand.\n");
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

static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        // log_msg(DEBUG_MSG, "OPU::sign18", "0%Lo => 0%o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}

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
    log_msg(DEBUG_MSG, buf, "PR[%o]=TPR -- rnr=0%o, snr=0%o, wordno=%o, bitno=%o\n", epp, AR_PR[epp].PR.rnr, AR_PR[epp].PR.snr, AR_PR[epp].wordno, AR_PR[epp].PR.bitno);
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
    log_msg(DEBUG_MSG, buf, "PR[%o]=TPR$0 -- rnr=0%o, snr=0%o, wordno=zero, bitno=zero\n", n, AR_PR[n].PR.rnr, AR_PR[n].PR.snr);
    return 0;
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
    if(opt_debug) log_msg(DEBUG_MSG, "OPU::spri*", "Saving PR[%d]: snr=0%o, rnr=%o, wordno=0%o, bitno=0%o\n", n, AR_PR[n].PR.snr, AR_PR[n].PR.rnr, AR_PR[n].wordno, AR_PR[n].PR.bitno);
    // log_msg(NOTIFY_MSG, "OPU::spri*", "Saving PR[%d]: { snr=0%o, rnr=%o, wordno=0%o, bitno=0%o} as {%012Lo, %012Lo}\n", n, AR_PR[n].PR.snr, AR_PR[n].PR.rnr, AR_PR[n].wordno, AR_PR[n].PR.bitno, word0, word1);
    return store_pair(TPR.CA, word0, word1);
}

// ============================================================================


static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs)
{
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

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&desc2));
    if (nargs == 3) {
        t_uint64 word3;
        if (fetch_word(PPR.IC + 3, &word3) != 0)
            return 1;
        log_msg(DEBUG_MSG, moi, "word3 = %012Lo\n", word3);
    }

    PPR.IC += nargs + 1;
}

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

    PPR.IC += 3;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&desc2));

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
            return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    return ret != 0;
}

// ============================================================================

static int get_table_char(uint addr, uint index, uint *charp)
{
    // index &= MASKBITS(9);
    addr += index / 4;      // 9 bit entries
    t_uint64 word;
    if (fetch_abs_word(addr, &word) != 0) {         // BUG: assumes entire table fits in same page
        // log_msg(WARN_MSG, "OPU::get-table-char", "Error fetching from addr 0%o\n", addr);
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

    PPR.IC += 4;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&desc1));
    uint addr2, addr3;
    if (get_eis_indir_addr(word2, &addr2) != 0)
        return 1;
    if (get_eis_indir_addr(word3, &addr3) != 0)
        return 1;
    log_msg(DEBUG_MSG, moi, "table addr: 0%o\n", addr2);
    log_msg(DEBUG_MSG, moi, "result addr: 0%o\n", addr3);

    uint n = desc1.n;
    extern DEVICE cpu_dev;
    //if (!fwd) { ++opt_debug; ++ cpu_dev.dctrl;}
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
            log_msg(DEBUG_MSG, moi, "Index %d: found non-zero table entry 0%o for table index 0%o, from offset %d in the str.\n", i, t, m, idx);
            t_uint64 word = (t_uint64) t << 27;
            word = setbits36(word, 12, 24, i);
            if (store_abs_word(addr3, word) != 0) {
                //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
                return 1;
            }
            //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
            //cancel_run(STOP_IBKPT);
            return 0;
        }
    }

    //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
    t_uint64 word = setbits36(0, 12, 24, n);
    if (store_abs_word(addr3, word) != 0)
        return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_IBKPT);
    // log_msg(DEBUG_MSG, moi, "finished.\n");
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

    PPR.IC += 4;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);
    uint addr3;
    if (get_eis_indir_addr(word3, &addr3) != 0)
        return 1;

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_alpha_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_alpha_desc_to_text(&desc2));
    log_msg(DEBUG_MSG, moi, "translation table at 0%Lo => 0%o\n", word3, addr3);

    int ret = 0;

    uint n = desc1.n;
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
    if (save_eis_an(&mf2, &desc2) != 0)
        return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    return ret;
}
