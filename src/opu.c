#include "hw6180.h"

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
static int do_an_op(instr_t *ip);   // todo: hack, fold into do_op
static void spri_to_words(int reg, t_uint64* word0p, t_uint64 *word1p);
static int op_mlr(const instr_t* ip);
static int op_tct(const instr_t* ip);
static int op_mvt(const instr_t* ip);

static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs);   // BUG: temp

// BUG: move externs to hdr file
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
            debug_msg("OPU", "Resetting addr mode for sequential instr\n");
        } else {
            warn_msg("OPU", "Address mode has been changed with a transfer instr.  Was 0%o, now 0%o\n", orig_ic, PPR.IC);
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

    int bit27 = op % 2;
    op >>= 1;
    if (opname == NULL) {
        warn_msg("OPU", "Illegal opcode 0%0o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    } else {
        if (opt_debug) debug_msg("OPU", "Opcode 0%0o(%d) -- %s\n", op, bit27, instr2text(ip));
    }
    
    // Check instr type for format before addr_mod
    // Todo: check efficiency of lookup table versus switch table
    // Also consider placing calls to addr_mod() in next switch table
    flag_t initial_tally = IR.tally_runout;
    if (ip->is_eis_multiword) {
        debug_msg("OPU", "Skipping addr_mod() for EIS instr.\n");
    } else if (bit27 == 0) {
        switch (op) {
            case opcode0_rpd:
            case opcode0_rpl:
            case opcode0_rpt:
                // special instr format
                debug_msg("OPU", "Skipping addr_mod() for special case instr.\n");
                cancel_run(STOP_WARN);
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
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
                reg_A = TPR.CA << 18;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            case opcode0_eaq:
                reg_Q = TPR.CA << 18;
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
                    debug_msg("OPU::instr::ldx*", "X[%d]: Loaded 0%Lo => 0%Lo (0%o aka 0%o)\n", n, word, word >> 18, reg_X[n], reg_X[n] & MASK18);
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
            case opcode0_sta:
                return store_word(TPR.CA, reg_A);
            case opcode0_sreg: {
                t_uint64 words[8];
                words[0] = (reg_X[0] << 18) | reg_X[1];
                words[1] = (reg_X[2] << 18) | reg_X[3];
                words[2] = (reg_X[4] << 18) | reg_X[5];
                words[3] = (reg_X[6] << 18) | reg_X[7];
                words[4] = reg_A;
                words[5] = reg_Q;
                words[6] = setbits36(0, 0, 7, reg_E);
                words[7] = setbits36(0, 0, 27, reg_TR);
                words[7] = setbits36(words[7], 33, 3, reg_RALR);
                return store_yblock8(TPR.CA, words);
            }
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
            // stba
            // stbq
            case opcode0_stc1: {
                t_uint64 word;
                save_IR(&word);
                word = setbits36(word, 25, 1, initial_tally);
                word |= ((PPR.IC + 1) << 18);
                return store_word(TPR.CA, word);
            }
            case opcode0_stc2: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word &= MASK18;
                    word |= ((PPR.IC + 2) << 18);
                }
                return store_word(TPR.CA, word);
            }
            case opcode0_stca: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    for (int i = 0; i < 6; ++i) {
                        if ((ip->mods.single.tag & (1<<(5-i))) != 0)
                            word = setbits36(word, i * 6, 6, getbits36(reg_A, i * 6, 6));
                    }
                }
            }
            case opcode0_stq:   // Store Q register
                return store_word(TPR.CA, reg_Q);
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
                debug_msg("OPU::stx*", "fetching word\n");
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = (word & MASK18) | (reg_X[n] << 18);
                    debug_msg("OPU::stx*", "storing word\n");
                    ret = store_word(TPR.CA, word);
                }
                debug_msg("OPU::stx*", "done\n");
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
                    word = (word & (MASK18<<18)) | reg_X[n];
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
                //debug_msg("OPU::als", "CA = 0%Lo; bits 11..17 = %0o\n", (t_uint64) TPR.CA, n);
                //debug_msg("OPU::als", "A = (%0Lo << %d) ==> %0Lo\n", reg_A, n, (reg_A << n) & MASK36);
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
                debug_msg("OPU::ars", "%012Lo>>%d ==> %012Lo\n", tmp, n, reg_A);
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
            // adaq
            // adl
            // adla
            case opcode0_adla: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_A += word;
                    if ((IR.carry = reg_A & MASK36) != 0)
                        reg_A &= MASK36;
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                return ret;
            }
            case opcode0_adlaq: {   // Add logical to AQ
                int n = op & 07;
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
                if (ret == 0) {
                    ret = add72(reg_A, reg_Q, &word1, &word2, 1);
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
                    if (add18(word, reg_X[n], &word) == 0)
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

            case opcode0_sba: { // Subtract from A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = negate36(word);
                    ret = add36(reg_A, word, &reg_A);
                }
                return ret;
            }
            case opcode0_sbaq: {    // Subtract from AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
                if (ret == 0)
                    if ((ret = negate72(&word1, &word2)) == 0)
                        ret = add72(reg_A, reg_Q, &word1, &word2, 0);
                return ret;
            }
            // sbla
            // sblaq
            case opcode0_sblq: {    // Subtract logical from Q
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    uint sign = reg_Q >> 35;
                    reg_Q = (reg_Q - word) & MASK36;
                    uint rsign = reg_Q >> 35;
                    IR.zero = reg_Q == 0;
                    IR.neg = rsign;
                    IR.carry = sign != rsign;
                }
                return ret;
            }
            //sblxn
            case opcode0_sbq: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    word = negate36(word);
                    ret = add36(reg_Q, word, &reg_Q);
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
                    if (add18(word, reg_X[n], &result) == 0) {
#if 0
                        char buf[40];
                        sprintf(buf, "OPU::sbx%d", n);
                        debug_msg(buf, "X[%d] -= 0%Lo(%Ld) ==> 0%o+0%Lo == %d+%d = 0%Lo (%Ld)\n",
                            n, orig, orig,
                            reg_X[n], word, reg_X[n], sign18(word),
                            result, result);
                        debug_msg(buf, "IR: zero=%d, neg=%d, carry=%d\n", IR.zero, IR.neg, IR.carry);
#endif
                        reg_X[n] = result;
                    }
                }
                return ret;
            }
            //ssa
            //ssq
            //ssxn
            //swca
            //swcq

            case opcode0_mpy: {
                t_uint64 word;
                int ret;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_uint64 q = reg_Q;
                    mpy(sign36(word), sign36(reg_Q), &reg_A, &reg_Q);
                    IR.zero = reg_Q == 0 && reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                    debug_msg("OPU::mpy", "0%Lo * 0%Lo ===> A=0%Lo, Q=0%Lo\n", word, q, reg_A, reg_Q);
                    debug_msg("OPU::mpy", "%Ld * %Ld ===> A=%Ld, Q=%Ld\n", word, q, reg_A, reg_Q);
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
                        debug_msg("OPU::div", "0%Lo/0%Lo => 0%Lo/0%Lo)\n", reg_Q, word, q, w);
                        reg_Q = (q / w) & MASK36;
                        reg_A = (q % w) & MASK36;
                        IR.zero = reg_Q == 0;
                        IR.neg = bit36_is_neg(reg_Q);
                    }
                }
                return ret;
            }

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

            case opcode0_szn: {
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
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

            case opcode0_ora: { // OR to A
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_A |= word;
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }
            case opcode0_oraq: {    // OR to AQ
                t_uint64 word1, word2;
                int ret = fetch_pair(TPR.CA, &word1, &word2);   // BUG: fetch_op not needed?
                if (ret == 0) {
                    reg_A |= word1;
                    reg_Q |= word2;
                    IR.zero = word1 == 0 && word2 == 0;
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
                    IR.neg = bit36_is_neg(word);
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
                    IR.neg = bit18_is_neg(word);
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
                    word &= reg_X[n];       // results discarded except for IR bits
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                }
                return ret;
            }

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

            case opcode0_tmi:
                if (IR.neg) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode0_tnc:
                if (! IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode0_tnz:
                if (! IR.zero) {
                    debug_msg("OPU::opcode::tnz", "transfer to %0Lo\n", TPR.CA);
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                } else
                    debug_msg("OPU::opcode::tnz", "no transfer (would have been to %0Lo)\n", TPR.CA);
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
            case opcode0_tze:
                if (IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;

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
                        debug_msg("OPU::lpri", "PR[%d]: rnr=%o, snr=%o, wordno=%0o, bitno=0%o\n",
                            i, AR_PR[i].PR.rnr, AR_PR[i].PR.snr, AR_PR[i].wordno, AR_PR[i].PR.bitno);
                    }
                }
                return ret;
            }
            // lprp*
            // spbp*
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
            case opcode0_spri2:
            case opcode0_spri4:
            case opcode0_spri6: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int n = op & 07;
                t_uint64 word0, word1;
                spri_to_words(n, &word0, &word1);
                return store_pair(TPR.CA, word0, word1);
            }

            case opcode0_xec: {
                // todo: combine with xec
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xec invokes xec
                // todo: handle rpd repeats
                // BUG: EIS multiword instructions may not be handled
                t_uint64 word0;
                instr_t IR;
                if (fetch_instr(TPR.CA, &IR) != 0) {
                    debug_msg("OPU::opcode::xec", "fetch instr: error or fault\n");
                    return 1;   // faulted
                }
                debug_msg("OPU::opcode::xec", "executing instr at 0%Lo\n", TPR.CA);
                if (do_op(&IR) != 0) {
                    debug_msg("OPU::opcode::xec", "fault or error executing instr\n");
                    return 1;
                }
                debug_msg("OPU::opcode::xec", "finished\n");
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
                    debug_msg("OPU::opcode::xed", "decode addr: error or fault\n");
                    return 1;   // faulted
                }
#else
                uint y = TPR.CA - TPR.CA % 2;   // force even
#endif
                // -----------
                if (fetch_instr(y, &IR) != 0) {
                    debug_msg("OPU::opcode::xed", "fetch even: error or fault\n");
                    return 1;   // faulted
                }
                debug_msg("OPU::opcode::xed", "executing even instr at 0%Lo\n", y);
                if (do_op(&IR) != 0) {
                    warn_msg("OPU::opcode::xed", "fault or error executing even instr\n");
                    return 1;
                }
                // -----------
                if (cpu.xfr) {
                    debug_msg("OPU::opcode::xed", "transfer instr executed, not doing odd instr\n");
                } else {
                    ++ y;
                    if (fetch_instr(y, &IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fetch odd: error or fault\n");
                        return 1;   // faulted
                    }
                    debug_msg("OPU::opcode::xed", "executing odd instr at 0%Lo\n", y);
                    if (do_op(&IR) != 0) {
                        warn_msg("OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
                debug_msg("OPU::opcode::xed", "finished\n");
                break;
            }

            case opcode0_nop:
            case opcode0_puls1:
            case opcode0_puls2:
                // BUG: certain address forms may generate faults -- so addr_mod should gen faults
                return 0;

            case opcode0_lcpr: {    // load central processor reg (priv)
                int ret = 0;
                t_uint64 word;
                switch (ip->mods.single.tag) {      // no addr modifications
                    case 2:
                        ret = fetch_word(TPR.CA, &word);
                        complain_msg("OPU::opcode::lcpr", "Not writing 0%Lo to cache mode reg.\n", word);
                        if (word != 0)
                            ret = 1;
                        break;
                    case 4:
                        ret = fetch_word(TPR.CA, &word);
                        complain_msg("OPU::opcode::lcpr", "Not writing 0%Lo to mode reg.\n", word);
                        if (word != 0)
                            ret = 1;
                        break;
                    case 3:
                        complain_msg("OPU::opcode::lcpr", "history reg zero unimplemented.\n");
                        break;
                    case 7:
                        complain_msg("OPU::opcode::lcpr", "history reg setting unimplemented.\n");
                        ret = 1;
                        break;
                    default:
                        complain_msg("OPU::opcode::lcpr", "Bad tag 0%o\n", ip->mods.single.tag);
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
                // BUG: Check that SDWAM is enabled (whatever that means)
                // BUG: Check that PTWAM is enabled (whatever that means)
                for (int i = 0; i < 16; ++i) {
                    SDWAM[i].assoc.is_full = 0;
                    SDWAM[i].assoc.use = i;
                    PTWAM[i].assoc.is_full = 0;
                    PTWAM[i].assoc.use = i;
                }
                // todo: If cache is enabled, reset all cache colume and level full flags
                DSBR.addr = getbits36(word1, 0, 24);
                DSBR.bound = getbits36(word2, 0, 14);   // 37-36- 1
                DSBR.u = getbits36(word2, 18, 1);   // 50-36-1
                DSBR.stack = getbits36(word2, 23, 12);  // 60-36-1
                debug_msg("OPU::ldbr", "DSBR: addr=0%o, bound=0%o(%u), u=%d, stack=0%o\n",
                    DSBR.addr, DSBR.bound, DSBR.bound, DSBR.u, DSBR.stack);
                return 0;
            }
            case opcode0_ldt: { // load timer reg (priv)
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    t_uint64 bits = getbits36(word, 0, 27);
                    debug_msg("OPU::opcode::ldt", "Operand is 0%Lo => 0%Lo\n", word, bits);
                    reg_TR = bits;
                    activate_timer();
                }
                return ret;
            }
            // lptp
            // lptr
            // lra
            // lsdp
            // lsdr
            // rcu
            // scpr
            // scu
            case opcode0_sdbr: {
                t_uint64 word0, word1;
                word0 = DSBR.addr << 12;
                word1 = setbits36(0, 0, 14, DSBR.bound);    // 37-36-1=0
                word1 = setbits36(word1, 0, 18, DSBR.u);    // 55-36-1=18
                word1 = setbits36(word1, 0, 23, DSBR.stack); // 60-36-1=23
                return store_pair(TPR.CA, word0, word1);
            }
            // sptp
            // sptr
            // ssdp
            // ssdr
            case opcode0_cams: {    // Clear Associative Memory Segments
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                //int clear = getbits36(TPR.CA, 15, 1);
                //int enable = getbits36(TPR.CA, 16, 2);
                int clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                int i;
                for (i = 0; i < 16; ++i) {
                    SDWAM[i].assoc.is_full = 0;
                    SDWAM[i].assoc.use = i;
                    if (clear) {
                        ret = 1;
                        warn_msg("OPU::cams", "Clear mode is unimplemented\n");
                        cancel_run(STOP_WARN);
                        // St the full/empty bits of all cache blocks to empty
                        // -- what are cache blocks?
                    }
                    if (enable == 2)
                        SDWAM[i].assoc.enabled = 1;
                    else if (enable == 1)
                        SDWAM[i].assoc.enabled = 0;
                }
                return ret;
            }

            case opcode0_rscr: { // priv
                // read system controller register (to AQ)
                int ret = 0;
                uint y = (TPR.CA >> 16) & 3;    // 18bit CA
                uint ea = y << 15;
                debug_msg("OPU::opcode::rscr", "EA is 0%04o\n", ea);
                debug_msg("OPU::opcode::rscr", "CA is 0%04Lo (0%03Lo=>0%03Lo)\n", TPR.CA, (TPR.CA >> 3), TPR.CA & ~7);
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    warn_msg("OPU::opcode::rscr", "mode register selected\n");
                    complain_msg("OPU::opcode::rscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    warn_msg("OPU::opcode::rscr", "sys config switches unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0020) {
                    debug_msg("OPU::opcode::rscr", "port zero selected\n");
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
                    warn_msg("OPU::opcode::rscr", "interrupts unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    ret = scu_get_calendar(TPR.CA);
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    ret = scu_get_calendar(TPR.CA);
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    warn_msg("OPU::opcode::rscr", "store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    warn_msg("OPU::opcode::rscr", "store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    debug_msg("OPU::opcode::rscr", "bad argument, CA 0%Lo\n", TPR.CA);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                debug_msg("OPU::opcode::rscr", "A = %Lo\n", reg_A);
                debug_msg("OPU::opcode::rscr", "Q = %Lo\n", reg_Q);
                return ret;
            }

            case opcode0_cioc: { // priv
                // Connect I/O channel
                debug_msg("OPU", "CIOC\n");
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
            case opcode0_sscr: { // priv
                // set system controller register (to value in AQ)
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                debug_msg("OPU::opcode::sscr", "A = %Lo\n", reg_A);
                debug_msg("OPU::opcode::sscr", "Q = %Lo\n", reg_Q);
                int ret = 0;
                // uint y = getbits36(TPR.CA, 0, 2);        // BUG: CA is 18 bits, not 36
                uint y = (TPR.CA >> 16) & 3;    // 18bit CA
                uint ea = y << 15;
                debug_msg("OPU::opcode::sscr", "EA is 0%04o\n", ea);
                debug_msg("OPU::opcode::sscr", "CA is 0%04Lo (0%03Lo=>0%03Lo)\n", TPR.CA, (TPR.CA >> 3), TPR.CA & ~7);
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    debug_msg("OPU::opcode::sscr", "mode register selected\n");
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0020) {
                    debug_msg("OPU::opcode::sscr", "port zero selected\n");
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
                    debug_msg("OPU::opcode::sscr", "port seven selected\n");
                    ret = scu_set_mask(TPR.CA, 7);
                } else if ((TPR.CA & ~7) == ea + 0030) {
                    // interrupts
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    // calendar
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    // calendar
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    warn_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    warn_msg("OPU::opcode::sscr", "bad argument, CA 0%Lo\n", TPR.CA);
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
                        warn_msg("OPU::absa", "Absolute mode undefined\n");
                        (void) store_word(TPR.CA, TPR.CA);  // results undefined, perform arbitrary action
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
                    reg_A = addr << 12; // upper 24 bits
                return ret;
            }
            // dis

            // limr ??
            // ldo ??
            // camp2 ??

            default:
                complain_msg("OPU", "Unimplemented opcode 0%0o(0)\n", op);
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
#if 0
            case opcode1_trtf:
                if (IR.truncation) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                IR.truncation = 0;
                return 0;
#endif
            case opcode1_ttn:
                if (IR.tally_runout) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;

            case opcode1_epp1:
                return do_epp(1);
            case opcode1_epp3:
                return do_epp(3);
            case opcode1_epp5:
                return do_epp(5);
            case opcode1_epp7:
                return do_epp(7);

            case opcode1_camp: {    // Clear Associative Memory Pages
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                //int sel_clear = getbits36(TPR.CA, 15, 1);
                //int enable = getbits36(TPR.CA, 16, 2);
                int sel_clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
                int i;
                for (i = 0; i < 16; ++i) {
                    PTWAM[i].assoc.is_full = 0;
                    PTWAM[i].assoc.use = i;
                    if (sel_clear) {
                        ret = 1;
                        warn_msg("OPU::camp", "Selective Clear mode is Unimplemented\n");
                        cancel_run(STOP_BUG);
                        // for any cache block for which upper 15 bits of the dir [are]
                        // entry equal PTWAM(i).ADDR0..13, 
                        //  set full/empty to empty
                        // -- what are cache blocks and dirs?
                    }
                    if (enable == 2)
                        PTWAM[i].assoc.enabled = 1;
                    else if (enable == 1)
                        PTWAM[i].assoc.enabled = 0;
                }
                return ret;
            }

            case opcode1_tct: {
                int ret = op_tct(ip);
                return ret;
            }
            case opcode1_tctr: {
                int ret = op_unimplemented_mw(ip, op, opname, 3);
                return ret;
            }
            case opcode1_mlr: {
                int ret = op_mlr(ip);
                return ret;
            }
            // mrl
            // mve
            case opcode1_mvt: {
                int ret = op_mvt(ip);
                return ret;
            }

            // aarn
            // larn
            // lpl
            // narn
            // ara
            // arn
            // sarn
            // sareg
            // spl
            // a4bd
            // a6bd
            case opcode1_a9bd:
                debug_msg("OPU::a9bd", "APU does our work for us\n");
                return 0;
            // abd
            // awd
            // s4bd
            // s6bd
            // s9bd
            // sbd
            // swd
            case opcode1_cmpc: {
                uint fill = ip->addr >> 9;
                uint t = (ip->addr >> 8) & 1;
                uint mf2bits = ip->addr & MASKBITS(7);
                eis_mf_t mf2;
                t_uint64 word1, word2;
                if (fetch_mf_ops(&ip->mods.mf1, &word1, parse_mf(mf2bits, &mf2), &word2, NULL, NULL) != 0)
                    return 1;
                PPR.IC += 3;
                uint y1 = getbits36(word1, 0, 18);  // addr
                uint y2 = getbits36(word2, 0, 18);
                uint cn1 = getbits36(word1, 18, 3); // 1st char position
                uint cn2 = getbits36(word2, 18, 3);
                uint ta1 = getbits36(word1, 21, 2); // data type
                uint n1 = getbits36(word1, 24, 12); // string len
                uint n2 = getbits36(word2, 24, 12);
                int nbits1 = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : 4;
                fix_mf_len(&n1, &ip->mods.mf1, nbits1);

                debug_msg("OPU::cmpc", "mf2 = {ar=%d, rl=%d, id=%d, reg=0%o}\n",
                    mf2.ar, mf2.rl, mf2.id, mf2.reg);
                debug_msg("OPU::cmpc", "y1=0%o, y2=0%o, cn1=0%o, cn2=0%o, ta1=0%o, n1=%d, n2=%d\n", y1, y2, cn1, cn2, ta1, n1, n2);
                uint addr1, addr2;
                // t_uint64 word1, word2;
                uint bitno1, bitno2;
                if (get_mf_an_addr(y1, &ip->mods.mf1, &addr1, &bitno1) != 0)
                    return 1;
                if (get_mf_an_addr(y2, &mf2, &addr2, &bitno2) != 0)
                    return 1;
                if (fetch_abs_word(addr1, &word1) != 0)
                    return 1;
                if (fetch_abs_word(addr2, &word2) != 0)
                    return 1;
                debug_msg("OPU::cmpc", "first word at 0%o:  %012Lo\n", addr1, word1);
                debug_msg("OPU::cmpc", "second word at 0%o: %012Lo\n", addr2, word2);
                complain_msg("OPU::cmpc", "Unimplemented.\n");
                cancel_run(STOP_BUG);
                return 0;
            }

            case opcode1_spri1:
            case opcode1_spri3:
            case opcode1_spri5:
            case opcode1_spri7: {
                if (get_addr_mode() == BAR_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int n = op & 07;
                t_uint64 word0, word1;
                spri_to_words(n, &word0, &word1);
                return store_pair(TPR.CA, word0, word1);
            }

            default:
                complain_msg("OPU", "Unimplemented opcode 0%0o(1)\n", op);
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
    return add36(word, *dest, dest);
}

// ----------------------------------------------------------------------------

static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest)
{
    uint sign1 = a >> 35;
    uint sign2 = b >> 35;
    t_uint64 result = a + b;
    uint signr = result >> 35;
    if ((result >> 36) != 0) {
        // BUG: generates inappropriate? carry when adding a negative to a positive number
        IR.carry = 1;
        warn_msg("OPU::add36", "0%012Lo (%Ld) + 0%012Lo (%Ld) ==> carry on result 0%012Lo (%Ld => %Ld)\n",
            a, sign36(a), b, sign36(b), result, sign36(result), sign36(result & MASK36));
        result &= MASK36;
    } else {
        IR.carry = 0;
    }
    IR.zero = result == 0;
    IR.neg = signr;
    if (sign1 == sign2 && signr != sign1) {
        warn_msg("OPU::add36", "0%012Lo (%Ld) + 0%012Lo (%Ld) ==> overflow on result 0%12Lo (%Ld)\n",
            a, sign36(a), b, sign36(b), result, sign36(result));
        IR.overflow = 1;
        if (IR.overflow_mask == 0) {
            fault_gen(overflow_fault);
            return 1;
        }
    }

    *dest = result;
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
    //return ((x << n) & ~(~((t_uint64)0)<<36)) | (x >> (36-n));
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
        // debug_msg("OPU::sign18", "0%Lo => 0%o (%+d decimal)\n", x, r, r);
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
        fault_gen(illproc_fault);   // BUG: which fault?
        return 1;
    }

    AR_PR[epp].PR.rnr = TPR.TRR;
    AR_PR[epp].PR.snr = TPR.TSR;
    AR_PR[epp].wordno = TPR.CA & MASK18;
    AR_PR[epp].PR.bitno = TPR.TBR;
    char buf[20];
    sprintf(buf, "OPU::epp%d", epp);;
    debug_msg(buf, "PR[%o]=TPR -- rnr=0%o, snr=0%o, wordno=%o, bitno=%o\n", epp, AR_PR[epp].PR.rnr, AR_PR[epp].PR.snr, AR_PR[epp].wordno, AR_PR[epp].PR.bitno);
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
    word1 = setbits36(0, 0, 21, AR_PR[reg].PR.bitno); // 36-(72-57)
    *word0p = word0;
    *word1p = word1;
}

// ============================================================================


static int op_old_mlr(const instr_t* ip)
{
    // old version

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    t_uint64 word1, word2;
    (void) parse_mf(mf2bits, &mf2);
    debug_msg("OPU::mlr", "mf2 = {ar=%d, rl=%d, id=%d, reg=0%o}\n", mf2.ar, mf2.rl, mf2.id, mf2.reg);
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    PPR.IC += 3;

    uint y1 = getbits36(word1, 0, 18);  // addr
    uint y2 = getbits36(word2, 0, 18);
    uint cn1 = getbits36(word1, 18, 3); // 1st char position
    uint cn2 = getbits36(word2, 18, 3);
    uint ta1 = getbits36(word1, 21, 2); // data type
    uint ta2 = getbits36(word2, 21, 2);
    uint n1 = getbits36(word1, 24, 12); // string len
    uint n2 = getbits36(word2, 24, 12);
    int nbits1 = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : 4;
    fix_mf_len(&n1, &ip->mods.mf1, nbits1);
    int nbits2 = (ta2 == 0) ? 9 : (ta2 == 1) ? 6 : 4;
    fix_mf_len(&n2, &mf2, nbits2);

    int easy = 0;
    debug_msg("OPU::mlr", "y1=0%o, y2=0%o, cn1=0%o, cn2=0%o, ta1=0%o, ta2=0%o, n1=0%o(%d), n2=0%o(%d)\n", y1, y2, cn1, cn2, ta1, ta2, n1, n1, n2, n2);

    if (ta1 == ta2 && cn1 == 0 && cn2 == 0) {
        int nbits = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : 4;
        int nparts = 36 / nbits;
        debug_msg("OPU::mlr", "nbits = %d; nparts = %d\n", nbits, nparts);
        if (n1 % nparts == 0 && n2 % nparts == 0) {
            // full words
            easy = 1;
            int nwords = min(n1,n2) / nparts;
            int saved_debug = opt_debug;
            for (int i = 0; i < nwords; ++i) {
                //if (i == 1)
                //  opt_debug = 0;
                if (i == nwords - 1)
                    opt_debug = saved_debug;
                uint addr1, addr2;
                uint bitno1, bitno2;
                if (get_mf_an_addr(y1, &ip->mods.mf1, &addr1, &bitno1) != 0)
                    return 1;
                if (get_mf_an_addr(y2, &mf2, &addr2, &bitno2) != 0)
                    return 1;
                if (bitno1 != 0 || bitno2 != 0) {
                    easy = 0;
                    break;
                }
                t_uint64 word;
                int od = opt_debug; opt_debug = 1;
                debug_msg("OPU::mlr", "i=%d; copying 0%o=>0%o to 0%o=>0%o\n", i, y1, addr1, y2, addr2);
                opt_debug = od;
                if (fetch_abs_word(addr1, &word) != 0)
                    return 1;
                debug_msg("OPU::mlr", "Storing %012Lo to y=0%o(addr=0%o)\n", word, y2, addr2);
                if (store_abs_word(addr2, word) != 0)
                    return 1;
                //y1 += nparts;
                //y2 += nparts;
                ++ y1;
                ++ y2;
            }
            opt_debug = saved_debug;
            if (easy) {
                if (n1 > n2)
                    IR.truncation = 1;
                    if (t)
                        fault_gen(overflow_fault);
                else if (n1 < n2) {
                    t_uint64 word = 0;
                    for (int i = 0; i < nparts; ++i)
                        word = (word << nbits) | (fill & MASKBITS(nbits));
                    uint addr2;
                    for (int i = n1; i < n2; ++i) {
                        //if (i == n1+1)
                        //  opt_debug = 0;
                        if (i == n2 - 1)
                            opt_debug = saved_debug;
                        uint bitno2;
                        if (get_mf_an_addr(y2, &mf2, &addr2, &bitno2) != 0)
                            return 1;
                        if (bitno2 != 0) {
                            easy = 0;
                            break;
                        }
                        int od = opt_debug; opt_debug = 1;
                        debug_msg("OPU::mlr", "i=%d; copying fill(0%o=>0%Lo) to 0%o=>0%o\n", i, fill, word, y2, addr2);
                        opt_debug = od;
                        if (store_abs_word(addr2, word) != 0)
                            return 1;
                        //y2 += nparts;
                        ++ y2;
                    }
                }
                opt_debug = saved_debug;
            }
            //warn_msg("OPU::mlr", "Auto Breakpoint.\n");
            //cancel_run(STOP_IBKPT);
        }
    }
    if (! easy) {
        int saved_debug = opt_debug;
        opt_debug = 1;
        complain_msg("OPU::mlr", "Complex mode\n");
        int nbits1 = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : (ta1 == 2) ? 4 : -1;
        int nbits2 = (ta2 == 0) ? 9 : (ta2 == 1) ? 6 : (ta2 == 2) ? 4 : -1;
        if (nbits1 == -1 || nbits2 == -1) {
            fault_gen(illproc_fault);
            return 1;
        }
        if (nbits1 == 9) {
            if ((cn1 & 1) != 0) {
                fault_gen(illproc_fault);
                return 1;
            }
            cn1 /= 2;
        }
        if (nbits2 == 9) {
            if ((cn2 & 1) != 0) {
                fault_gen(illproc_fault);
                return 1;
            }
            cn2 /= 2;
        }
        if (nbits1 * cn1 >= 36 || nbits2 * cn2 >= 36) {
            fault_gen(illproc_fault);
            return 1;
        }
        complain_msg("OPU::mlr", "nbits1=%d, nbits2=%d\n", nbits1, nbits2);
        int bitpos1 = 36;
        int bitpos2 = -1;
        int ret = 0;
        t_uint64 word1;
        t_uint64 word2;
        int first = 1;

        while (n1 > 0 && n2 > 0) {
            --n1;
            --n2;
            if (bitpos1 == 36) {
                // fetch a new src word
                if (n1 < 0)
                    bitpos1 = -1;
                else {
                    uint addr1;
                    uint bitno1;
                    debug_msg("OPU::mlr", "Finding src word.\n");
                    if (get_mf_an_addr(y1++, &ip->mods.mf1, &addr1, &bitno1) != 0) {
                        ret = 1;
                        break;
                    }
                    if (bitno1 != 0) {
                        debug_msg("OPU::mlr", "Src bitno is %d\n", bitno1);
                        if (!first) {
                            warn_msg("OPU::mlr", "Non-zero bitno after first char?\n");
                            cancel_run(STOP_IBKPT);
                        }
                    }
                    debug_msg("OPU::mlr", "Fetching src word.\n");
                    if (fetch_abs_word(addr1, &word1) != 0) {
                        ret = 1;
                        break;
                    }
                    bitpos1 = bitno1;
                }
            }
            if (bitpos2 == -1) {
                word2 = 0;
                uint addr2;
                bitpos2 = 0;
                if (first) {
                    first = 0;
                    if (cn1 != 0)
                        bitpos1 = cn1 * nbits1; // BUG: ERROR: should be +=
                    if (cn2 != 0)
                        bitpos2 = cn2 * nbits2;
                    debug_msg("OPU::mlr", "Checking dest word addr\n"); // it might specify a char offset
                    uint bitno2;
                    if (get_mf_an_addr(y2, &mf2, &addr2, &bitno2) != 0) {   // no incr of y2
                        ret = 1;
                        break;
                    }
                    if (bitno2 != 0) {
                        debug_msg("OPU::mlr", "Dest addr bitno is %d\n", bitno2);
                        bitpos2 += bitno2;
                    }
                    if (bitpos2 != 0) {
                        debug_msg("OPU::mlr", "Fetching first dest word (bitpos = %d, not all bits will be set).\n", bitpos2);
                        if (fetch_abs_word(addr2, &word2) != 0) {
                            ret = 1;
                            break;
                        }
                    } else {
                        debug_msg("OPU::mlr", "No need to fetch first dest word.\n");
                    }
                }
            }
            uint nib = (n1>=0) ? getbits36(word1, bitpos1, nbits1) : MASKBITS(nbits2);
            t_uint64 tmp = word2;
            word2 = setbits36(word2, bitpos2, nbits2, nib);
            debug_msg("OPU::mlr", "N1=%d, N2=%d; source=%012Lo; copying bits @ %d to @ %d: dest %012Lo => %012Lo\n",
                n1, n2, word1, bitpos1, bitpos2, tmp, word2);
            bitpos1 += nbits1;
            bitpos2 += nbits2;
            if (bitpos2 == 36) {
                uint addr2;
                debug_msg("OPU::mlr", "Finding dest addr.\n");
                uint bitno2;
                if (get_mf_an_addr(y2++, &mf2, &addr2, &bitno2) != 0) {
                    ret = 1;
                    break;
                }
                if (bitno2 != 27) {
                    warn_msg("OPU::mlr", "Bitno is %d, not 27 while storing dest.\n", bitno2);
                    cancel_run(STOP_WARN);
                }
                //if (bitno2 != 0) {
                //  warn_msg("OPU::mlr", "Bitno is %d, not zero while storing dest.\n", bitno2);
                //  cancel_run(STOP_WARN);
                //}
                debug_msg("OPU::mlr", "Storing %012o to y=0%o=(addr=0%o)\n", word2, y2, addr2);
                if (store_abs_word(addr2, word2) != 0) {
                    ret = 1;
                    break;
                }
                bitpos2 = -1;
            }
        }
        if (bitpos2 != -1) {
            // write all or part of word2
            debug_msg("OPU::mlr", "End of loop.  Finding last dest addr.\n");
            uint addr2;
            debug_msg("OPU::mlr", "Finding dest addr.\n");
            uint bitno2;
            if (get_mf_an_addr(y2++, &mf2, &addr2, &bitno2) != 0) {
                return 1;
            }
            //if (bitno2 != 0) {
            //  warn_msg("OPU::mlr", "Non-zero bitno while storing dest.\n");
            //  cancel_run(STOP_BUG);
            //}
            if (bitno2 + 9 != bitpos2) {
                warn_msg("OPU::mlr", "Dest addr bitno of %d does not match current output bitno of %d minus 9\n", bitno2, bitpos2);
                cancel_run(STOP_BUG);
            }
            if (bitpos2 == 36) {
                // odd
                warn_msg("OPU::mlr", "Odd, dest is a full word.\n");    // why didn't we write it during loop?
                cancel_run(STOP_WARN);
            } else {
                debug_msg("OPU::mlr", "Dest word isn't full.  Loading dest from memory 0%o and folding.\n", addr2);
                t_uint64 word;
                if (fetch_abs_word(addr2, &word) != 0)
                    return 1;
                t_uint64 tmp = word2;
                word2 = setbits36(word2, bitpos2, 36-bitpos2, word);
                debug_msg("OPU::mlr", "Combined temp dest %012Lo with fetched %012Lo: %012Lo\n", tmp, word, word2);
            }
            debug_msg("OPU::mlr", "Storing to 0%o.\n", addr2);
            if (store_abs_word(addr2, word2) != 0)
                return 1;
        }
        opt_debug = saved_debug;
        //warn_msg("OPU::mlr", "Need to verify; auto breakpoint\n");
        //cancel_run(STOP_WARN);
    }
    debug_msg("OPU::mlr", "finished.\n");
    return 0;
}

static int op_mvt(const instr_t* ip)
{
    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    t_uint64 word1, word2;
    (void) parse_mf(mf2bits, &mf2);
    debug_msg("OPU::mvt", "mf2 = {ar=%d, rl=%d, id=%d, reg=0%o}\n", mf2.ar, mf2.rl, mf2.id, mf2.reg);
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    t_uint64 word3;
    if (fetch_word(PPR.IC + 3, &word3) != 0)
        return 1;
    debug_msg("OPU::mvt", "word3 = %012Lo\n", word3);
    PPR.IC += 4;

    uint y1 = getbits36(word1, 0, 18);  // addr
    uint y2 = getbits36(word2, 0, 18);
    uint cn1 = getbits36(word1, 18, 3); // 1st char position
    uint cn2 = getbits36(word2, 18, 3);
    uint ta1 = getbits36(word1, 21, 2); // data type
    uint ta2 = getbits36(word2, 21, 2);
    uint n1 = getbits36(word1, 24, 12); // string len
    uint n2 = getbits36(word2, 24, 12);
    int nbits1 = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : 4;
    fix_mf_len(&n1, &ip->mods.mf1, nbits1);
    int nbits2 = (ta2 == 0) ? 9 : (ta2 == 1) ? 6 : 4;
    fix_mf_len(&n2, &mf2, nbits2);
    debug_msg("OPU::mvt", "y1=0%o, y2=0%o, cn1=0%o, cn2=0%o, ta1=0%o, ta2=0%o, n1=0%o(%d), n2=0%o(%d)\n", y1, y2, cn1, cn2, ta1, ta2, n1, n1, n2, n2);
    
    warn_msg("OPU::mvt", "Unimplemented\n");
    cancel_run(STOP_BUG);
    return 1;
}

static int op_unimplemented_mw(const instr_t* ip, int op, const char* opname, int nargs)
{
    char buf[20];
    sprintf(buf, "OPU::%s", opname);
    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    t_uint64 word1, word2;
    (void) parse_mf(mf2bits, &mf2);
    debug_msg(buf, "mf2 = {ar=%d, rl=%d, id=%d, reg=0%o}\n", mf2.ar, mf2.rl, mf2.id, mf2.reg);
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    if (nargs == 3) {
        t_uint64 word3;
        if (fetch_word(PPR.IC + 3, &word3) != 0)
            return 1;
        debug_msg(buf, "word3 = %012Lo\n", word3);
    }
    PPR.IC += 3 + 1;

    uint y1 = getbits36(word1, 0, 18);  // addr
    uint y2 = getbits36(word2, 0, 18);
    uint cn1 = getbits36(word1, 18, 3); // 1st char position
    uint cn2 = getbits36(word2, 18, 3);
    uint ta1 = getbits36(word1, 21, 2); // data type
    uint ta2 = getbits36(word2, 21, 2);
    uint n1 = getbits36(word1, 24, 12); // string len
    uint n2 = getbits36(word2, 24, 12);
    int nbits1 = (ta1 == 0) ? 9 : (ta1 == 1) ? 6 : 4;
    fix_mf_len(&n1, &ip->mods.mf1, nbits1);
    int nbits2 = (ta2 == 0) ? 9 : (ta2 == 1) ? 6 : 4;
    fix_mf_len(&n2, &mf2, nbits2);
    debug_msg(buf, "y1=0%o, y2=0%o, cn1=0%o, cn2=0%o, ta1=0%o, ta2=0%o, n1=0%o(%d), n2=0%o(%d)\n", y1, y2, cn1, cn2, ta1, ta2, n1, n1, n2, n2);
    
    warn_msg(buf, "Unimplemented\n");
    cancel_run(STOP_BUG);
    return 1;
}

static int op_mlr(const instr_t* ip)
{
    const char* moi = "OPU::mlr";

    // BUG: Detection of GBCD overpunch is not done

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    debug_msg(moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    PPR.IC += 3;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    eis_alpha_desc_t desc2;
    parse_eis_alpha_desc(word2, &mf2, &desc2);

    debug_msg(moi, "desc1: %s;  desc2: %s\n", eis_alpha_desc_to_text(&desc1), eis_alpha_desc_to_text(&desc2));

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

    warn_msg(moi, "Need to verify; auto breakpoint\n");
    cancel_run(STOP_WARN);
    debug_msg(moi, "finished.\n");
    return ret;
}

static int op_tct(const instr_t* ip)
{

    const char* moi = "OPU::tct";

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, NULL, &word2, NULL, &word3) != 0)
        return 1;

    PPR.IC += 4;
warn_msg(moi, "Unimplemented\n");
cancel_run(STOP_BUG);
return 0;

    eis_alpha_desc_t desc1;
    parse_eis_alpha_desc(word1, &ip->mods.mf1, &desc1);
    // BUG: handle words 2 & 3 as pointers (std eis indir format -- addr, 'A' flag, 4bit reg 'Td'
    // Could just translate immed to address
    //      For Y-char92, just add offsets as needed
    //      For Y3, we just need an addr of a single word to modify

    debug_msg(moi, "desc1: %s;  desc2: %s\n", eis_alpha_desc_to_text(&desc1), eis_alpha_desc_to_text(&desc2));

    int ret = 0;

    // while (desc1.n > 0 && desc2.n > 0)
    while (desc2.n > 0) {
        uint nib;
        if (desc1.n == 0)
            nib = MASKBITS(desc2.nbits);
        else
            if (get_eis_an(&ip->mods.mf1, &desc1, &nib) != 0) { // must fetch when needed
                ret = 1;
                break;
            }
        if (put_eis_an(&mf2, &desc2, nib) != 0) {   // must fetch/store when needed
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

    warn_msg(moi, "Need to verify; auto breakpoint\n");
    cancel_run(STOP_WARN);
    debug_msg(moi, "finished.\n");
    return ret;
}
