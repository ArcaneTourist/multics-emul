#include "hw6180.h"

// extern t_uint64 M[]; /* memory */ // BUG

// ============================================================================

static int do_op(instr_t *ip);

void execute_instr(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)
    // BUG: handle interrupt inhibit
    do_op(&cu.IR);
}

// ============================================================================

static int fetch_op(const instr_t *ip, t_uint64 *wordp)
{
    if (ip->is_value) {
        *wordp = TPR.CA;
        return 0;
    }
    return fetch_word(TPR.CA, wordp);
}

const t_uint64 MASK36 = ~(~((t_uint64)0)<<36);  // lower 36 bits all ones

static inline t_uint64 lrotate(t_uint64 x, unsigned n)
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
        t_uint64 aout_lo = a & ~(~0 << (n - 36));
        t_uint64 bout_hi = b >> (72 - n);
        t_uint64 bout_lo = b & ~(~0 << (n - 36));
        a = (bout_lo << (n - 36)) | aout_hi;
        b = (aout_lo << (n - 36)) | bout_hi;
    }
            
    *ap = a;
    *bp = b;
}

static inline void lshift72(t_uint64* ap, t_uint64* bp, unsigned n)
{
}

// ============================================================================

static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);

static int do_op(instr_t *ip)
{
    // Returns non-zero on error or non-group-7  fault
    // BUG: check for illegal modifiers

    uint op = ip->opcode;
    char *opname = opcodes2text[op];

    int bit27 = op % 2;
    op >>= 1;
    if (opname == NULL) {
        debug_msg("OPU", "Illegal opcode 0%0o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    } else {
        int x = ip->offset;
        if (bit18_is_neg(x))
            x = - ((1<<18) - x);
        debug_msg("OPU", "Opcode 0%0o(%d) -- %s, offset 0%06o(%+d), inhibit %u, pr %u, tag 0%03o(Tm=%u,Td=%02u)\n",
            op, bit27, opname, 
            ip->offset, x, 
            ip->inhibit, ip->pr_bit, ip->tag, ip->tag >> 4, ip->tag & 017);
    }
    
    // Check instr type for format before addr_mod
    // Todo: check efficiency of lookup table versus switch table
    // Also consider placing calls to addr_mod() in next switch table
    if (bit27 == 0) {
        switch (op) {
            case opcode0_rpd:
            case opcode0_rpl:
            case opcode0_rpt:
                // special instr format
                break;
            default:
                TPR.CA = ip->offset;
                addr_mod(ip);       // note that ip == &cu.IR
        }
    } else {
        switch (op) {
            case opcode1_a4bd:
            case opcode1_a6bd:
            case opcode1_a9bd:
                break;
            default:
                TPR.CA = ip->offset;
                addr_mod(ip);       // note that ip == &cu.IR
        }
    }
    
    if (bit27 == 0) {
        switch (op) {
            case opcode0_lda: {
                int ret = fetch_op(ip, &reg_A);
                if (ret == 0) {
                    IR.zero = reg_A == 0;
                    IR.neg = bit36_is_neg(reg_A);
                }
                printf("LDA: A <= M[%u]: %Lu\n", TPR.CA, reg_A);
                return ret;
                break;
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
            case opcode0_stz:   // store zero
                return store_word(TPR.CA, 0);
            case opcode0_stq:   // Store Q register
                return store_word(TPR.CA, reg_Q);
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
                t_uint64 word1;
                int ret = fetch_op(ip, &word1);
                if (ret == 0) {
                    IR.zero = word1 == reg_Q;
                    if (word1 == reg_Q)
                        ret = store_word(TPR.CA, reg_A);
                }
                return ret;
            }
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
#if 0
                int ret;
                t_uint64 word;
                if ((ret = fetch_op(ip, &word)) == 0) {
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                    ret = store_word(reg_X[n], word);
                }
                return ret;
#endif
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
            case opcode0_sscr: { // priv
                // set system controller register (to value in AQ)
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                uint y = getbits36(TPR.CA, 0, 2);       // BUG: just mask off
                uint ea = y << 15;
                debug_msg("OPU::opcode::sscr", "EA is 0%04o\n", ea);
                debug_msg("OPU::opcode::sscr", "CA is 0%04o (0%03ox=>0%03o)\n", TPR.CA, (TPR.CA >> 3), TPR.CA & ~7);
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    debug_msg("OPU::opcode::sscr", "mode register selected\n");
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0020) {
                    debug_msg("OPU::opcode::sscr", "port zero selected\n");
                    scu_set_mask(TPR.CA, 0);
                } else if ((TPR.CA & ~7) == ea + 0120)
                    scu_set_mask(TPR.CA, 1);
                else if ((TPR.CA & ~7) == ea + 0220)
                    scu_set_mask(TPR.CA, 2);
                else if ((TPR.CA & ~7) == ea + 0320)
                    scu_set_mask(TPR.CA, 3);
                else if ((TPR.CA & ~7) == ea + 0420)
                    scu_set_mask(TPR.CA, 4);
                else if ((TPR.CA & ~7) == ea + 0520)
                    scu_set_mask(TPR.CA, 5);
                else if ((TPR.CA & ~7) == ea + 0620)
                    scu_set_mask(TPR.CA, 6);
                else if ((TPR.CA & ~7) == ea + 0720) {
                    debug_msg("OPU::opcode::sscr", "port seven selected\n");
                    scu_set_mask(TPR.CA, 7);
                } else if ((TPR.CA & ~7) == ea + 0030) {
                    // interrupts
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    // calendar
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    // calendar
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    debug_msg("OPU::opcode::sscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else {
                    debug_msg("OPU::opcode::sscr", "bad argument, CA 0%o\n", TPR.CA);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                debug_msg("OPU::opcode::sscr", "A = %Lo\n", reg_A);
                debug_msg("OPU::opcode::sscr", "Q = %Lo\n", reg_Q);
                return ret;
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
            case opcode0_lcpr:  // load central processor reg (priv)
                debug_msg("OPU::opcode::lcpr", "unimplemented (and perhaps not relevant\n");
                cancel_run(STOP_WARN);
                return 0;
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
                int n = op & 07;
                t_uint64 word;
                int ret = fetch_op(ip, &word);
                if (ret == 0) {
                    reg_X[n] = word >> 18;  // reg is 18 bits
                    IR.zero = reg_X[n] == 0;
                    IR.neg = bit18_is_neg(reg_X[n]);
                }
                return ret;
            }
            case opcode0_qlr: {
                unsigned n = getbits36(TPR.CA, 11, 7);
                reg_Q = lrotate(reg_Q, n);
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return 0;
            }
            case opcode0_qls: { // Q reg left shift
                int n = getbits36(TPR.CA, 11, 7);
                int init_neg = bit36_is_neg(reg_Q);
                reg_Q = (reg_Q << n) & MASK36;
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_qrl: {
                int n = getbits36(TPR.CA, 11, 7);
                reg_Q >>= n;
                IR.zero = reg_Q == 0;
                IR.neg = bit36_is_neg(reg_Q);
                return 0;
#if 0
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
                int n = getbits36(TPR.CA, 11, 7);
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
            case opcode0_alr: {
                unsigned n = getbits36(TPR.CA, 11, 7);
                reg_A = lrotate(reg_A, n);
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            }
            case opcode0_als: { // A reg left shift
                int n = getbits36(TPR.CA, 11, 7);
                int init_neg = bit36_is_neg(reg_A);
                reg_A = (reg_A << n) & MASK36;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_arl: { // A reg right logical shift
                int n = getbits36(TPR.CA, 11, 7);
                reg_A >>= n;
                IR.zero = reg_A == 0;
                IR.neg = bit36_is_neg(reg_A);
                return 0;
            }
            case opcode0_llr: {     // Long left rotate
                int init_neg = bit36_is_neg(reg_A);
                unsigned n = getbits36(TPR.CA, 11, 7);
                lrotate72(&reg_A, &reg_Q, n);
                IR.zero = reg_A == 0 && reg_Q == 0;
                IR.neg = bit36_is_neg(reg_A);
                IR.carry = init_neg != IR.neg;
                return 0;
            }
            case opcode0_lls: {     // Long left shift
                int init_neg = bit36_is_neg(reg_A);
                unsigned n = getbits36(TPR.CA, 11, 7);
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
            // case opcode0_adlx6:  // Add logical to X[n]
            // ldaq:    // Load AQ reg
            // cana:    // Comparative AND with A reg
            // limr ??
            // ldo ??
            // camp2 ??
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
            case opcode0_cams: {    // Clear Associative Memory Segments
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                int clear = getbits36(TPR.CA, 15, 1);
                int enable = getbits36(TPR.CA, 16, 2);
                int i;
                for (i = 0; i < 16; ++i) {
                    SDWAM[i].is_full = 0;
                    SDWAM[i].use = i;
                    if (clear) {
                        ret = 1;
                        debug_msg("OPU::cams", "Clear mode is unimplemented\n");
                        cancel_run(STOP_BUG);
                        // St the full/empty bits of all cache blocks to empty
                        // -- what are cache blocks?
                    }
                    if (enable == 2)
                        SDWAM[i].enabled = 1;
                    else if (enable == 1)
                        SDWAM[i].enabled = 0;
                }
                return ret;
            }
            case opcode0_tra:
                PPR.IC = TPR.CA;
                PPR.PSR = TPR.TSR;
                return 0;
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
                    debug_msg("OPU::opcode::tnz", "transfer to %u\n", TPR.CA);
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                } else
                    debug_msg("OPU::opcode::tnz", "no transfer (would have been to %d)\n", TPR.CA);
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
            case opcode0_trc:
                if (IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode0_adx0:  // Add to X[n]
            case opcode0_adx1:
            case opcode0_adx2:
            case opcode0_adx3:
            case opcode0_adx4:
            case opcode0_adx5:
            case opcode0_adx6:
            case opcode0_adx7: {
                int n = op & 07;
                return op_add(ip, &reg_X[n]);
            }
            case opcode0_asa: { // Add Stored to A reg
                t_uint64 word = reg_A;
                int ret = op_add(ip, &word);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
            }
            case opcode0_xed: {
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xed invokes xed
                // todo: handle rpd repeats
                t_uint64 word0;
                t_uint64 word1;
                t_uint64 addr;
                instr_t IR;
                if (decode_ypair_addr(ip, &addr)) {
                    debug_msg("OPU::opcode::xed", "decode addr: error or fault\n");
                    return 1;   // faulted
                }
                // -----------
                if (fetch_instr(addr, &IR) != 0) {
                    debug_msg("OPU::opcode::xed", "fetch even: error or fault\n");
                    return 1;   // faulted
                }
                debug_msg("OPU::opcode::xed", "executing even instr at %ld\n", (long) addr);
                if (do_op(&IR) != 0) {
                    debug_msg("OPU::opcode::xed", "fault or error executing even instr\n");
                    return 1;
                }
                // -----------
                if (cpu.xfr) {
                    debug_msg("OPU::opcode::xed", "transfer instr executed, not doing odd instr\n");
                } else {
                    ++ addr;
                    if (fetch_instr(addr, &IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fetch odd: error or fault\n");
                        return 1;   // faulted
                    }
                    debug_msg("OPU::opcode::xed", "executing odd instr at %ld\n", (long) addr);
                    if (do_op(&IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
                debug_msg("OPU::opcode::xed", "finished\n");
                break;
            }
            default:
                debug_msg("OPU", "Unimplemented opcode 0%0o(0)\n", op);
                fault_gen(oob_fault);   // todo: mechanism to bomb back to simh
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
            case opcode1_camp: {    // Clear Associative Memory Pages
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                int sel_clear = getbits36(TPR.CA, 15, 1);
                int enable = getbits36(TPR.CA, 16, 2);
                int i;
                for (i = 0; i < 16; ++i) {
                    PTWAM[i].is_full = 0;
                    PTWAM[i].use = i;
                    if (sel_clear) {
                        ret = 1;
                        debug_msg("OPU::camp", "Selective Clear mode is Unimplemented\n");
                        cancel_run(STOP_BUG);
                        // for any cache block for which upper 15 bits of the dir [are]
                        // entry equal PTWAM(i).ADDR0..13, 
                        //  set full/empty to empty
                        // -- what are cache blocks and dirs?
                    }
                    if (enable == 2)
                        PTWAM[i].enabled = 1;
                    else if (enable == 1)
                        PTWAM[i].enabled = 0;
                }
                return ret;
            }
            default:
                debug_msg("OPU", "Unimplemented opcode 0%0o(1)\n", op);
                fault_gen(oob_fault);   // todo: better mechanism to bomb back to simh
        }
    }
    return 0;
}

static int op_add(instr_t *ip, t_uint64 *dest)
{
    int ret;
    t_uint64 word;
    t_uint64 result;
    if ((ret = fetch_op(ip, &word)) != 0)
        return ret;
    
    uint sign1 = word >> 35;
    uint sign2 = *dest >> 35;
    result = word + *dest;
    uint signr = result >> 35;
    if ((result >> 36) != 0) {
        IR.carry = 1;
        result &= ~ (~(t_uint64)0 << 36);
    } else {
        IR.carry = 0;
    }
    IR.zero = result == 0;
    IR.neg = ! signr;
    if (sign1 == sign2 && signr != sign1) {
        IR.overflow = 1;
        if (IR.overflow_mask == 0) {
            fault_gen(overflow_fault);
            return 1;
        }
    }

    return 0;
}

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

