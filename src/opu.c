#include "hw6180.h"

// extern t_uint64 M[]; /* memory */ // BUG

// ============================================================================

static inline t_uint64 lrotate36(t_uint64 x, unsigned n);
static inline void lrotate72(t_uint64* ap, t_uint64* bp, unsigned n);
static inline t_int64 negate36(t_uint64 x);
static inline int negate72(t_uint64* a, t_uint64* b);

static int do_op(instr_t *ip);
static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);
static int add36(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add18(t_uint64 a, t_uint64 b, t_uint64 *dest);
static int add72(t_uint64 a, t_uint64 b, t_uint64* dest1, t_uint64* dest2);
static int32 sign18(t_uint64 x);
static int do_epp(int epp);

// BUG: move externs to hdr file
extern int scu_cioc(t_uint64 addr);
extern int scu_set_mask(t_uint64 addr, int port);
extern int scu_get_mask(t_uint64 addr, int port);
extern int scu_set_cpu_mask(t_uint64 addr);
extern int scu_get_calendar(t_uint64 addr);
extern int addr_mod(instr_t *ip);
extern int activate_timer();

// ============================================================================

static char* instr2text(const instr_t* ip)
{
    static char buf[100];
    uint op = ip->opcode;
    char *opname = opcodes2text[op];
    if (opname == NULL) {
        strcpy(buf, "<illegal instr>");
    } else {
        if (ip->pr_bit == 0) {
            sprintf(buf, "%s, offset 0%06o(%+d), inhibit %u, pr=N, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                ip->addr.offset, ip->addr.soffset, 
                ip->inhibit, ip->tag, ip->tag >> 4, ip->tag & 017);
        } else {
            sprintf(buf, "%s, PR %d, offset 0%06o(%+d), inhibit %u, pr=Y, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                ip->addr.pr, ip->addr.offset, ip->addr.soffset, 
                ip->inhibit, ip->tag, ip->tag >> 4, ip->tag & 017);
        }
    }
    return buf;
}


char* print_instr(t_uint64 word)
{
    instr_t instr;
    decode_instr(&instr, word);
    return instr2text(&instr);
}


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
    if (ip->is_value) {
        *wordp = TPR.CA;
        return 0;
    }
    return fetch_word(TPR.CA, wordp);
}


// ============================================================================


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
        debug_msg("OPU", "Opcode 0%0o(%d) -- %s\n", op, bit27, instr2text(ip));
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
                addr_mod(ip);       // note that ip == &cu.IR
        }
    } else {
        switch (op) {
            case opcode1_a4bd:
            case opcode1_a6bd:
            case opcode1_a9bd:
                break;
            default:
                addr_mod(ip);       // note that ip == &cu.IR
        }
    }
    
    if (bit27 == 0) {
        switch (op) {
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
                uint y = TPR.CA - TPR.CA % 2;
                int ret = store_word(y, reg_A);
                if (ret == 0)
                    ret = store_word(y+1, reg_Q);
                return ret;
            }
            case opcode0_era: { // AOR to A
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
            case opcode0_cioc: { // priv
                // Connect I/O channel
                debug_msg("OPU", "CIOC\n");
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                return scu_cioc(TPR.CA);
            }
            case opcode0_smcm: { // priv
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                return scu_set_cpu_mask(TPR.CA);
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
                    debug_msg("OPU::opcode::sscr", "bad argument, CA 0%Lo\n", TPR.CA);
                    cancel_run(STOP_BUG);
                    ret = 1;
                    // error
                }
                return ret;
            }
            case opcode0_rscr: { // priv
                // read system controller register (to AQ)
                if (get_addr_mode() != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                int ret = 0;
                // uint y = getbits36(TPR.CA, 0, 2);        // BUG: CA is 18 bits, not 36
                uint y = (TPR.CA >> 16) & 3;    // 18bit CA
                uint ea = y << 15;
                debug_msg("OPU::opcode::rscr", "EA is 0%04o\n", ea);
                debug_msg("OPU::opcode::rscr", "CA is 0%04Lo (0%03Lo=>0%03Lo)\n", TPR.CA, (TPR.CA >> 3), TPR.CA & ~7);
                if ((TPR.CA & ~7) == ea) {
                    ; // SC mode reg
                    debug_msg("OPU::opcode::rscr", "mode register selected\n");
                    debug_msg("OPU::opcode::rscr", "unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0010) {
                    ; // SC config reg
                    debug_msg("OPU::opcode::rscr", "sys config switches unimplemented\n");
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
                    debug_msg("OPU::opcode::rscr", "interrupts unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0040) {
                    ret = scu_get_calendar(TPR.CA);
                } else if ((TPR.CA & ~7) == ea + 0050) {
                    ret = scu_get_calendar(TPR.CA);
                } else if ((TPR.CA & ~7) == ea + 0060) {
                    // store unit mode reg
                    debug_msg("OPU::opcode::rscr", "store unit mode reg unimplemented\n");
                    cancel_run(STOP_BUG);
                    ret = 1;
                } else if ((TPR.CA & ~7) == ea + 0070) {
                    debug_msg("OPU::opcode::rscr", "store unit mode reg unimplemented\n");
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
            case opcode0_lcpr: {    // load central processor reg (priv)
                int ret = 0;
                t_uint64 word;
                switch (ip->tag) {      // no addr modifications
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
                        complain_msg("OPU::opcode::lcpr", "Bad tag 0%o\n", ip->tag);
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
                    SDWAM[i].is_full = 0;
                    SDWAM[i].use = i;
                    PTWAM[i].is_full = 0;
                    PTWAM[i].use = i;
                }
                // todo: If cache is enabled, reset all cache colume and level full flags
                DSBR.addr = getbits36(word1, 0, 24);
                DSBR.bound = getbits36(word2, 0, 14);   // 37-36- 1
                DSBR.u = getbits36(word2, 18, 1);   // 50-36-1
                DSBR.stack = getbits36(word2, 23, 12);  // 60-36-1
                return 0;
            }
            case opcode0_epp0:
                return do_epp(0);
            case opcode0_epp2:
                return do_epp(2);
            case opcode0_epp4:
                return do_epp(4);
            case opcode0_epp6:
                return do_epp(6);
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
                IR.neg = bit36_is_neg(reg_A);
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
            case opcode0_nop:
            case opcode0_puls1:
            case opcode0_puls2:
                // BUG: certain address forms may generate faults -- so addr_mod should gen faults
                return 0;
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
            case opcode0_tze:
                if (IR.zero) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
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
            case opcode0_trc:
                if (IR.carry) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
            case opcode0_ada:
                return op_add(ip, &reg_A);
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
                        ret = add72(reg_A, reg_Q, &word1, &word2);
                return ret;
            }
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
                debug_msg("OPU::opcode::xed", "executing even instr at 0%Lo\n", addr);
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
                    debug_msg("OPU::opcode::xed", "executing odd instr at 0%Lo\n", addr);
                    if (do_op(&IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
                debug_msg("OPU::opcode::xed", "finished\n");
                break;
            }
            default:
                complain_msg("OPU", "Unimplemented opcode 0%0o(0)\n", op);
                cancel_run(STOP_BUG);
                return 1;
        }
    } else {
        switch (op) {
            case opcode1_ttn:
                if (IR.tally_runout) {
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
                return 0;
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
                //int sel_clear = getbits36(TPR.CA, 15, 1);
                //int enable = getbits36(TPR.CA, 16, 2);
                int sel_clear = (TPR.CA >> 2) & 1;  // Bit 15 of 18-bit CA
                int enable = TPR.CA & 3;        // Bits 16 and 17 of 18-bit CA
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
            case opcode1_epp1:
                return do_epp(1);
            case opcode1_epp3:
                return do_epp(4);
            case opcode1_epp5:
                return do_epp(5);
            case opcode1_epp7:
                return do_epp(7);
            default:
                debug_msg("OPU", "Unimplemented opcode 0%0o(1)\n", op);
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
        IR.carry = 1;
        result &= MASK36;
    } else {
        IR.carry = 0;
    }
    IR.zero = result == 0;
    IR.neg = signr;
    if (sign1 == sign2 && signr != sign1) {
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
    uint signr = result >> 17;
    if ((result >> 18) != 0) {
        IR.carry = 1;
        result &= MASK18;
    } else {
        IR.carry = 0;
    }
    IR.zero = result == 0;
    IR.neg = signr;
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

static int add72(t_uint64 a, t_uint64 b, t_uint64* dest1, t_uint64* dest2)
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

static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        debug_msg("OPU::sign18", "0%Lo => 0%o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}

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
    return 0;
}
