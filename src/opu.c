#include "hw6180.h"

extern t_uint64 M[];    /* memory */ // BUG

// #define bit36_is_neg(x) ( ((x)>>35) == 1 )
#define bit36_is_neg(x) (((x) & (((t_uint64)1)<<35)) != 0)

// ============================================================================

static int do_op(instr_t *ip);

void execute_instr(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)
    do_op(&cu.IR);
}

// ============================================================================

static int op_add(instr_t *ip, t_uint64 *operand);
static int op_and(instr_t *ip, t_uint64 *op, t_uint64 *op2, t_uint64 *dest1, t_uint64 *dest2);

static int do_op(instr_t *ip)
{
    // Returns non-zero on error or non-group-7  fault

    uint op = ip->opcode;
    char *opname = opcodes2text[op];

    int bit27 = op % 2;
    op >>= 1;
    if (opname == NULL) {
        debug_msg("OPU", "Illegal opcode 0%0o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    } else {
        debug_msg("OPU", "Opcode 0%0o(%d) -- %s\n", op, bit27, opname);
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
                int ret = fetch_word(TPR.CA, &reg_A);
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
            case opcode0_sta: {
                t_uint64 word;
                int ret = fetch_word(reg_A, &word);
                if (ret == 0)
                    ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_stac: {
                t_uint64 word;
                int ret = fetch_word(TPR.CA, &word);
                if (ret == 0 && word == 0)
                    if ((ret = fetch_word(reg_A, &word)) == 0)
                        ret = store_word(TPR.CA, word);
                return ret;
            }
            case opcode0_stacq: {
                t_uint64 word1;
                int ret = fetch_word(TPR.CA, &word1);
                if (ret == 0) {
                    t_uint64 word2;
                    if ((ret = fetch_word(reg_Q, &word2)) == 0) {
                        IR.zero = word1 == word2;
                        if (word1 == word2)
                            if ((ret = fetch_word(reg_A, &word1)) == 0)
                                ret = store_word(TPR.CA, word1);
                    }
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
                int ret;
                t_uint64 word;
                if ((ret = fetch_word(TPR.CA, &word)) == 0) {
                    IR.zero = word == 0;
                    IR.neg = bit36_is_neg(word);
                    ret = store_word(reg_X[n], word);
                }
                return ret;
            }
            case opcode0_fld: {
                int ret;
                t_uint64 word;
                if ((ret = fetch_word(TPR.CA, &word)) == 0) {
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
                if (get_addr_mode != ABSOLUTE_mode) {
                    fault_gen(illproc_fault);
                    return 1;
                }
                uint y = getbits36(TPR.CA, 0, 2);       // BUG: just mask off
                uint ea = y << 15;
                if ((TPR.CA & ~ 0177) == ea)
                    ; // SC mode reg
                else if ((TPR.CA & ~ 0177) == ea + 0010)
                    ; // SC config reg
                else if ((TPR.CA & ~ 0177) == ea + 0020)
                    ; // port 0
                else if ((TPR.CA & ~ 0177) == ea + 0120)
                    ; // port 1
                else if ((TPR.CA & ~ 0177) == ea + 0220)
                    ;
                else if ((TPR.CA & ~ 0177) == ea + 0320)
                    ;
                else if ((TPR.CA & ~ 0177) == ea + 0420)
                    ;
                else if ((TPR.CA & ~ 0177) == ea + 0520)
                    ;
                else if ((TPR.CA & ~ 0177) == ea + 0620)
                    ;
                else if ((TPR.CA & ~ 0177) == ea + 0720)
                    ; // port 7
                else if ((TPR.CA & ~ 0177) == ea + 0030)
                    ; // interrupts
                else if ((TPR.CA & ~ 0177) == ea + 0040)
                    ; // calendar
                else if ((TPR.CA & ~ 0177) == ea + 0050)
                    ; // calendar
                else if ((TPR.CA & ~ 0177) == ea + 0060)
                    ; // store unit mode reg
                else if ((TPR.CA & ~ 0177) == ea + 0070)
                    ;
                else {
                    // error
                }
                debug_msg("OPU::opcode::sscr", "unimplemented\n");
                fault_gen(oob_fault);   // todo: mechanism to bomb back to simh
                return 1;
            }
            // case opcode0_ldt:    // load timer reg (priv)
            // case opcode0_lcpr:   // load central processor reg (priv)
            // case opcode  1_camp: // Clear Associative Memory Pages
            // case opcode  1_cams: // Clear Associative Memory Segments
            // case opcode0_stz:    // store zero
            // case opcode0_ldq:    // load Q reg
            // case opcode0_qls:    // Q reg left shift
            // case opcode0_qrl:
            // case opcode0_als:    // A reg left shift
            // case opcode0_orsq:   // OR to strorage from Q reg
            // case opcode0_adlx6:  // Add logical to X[n]
            // case opcode0_ldx5:   // Load X[n]
            // ldaq:    // Load AQ reg
            // cana:    // Comparative AND with A reg
            // stq:     // Store Q register
            // opcode0_llr:     // Long left rotate
            // limr ??
            // ldo ??
            // camp2 ??
            // tsx2 // transfer and set X[n]
            // arl  // A reg right logical shift
            // asa  // And Stored to A reg
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
                    PPR.IC = TPR.CA;
                    PPR.PSR = TPR.TSR;
                }
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
                break;
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
                return 0;
                IR.truncation = 0;
#endif
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
    if ((ret = fetch_word(TPR.CA, &word)) != 0)
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
        if ((ret = fetch_word(TPR.CA, &word1)) != 0)
            return ret;
    } else {
        if ((ret = fetch_pair(TPR.CA, &word1, &word2)) != 0)
            return ret;
    }
    *dest1 = word1 & *op1;
    if (op2 != NULL && dest2 != NULL)
        *dest2 = word2 & *op2;

    IR.zero = (*dest1 == 0) && (dest2 == NULL || *dest2 == 0);
    IR.neg = bit36_is_neg(*dest1);  // dest1 presumed to hold bit zero
    return 0;
}

