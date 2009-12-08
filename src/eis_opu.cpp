/*
    eis_opu.cpp -- implements EIS multi-word instructions.

    Called by routines in opu.c

    Makes heavy use of functions in eis_desc.cpp for descriptor addressing
    and fetches/puts.

*/

using namespace std;
#include <iostream>

#include <ctype.h>  // for isprint
#include "hw6180.h"
#include "eis.hpp"

// ============================================================================

int op_move_alphanum(const instr_t* ip, int fwd)
{
    const char* moi = (fwd) ? "OPU::mlr" : "OPU::mrl";

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

    char msg_buf[100];
    alpha_desc_t desc1(ip->mods.mf1, word1, fwd);
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    alpha_desc_t desc2(mf2, word2, fwd);
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));


    int ret = 0;

    while (desc2.n() > 0) {
        uint nib;
        if (desc1.n() == 0)
            nib = fill & MASKBITS(desc2.width());
        else {
            ret = desc1.get(&nib);
            if (ret != 0) {
                ret = 1;
                break;
            }
        }
        ret = desc2.put(nib);
        if (ret != 0) {
            ret = 2;
            break;
        }
    }
    if (ret == 0) {
        IR.truncation = desc1.n() != 0;
        if (IR.truncation && t) {
            fault_gen(overflow_fault);  // truncation
            ret = 1;
        }
    }
    // write unsaved data (if any)
    if (ret < 2) {
        if (desc2.flush() != 0)
            ret = 2;
    }

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

// ============================================================================

int op_tct(const instr_t* ip, int fwd)
{
    const char* moi = (fwd) ? "OPU::tct" : "OPU::tctr";

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, NULL, &word2, NULL, &word3) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    alpha_desc_t desc1(ip->mods.mf1, word1, fwd);
    char msg_buf[100];
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
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

    uint n = desc1.n();
    while (desc1.n() > 0) {
        log_msg(DEBUG_MSG, moi, "Remaining length %d\n", desc1.n());
        uint m;
        if (desc1.get(&m) != 0)
            return 1;
        uint t;
        if (get_table_char(addr2, m, &t) != 0) {
            log_msg(WARN_MSG, moi, "Unable to read table\n");
            //if (!fwd) { --opt_debug; -- cpu_dev.dctrl; }
            return 1;
        }
        if (t != 0) {
            int idx = (fwd) ? n - desc1.n() - 1 : desc1.n();    // reverse scan also stores i-1 not N1-i !
            int i = n - desc1.n() - 1;
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
    log_msg(INFO_MSG, moi, "finished.\n");
    PPR.IC += 4;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return 0;
}

// ============================================================================

extern int op_mvt(const instr_t* ip)
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

    eis_desc_t desc1;
    if (decode_eis_alphanum_desc(&desc1, &ip->mods.mf1, word1, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 1.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_t desc2;
    if (decode_eis_alphanum_desc(&desc2, &mf2, word2, 0, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    uint addr3;
    if (get_eis_indir_addr(word3, &addr3) != 0)
        return 1;

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));
    log_msg(DEBUG_MSG, moi, "translation table at %#llo => %#o\n", word3, addr3);

    int ret = 0;

    // uint n = desc1.n;
    while (desc2.n > 0) {
        uint m;
        if (desc1.n == 0)
            m = fill & MASKBITS(desc2.nbits);
        else
            if (eis_desc_get(&desc1, &m) != 0) {
                ret = 1;
                break;
            }
        uint t;
        if (get_table_char(addr3, m, &t) != 0) {
            log_msg(WARN_MSG, moi, "Unable to read table\n");
            ret = 1;
            break;
        }
        if (eis_desc_put(&desc2, t) != 0) {
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
        if (eis_desc_flush(&desc2) != 0)
            return 1;

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    PPR.IC += 4;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret;
}

// ============================================================================

int op_cmpc(const instr_t* ip)
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

    eis_desc_t desc1;
    if (decode_eis_alphanum_desc(&desc1, &ip->mods.mf1, word1, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 1.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_t desc2;
    // force (ignored) type of descriptor 2 to match descriptor 1
    unsigned ta = getbits36(word1, 21, 2);
    word2 = setbits36(word2, 21, 2, ta);
    if (decode_eis_alphanum_desc(&desc2, &mf2, word2, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));

    int ret = 0;

    uint nib1, nib2;
    while (desc1.n > 0 || desc2.n > 0) {
        if (desc1.n == 0)
            nib1 = fill & MASKBITS(desc1.nbits);
        else
            if (eis_desc_get(&desc1, &nib1) != 0) {
                ret = 1;
                break;
            }
        if (desc2.n == 0)
            nib2 = fill & MASKBITS(desc2.nbits);
        else
            if (eis_desc_get(&desc2, &nib2) != 0) {
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


int op_btd(const instr_t* ip)
{
    const char* moi = "OPU::btd";

    uint sign_ctl = ip->addr >> 17;

    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    num_desc_t desc1(ip->mods.mf1, word1, 1);
    uint tn = getbits36(word1, 21, 1);
    word2 = setbits36(word2, 21, 1, tn);    // force (ignored) type of desc 2 to match desc 1
    num_desc_t desc2(mf2, word2, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));

    if (desc2.scaling_factor != 0) {
        log_msg(ERR_MSG, moi, "Descriptor 2 scaling factor must be zero\n");
        fault_gen(illproc_fault);
        return 1;
    }
    if (desc2.s == 0) {
        log_msg(ERR_MSG, moi, "Descriptor 2 conversion type must not be zero\n");
        fault_gen(illproc_fault);
        return 1;
    }
    if (desc1.n() == 0 || desc1.n() > 8) {
        log_msg(ERR_MSG, moi, "Descriptor 1 length must in range 1..8\n");
        fault_gen(illproc_fault);
        return 1;
    }

    // First op is a binary number between 9bits and 72bits.
    // Second op is bcd with either 4bit or 9bit nibbles.

    // First, read out the nibbles into src_hi and src_lo -- we might need to negate
    t_uint64 src_hi = 0, src_lo = 0;
    // int n = desc1.n;
    for (int n_quads = 0; desc1.n() > 0; ++ n_quads) {
        uint nib;
        if (desc1.get(&nib) != 0) { // must fetch when needed
            return 1;
            break;
        }
        if (n_quads >= 9) {
            src_hi <<= 4;
            src_hi |= getbits36(src_lo, 0, 4);
        }
        src_lo <<= 4;
        src_lo |= nib;
    }
    src_lo &= MASK36;
    log_msg(INFO_MSG, moi, "source binary is { %#llo, %012llo}\n", src_hi, src_lo);

    int negate = 0;
    if (desc2.s != 03)
        if (bit36_is_neg(src_hi)) {
            negate72(&src_hi, &src_lo);
            negate = 1;
            log_msg(INFO_MSG, moi, "source binary is negative, negating yields { %#llo, %012llo}\n", src_hi, src_lo);
        }
        
    // We'll initially compute the results into rhi and rlo via 4bit nibbles.  We only need just over 72 bits;
    // note that we'll use all 64 bits of rlo, not use 36 of them.
    t_uint64 rlo = 0;
    uint rhi = 0;

    // Use "shift and add three" algorithm
    for (int i  = 0; i < 72; ++i) {
        // Shift current msb out of nibble
        uint msb;
        if (i < 36) {
            msb = bit36_is_neg(src_hi);
            src_hi <<= 1;
        } else {
            msb = bit36_is_neg(src_lo);
            src_lo <<= 1;
        }
        if (i >= 64) {
            rhi <<= 1;
            rhi |= (rlo >> 63);
        }
        rlo <<= 1;
        rlo |= msb;
        // Finished?
        if (i == 72 - 1)
            break;
        // Check low bits of result for 4bit quads >= 5
        for (int j = 0; j < 64/4; ++j)
            if (((rlo >> (j*4)) & 0xf) >= 5) {
                rlo += (3 << (j*4));
                if (j == 64/4 - 1) {
                    rhi += 1;   // overflow
                    log_msg(WARN_MSG, moi, "intermediate overflow not tested\n");
                    cancel_run(STOP_IBKPT);
                }
            }
        // Check hi bits of result for 4bit quads >= 5
        for (int j = 0; j < 1; ++j)
            if (((rhi >> (j*4)) & 0xf) >= 5)
                rhi += (3 << (j*4));
    }
    
    int ndigits = 0;
    for (int i = 0 ; i < 64/4; ++i)
        if (((rlo >> (i*4)) & 0xf) != 0)
            ndigits = i + 1;
    for (int i = 0 ; i < 3; ++i)
        if (((rhi >> (i*4)) & 0xf) != 0)
            ndigits = 64/4 + i + 1;
        
    log_msg(INFO_MSG, moi, "intermediate result is %d digits: { hi=%x, low=%llx}\n", ndigits, rhi, rlo);    // hex format displays bcd as decimal

    IR.zero = rlo == 0 && rhi == 0;
    IR.neg = negate;

    // int needed = ndigits + (desc2.num.s != 3);
    // if (needed > desc2.n) ... overflow
    
    int ret = 0;

    // put_eis_an_rev doesn't exist...
    uint results[64];
    int n = 0;

    if (desc2.s == 2)
        // Push trailing sign
        if (n < desc2.n()) {
            if (negate)
                results[n++] = (desc2.width() == 4) ? 015 : (015 | 040);
            else if (desc2.width() == 4)
                results[n++] = (sign_ctl) ? 013 : 014;
            else
                results[n++] = 013 | 040;
        }

    while (n < desc2.n()) {
        results[n] = rlo & 0xf;
        if (desc2.width() == 9)
            results[n] |= 060;
        ++n;
        rlo >>= 4;
        rlo |= (t_uint64) ((rhi >> 4) & 0xf) << 60;
        rhi >>= 4;
    }

    int overflow = 0;
    if (rlo != 0 || rhi != 0) {
        overflow = 1;
        IR.overflow = 1;
    }

    --n;
    if (desc2.s == 1) {
        // leading sign
        if ((results[n] & 013) != 0)
            overflow = 1;
        else {
            if (negate)
                results[n] = 015;
            else if (desc2.width() == 4)
                results[n] = (sign_ctl) ? 013 : 014;
            else
                results[n] = 013;
            if (desc2.width() == 9)
                results[n] |= 040;
        }
    }
    ++n;

    if (overflow)
        fault_gen(overflow_fault);
    else {
        log_msg(DEBUG_MSG, moi, "Storing %d (aka %d) bytes\n", desc2.n(), n);
        while (desc2.n() > 0) {
            // log_msg(DEBUG_MSG, moi, "Storing byte %d from results[%d]\n", desc2.n(), desc2.n()-1);
            if (desc2.put(results[desc2.n()-1]) != 0) {
                ret = 1;
                break;
            }
        }
    }

    // write unsaved data (if any)
    if (ret == 0)
        if (desc2.flush() != 0)
            return 1;

    PPR.IC += 3;        // BUG: when should we bump IC?  probably not for faults, but probably yes for conditions
    return ret;
}

// ============================================================================

int op_cmpb(const instr_t* ip)
{
    const char* moi = "OPU::cmpb";

    uint fill = ip->addr >> 17;
    // uint trunc_enable = (ip->addr >> 8) & 1; // Ignored for cmpb
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_desc_t desc1;
    if (decode_eis_bit_desc(&desc1, &ip->mods.mf1, word1, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 1.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_t desc2;
    if (decode_eis_bit_desc(&desc2, &mf2, word2, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));
    log_msg(DEBUG_MSG, moi, "fill bit %d\n", fill);

    int ret = 0;

    flag_t nib1, nib2;  // for this instruction, these are bit valued
    int zero = 1;
    int carry = 1;
    while (desc1.n > 0 || desc2.n > 0) {
        if (desc1.n == 0)
            nib1 = fill & MASKBITS(desc1.nbits);
        else
            if (eis_desc_get(&desc1, &nib1) != 0) { // must fetch when needed
                ret = 1;
                break;
        }
        if (desc2.n == 0)
            nib2 = fill & MASKBITS(desc2.nbits);
        else
            if (eis_desc_get(&desc2, &nib2) != 0) { // must fetch when needed
                ret = 1;
                break;
            }
        if (nib1 < nib2) {
            zero = 0;
            carry = 0;
            break;
        } else if (nib1 > nib2) {
            zero = 0;
            break;
        }
    }
    if (ret == 0) {
        IR.zero = zero;
        IR.carry = carry;
    }

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_WARN);
    log_msg(DEBUG_MSG, moi, "finished.\n");
    PPR.IC += 3;        // BUG: when should we bump IC?  probably not for seg faults, but probably yes for overflow
    return ret;
}

// ============================================================================

int op_scm(const instr_t* ip, int fwd)
{
    const char* moi = fwd ? "OPU::scm" : "OPU::scmr";

    cpu.irodd_invalid = 1;

    uint mask = ip->addr >> 9;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, &word3) != 0)
        return 1;

    eis_desc_t desc1;
    if (decode_eis_alphanum_desc(&desc1, &ip->mods.mf1, word1, 1, fwd) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 1.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_t desc2;
    // force (ignored) type of desc2 to match desc1
    unsigned ta = getbits36(word1, 21, 2);
    word2 = setbits36(word2, 21, 2, ta);
    // force (ignored) count of desc2 to one
    word2 = setbits36(word2, 24, 12, 1);
    if (decode_eis_alphanum_desc(&desc2, &mf2, word2, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    uint y3;
    if (get_eis_indir_addr(word3, &y3) != 0)
            return 1;
    
    log_msg(DEBUG_MSG, moi, "mask = %03o\n", mask);
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", eis_desc_to_text(&desc1));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));
    log_msg(DEBUG_MSG, moi, "y3: %06o\n", y3);

    uint test_nib;
    if (mf2.id == 0 && mf2.reg == 3) {  // 3 is ",du"
        test_nib = getbits36(word2, 0, 9);
        log_msg(DEBUG_MSG, moi, "test char specified via special-case ',du'\n");
    } else {
        log_msg(DEBUG_MSG, moi, "Getting test char\n");
        if (eis_desc_get(&desc2, &test_nib) != 0)
            return 1;
    }
    if (opt_debug) {
        if (isprint(test_nib))
            log_msg(DEBUG_MSG, moi, "test char: %03o '%c'\n", test_nib, test_nib);
        else
            log_msg(DEBUG_MSG, moi, "test char: %03o\n", test_nib);
    }

    int ret = 0;
    uint n = desc1.n;
    uint i;
    for (i = 0; i < n; ++i) {
        uint nib;
        ret = eis_desc_get(&desc1, &nib);
        if (ret != 0)
            break;
        uint z = ~mask & (nib ^ test_nib);
        log_msg(DEBUG_MSG, moi, "compare nibble %#o(%+d): value %03o yields %#o\n", i, i, nib, z);
        if (z == 0)
            break;
    }
    if (ret == 0) {
        IR.tally_runout = i == n;
        // t_uint64 word = setbits36(0, 12, 24, i);
        t_uint64 word = i;
        log_msg(DEBUG_MSG, moi, "TRO=%d (%s); writing %#o(%+d) to abs addr %#o\n", IR.tally_runout, IR.tally_runout ? "not found" : "found", i, i, y3);
        ret = store_abs_word(y3, word);
    }

    PPR.IC += 4;        // BUG: check other eis mw instr to make sure IC update is after op fetches (affects ic addr mode and prob restart)
    return ret;
}

// ============================================================================

int op_csl(const instr_t* ip)
{
    // Combine bit strings left

    const char* moi = "OPU::csl";

    uint fill = ip->addr >> 17;
    uint bolr = (ip->addr >> 9) & MASKBITS(4);
    flag_t t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));
    const char *ops[16] = { "clear", "and", "x&!y", "x", "!x&y", "y", "xor", "or", "!or", "!xor", "!y", "!x&y", "!x", "x|!y", "nand", "set" };
    log_msg(INFO_MSG, moi, "bool oper: %#o =b%d%d%d%d (%s), fill: %d\n", bolr, (bolr>>3)&1, (bolr>>2)&1, (bolr>>1)&1, bolr&1, ops[bolr], fill);

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    eis_desc_t desc1;
    if (decode_eis_bit_desc(&desc1, &ip->mods.mf1, word1, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 1.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    log_msg(INFO_MSG, moi, "desc1: %s\n", eis_desc_to_text(&desc1));
    eis_desc_t desc2;
    if (decode_eis_bit_desc(&desc2, &mf2, word2, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    // BUG: eis_alpha_desc_to_text is magic and sees real type
    log_msg(INFO_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));

    int ret = 0;

    flag_t all_zero = 1;
    while (desc2.n > 0) {
        flag_t bit1, bit2;
        if (desc1.n == 0)
            bit1 = fill;
        else
            if (eis_desc_get(&desc1, &bit1) != 0) {
                ret = 1;
                break;
            }
        // eis_desc_val() won't ever advance the pointer
        if (eis_desc_val(&desc2, &bit2) != 0) { // must fetch when needed
            ret = 1;
            break;
        }
        flag_t r = (bolr >> (3 - ((bit1 << 1) | bit2))) & 1;    // like indexing into a truth table
        log_msg(DEBUG_MSG, moi, "nbits1=%d, nbits2=%d; %d op(%#o) %d => %d\n", desc1.n, desc2.n, bit1, bolr, bit2, r);
        if (eis_desc_put(&desc2, r) != 0) {
            ret = 1;
            break;
        }
        if (r)
            all_zero = 0;
    }
    if (ret == 0)
        // write unsaved data (if any)
        if (eis_desc_flush(&desc2) != 0)
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

// ============================================================================

// The MVNE and MVE instructions use MOP (EIS micro operations for edit)

typedef struct {
    unsigned char eit[9];   // accessed via indices 1..8
    int n_src;
    struct {
        int es;
        int sn;
        int z;
        int bz;
    } flags;
    unsigned char *srcp;
    unsigned char src[64];
    unsigned char sign;
    unsigned char exp;
} mop_support_t;

static mop_support_t mopinfo;

// ============================================================================

static int mop_init(num_desc_t *descp, int is_decimal)
    // First stage of MVNE or MVE instruction
{
    const char *moi = "opu::MOP::init";

    mopinfo.eit[0] = 0;     // unused
    mopinfo.eit[1] = ' ';
    mopinfo.eit[2] = '*';
    mopinfo.eit[3] = '+';
    mopinfo.eit[4] = '-';
    mopinfo.eit[5] = '*';
    mopinfo.eit[6] = '\'';
    mopinfo.eit[7] = '.';
    mopinfo.eit[8] = '0';

    mopinfo.flags.es = 0;
    mopinfo.flags.sn = 0;   // set on if src is a signed decimal and it is negative
    mopinfo.flags.z = 1;
    mopinfo.flags.bz = 0;
    mopinfo.sign = 0;
    mopinfo.exp = 0;

    mopinfo.n_src = 0;
    mopinfo.srcp = mopinfo.src;
    int first = 1;
    while (descp->n() != 0) {
        uint src;
        if (descp->get(&src) != 0)
            return 1;
        if (is_decimal) {
            if (first) {
                first = 0;
                src &= 017;     // we only want four bits except for the exponent (if any)
                if (descp->s == 0 || descp->s == 1) {
                    // decimal with leading sign
                    if (src < 012) {    //  note that src > 017 is illegal but also impossible
                        fault_gen(illproc_fault);
                        return 1;
                    }
                    mopinfo.sign = src;
                    mopinfo.flags.sn = mopinfo.sign == 015;     // on if negative
                    log_msg(INFO_MSG, moi, "Got leading sign %02o (%d)\n", mopinfo.sign, mopinfo.flags.sn);
                    continue;
                }
            } else if (descp->n() == 1 && descp->width() == 4 && descp->s == 0  ) {
                // these second-to-last 4 bits are 1/2 of an 8 bit exponent
                mopinfo.exp = src << 4;
                log_msg(INFO_MSG, moi, "Got exp hi four bits %02o\n", src);
                continue;
            } else if (descp->n() == 0) {
                // last nibble
                if (descp->s == 0) {
                    // decimal with exponent
                    if (descp->width() == 4) {
                        mopinfo.exp |= src; // these last 4 bits are the 2nd 1/2 of an 8 bit exponent
                        log_msg(INFO_MSG, moi, "Got exp low four bits %02o\n", src);
                    } else {
                        mopinfo.exp = src & 0377;   // 8 bit exponent; AL-39 doesn't mention validation
                        log_msg(INFO_MSG, moi, "Got exp %02o -> %02o\n", src, src & 0377);
                    }
                    continue;
                }
                src &= 017;     // we only want four bits except for the exponent (if any)
                if (descp->s == 2) {
                    // decimal with trailing sign
                    if (src < 012) {    //  note that src > 017 is illegal but also impossible
                        fault_gen(illproc_fault);
                        return 1;
                    }
                    mopinfo.sign = src;
                    mopinfo.flags.sn = mopinfo.sign == 015;     // on if negative
                    log_msg(INFO_MSG, moi, "Got trailing sign %02o (%d)\n", mopinfo.sign, mopinfo.flags.sn);
                    continue;
                }
            }
            // log_msg(NOTIFY_MSG, moi, "Got source quad 0x%x -> %02o\n", src, src & 017);
            mopinfo.src[mopinfo.n_src++] = src & 017;
        }
    }

    log_msg(INFO_MSG, moi, "N = %d, flags.sn = %d, sign=%#o, exp=%#o(%d)\n", mopinfo.n_src, mopinfo.flags.sn, mopinfo.sign, mopinfo.exp, mopinfo.exp);
    {
    char msg[4*64+1];
    for (int i = 0; i < mopinfo.n_src; ++i) 
        sprintf(msg + i * 4, " %02o,", mopinfo.src[i] & 0xf);
    log_msg(DEBUG_MSG, moi, "SRC = {%s }\n", msg);
    }

    return 0;
}

// ============================================================================

static int mop_put(eis_desc_t *dest_descp, int is_decimal, unsigned byte)
{
    const char *moi = "opu::MOP::put";

    if (is_decimal) {
        if (mopinfo.flags.bz && mopinfo.flags.z) {
            if (byte == 0)
                byte = mopinfo.eit[1];
            else {
                // Given byte must not be from the source string...
                log_msg(WARN_MSG, moi, "Blank-when-Zero set while writing non-zero %02o\n", byte);
                cancel_run(STOP_IBKPT);
            }
        }
        if (dest_descp->nbits == 4)
            byte &= 0xf;
        else if (dest_descp->nbits == 6)
            byte &= 0x3f;
        else if (dest_descp->nbits == 9) {
            byte &= 0xf;
            byte |= mopinfo.eit[8] & 0x1f0;
        }

        if (040 <= byte && byte <= 0176)
            log_msg(DEBUG_MSG, moi, "Writing %03o '%c'\n", byte, byte);
        else
            log_msg(DEBUG_MSG, moi, "Writing %03o\n", byte);
        if (eis_desc_put(dest_descp, byte) != 0)
            return 1;
        
    } else {
        log_msg(ERR_MSG, moi, "Alphanumeric unimplemented.\n");
        cancel_run(STOP_BUG);
        if (eis_desc_put(dest_descp, byte) != 0)    // wrong
            return 1;
        log_msg(ERR_MSG, moi, "Alphanumeric unimplemented.\n");
    }
    return 0;
}

// ============================================================================

static int mop_exec_single(eis_desc_t *mop_descp, eis_desc_t *dest_descp, int is_decimal)
{
    const char *moi = "opu::MOP::exec";

    uint mop_byte;
    if (eis_desc_get(mop_descp, &mop_byte) != 0) {
        fault_gen(illproc_fault);
        return 1;
    }
    uint mop = (mop_byte >> 4) & 0177;
    uint mop_if = mop_byte & 017;

    log_msg(INFO_MSG, moi, "Got MOP {%03o,%02o} %#o\n", mop, mop_if, mop_byte);
    switch(mop) {
        case 002:   // enf -- End floating suppression
            log_msg(INFO_MSG, moi, "ENF\n");
            if ((mop_if & 010) == 0) {          // bit zero
                if (!mopinfo.flags.es) {
                    unsigned sign = mopinfo.eit[(mopinfo.flags.sn) ? 4 : 3];
                    if (mop_put(dest_descp, is_decimal, sign) != 0)
                        return 1;
                    mopinfo.flags.es = 1;
                } else {
                    // no action
                }
            } else {
                if (!mopinfo.flags.es) {
                    unsigned sign = mopinfo.eit[5];     // normally a "*"
                    if (mop_put(dest_descp, is_decimal, sign) != 0)
                        return 1;
                    mopinfo.flags.es = 1;
                } else {
                    // no action
                }
            }
            if ((mop_if & 004) == 0)    // bit one
                mopinfo.flags.bz = 1;       // Latest version of AL-39 incorrectly drops the subscript from IF
            break;
        case 001:   // insm -- insert table entry one multiple
            if (mop_if == 0)
                mop_if = 16;
            log_msg(INFO_MSG, moi, "INSM %#o(%dd) for EIT[1]==%#o\n", mop_if, mop_if, mopinfo.eit[1]);
            for (uint i = 0; i < mop_if; ++i) {
                if (mop_put(dest_descp, is_decimal, mopinfo.eit[1]) != 0)
                    return 1;
                if (dest_descp->n == 0)
                    return 0;   // exhaustion of destination is the only normal termination
            }
            break;
        case 020: { // lte -- load table entry
            if (mop_if == 0 || mop_if > 8) {
                log_msg(INFO_MSG, moi, "LTE with bad IF %d\n", mop_if);
                fault_gen(illproc_fault);
                return 1;
            }
            unsigned int m;
            if (eis_desc_get(mop_descp, &m) != 0) {
                fault_gen(illproc_fault);
                return 1;
            }
            mopinfo.eit[mop_if] = m;
            log_msg(INFO_MSG, moi, "LTE: Setting EIT[%d] to %02o\n", mop_if, m);
            break;
        }
        case 006:   // mfls -- move with floating sign insertion
            if (mop_if == 0)
                mop_if = 16;
            log_msg(INFO_MSG, moi, "MFLS %#o(%dd)\n", mop_if, mop_if);
            for (uint i = 0; i < mop_if; ++i) {
                if (mopinfo.n_src == 0) {
                    log_msg(ERR_MSG, moi, "Source exhausted\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                unsigned src = *mopinfo.srcp++;
                --mopinfo.n_src;
                if (src != 0)
                    mopinfo.flags.z = 0;
                log_msg(DEBUG_MSG, moi, "Fetched src %#o\n", src);
                if (! mopinfo.flags.es) {
                    if (src == 0)
                        src = mopinfo.eit[1];   // normally " "
                    else {
                        unsigned sign = mopinfo.eit[(mopinfo.flags.sn) ? 4 : 3];
                        if (mop_put(dest_descp, is_decimal, sign) != 0)
                            return 1;
                        // TODO: should we test for destination exhaustion here?
                        mopinfo.flags.es = 1;
                    }
                }
                if (mop_put(dest_descp, is_decimal, src) != 0)
                    return 1;
                if (dest_descp->n == 0)
                    return 0;   // exhaustion of destination is the only normal termination
            }
            break;
        case 015:   // mvc -- move source chars
            if (mop_if == 0)
                mop_if = 16;
            log_msg(INFO_MSG, moi, "MVC %#o(%dd)\n", mop_if, mop_if);
            for (uint i = 0; i < mop_if; ++i) {
                if (mopinfo.n_src == 0) {
                    log_msg(ERR_MSG, moi, "Source exhausted\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                unsigned src = *mopinfo.srcp++;
                --mopinfo.n_src;
                if (src != 0)
                    mopinfo.flags.z = 0;
                if (mop_put(dest_descp, is_decimal, src) != 0)
                    return 1;
                if (dest_descp->n == 0)
                    return 0;   // exhaustion of destination is the only normal termination
            }
            break;
        case 003:   // ses -- Set End Suppression
            mopinfo.flags.es =  (mop_if & 010) != 0;            // bit zero
            if ((mop_if & 004) != 0)        // bit one
                mopinfo.flags.bz = 1;
            break;
        case 010:   // insb -- Insert Blank On Suppression
            if (mop_if > 8) {
                log_msg(NOTIFY_MSG, moi, "INSB with bad IF %d\n", mop_if);
                fault_gen(illproc_fault);
                return 1;
            }
            log_msg(INFO_MSG, moi, "INSB: ES==%d, IF==%dd\n", mopinfo.flags.es, mop_if);
            unsigned byte;
            if (! mopinfo.flags.es) {
                byte = mopinfo.eit[1];
                if (mop_if == 0) {
                    log_msg(INFO_MSG, moi, "INSB: write EIT[one]==%#o and skip next MOP.\n", mopinfo.eit[1]);
                    unsigned ignored;
                    if (eis_desc_get(mop_descp, &ignored) != 0) {
                        fault_gen(illproc_fault);
                        return 1;
                    }
                } else
                    log_msg(INFO_MSG, moi, "INSB: write EIT[one]==%#o.\n", mopinfo.eit[1]);
            } else {
                if (mop_if != 0) {
                    byte = mopinfo.eit[mop_if];
                    log_msg(INFO_MSG, moi, "INSB: write EIT[IF==%d] which is %#o\n", mop_if, byte);
                } else {
                    if (eis_desc_get(mop_descp, &byte) != 0) {
                        fault_gen(illproc_fault);
                        return 1;
                    }
                    log_msg(INFO_MSG, moi, "INSB: read next mop (%#o) and write it.\n", byte);
                }
            }
            if (mop_put(dest_descp, is_decimal, byte) != 0)
                return 1;
            break;
        default:
            cancel_run(STOP_BUG);
            log_msg(ERR_MSG, moi, "Unimplemented MOP code %#o\n", mop);
            return 1;
    }

    return 0;
}


// ============================================================================

/*
 *
 * Main work of MVNE and MVE instructions -- execute a list of MOPs
 *
 */

static int mop_execute(eis_desc_t *mop_descp, eis_desc_t *dest_descp, int is_decimal)
{
    const char *moi = "opu::MOP::execute";

    int ret = 0;

    while (mop_descp->n > 0 && dest_descp->n > 0) {
        if (mop_exec_single(mop_descp, dest_descp, is_decimal) != 0) {
            ret = 1;
            // break;       // BUG
        }
        if (ret) log_msg(ERR_MSG, moi, "MOP list %d remaining, dest %d remaining.\n", mop_descp->n, dest_descp->n);
    }

    // write unsaved data (if any)
    // if (ret == 0)    // BUG
        if (eis_desc_flush(dest_descp) != 0)
            return 1;

    return ret;
}

// ============================================================================

#if 0
static int _op_mvne(const instr_t* ip);
int op_mvne(const instr_t* ip)
{
    extern DEVICE cpu_dev;
    int saved_debug = opt_debug;
    int saved_dctrl = cpu_dev.dctrl;
    ++ opt_debug; ++ cpu_dev.dctrl;
    int ret = _op_mvne(ip);
    opt_debug = saved_debug;
    cpu_dev.dctrl = saved_dctrl;
    return ret;
}
#endif

int op_mvne(const instr_t* ip)
{
    const char* moi = "OPU::mvne";

    uint mf2bits = ip->addr & MASKBITS(7);
    uint mf3bits = (ip->addr >> 9) & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));
    eis_mf_t mf3;
    (void) parse_mf(mf3bits, &mf3);
    log_msg(DEBUG_MSG, moi, "mf3 = %s\n", mf2text(&mf3));

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, &mf3, &word3) != 0)
        return 1;

    cpu.irodd_invalid = 1;

    num_desc_t desc1(ip->mods.mf1, word1, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    eis_desc_t desc2;
    if (decode_eis_alphanum_desc(&desc2, &mf2, word2, 1, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 2.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_mod64(&desc2);
    eis_desc_t desc3;
    if (decode_eis_alphanum_desc(&desc3, &mf3, word3, 0, 1) != 0) {
        log_msg(ERR_MSG, moi, "failed to decode descriptor 3.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    eis_desc_mod64(&desc3);
    log_msg(INFO_MSG, moi, "desc2: %s\n", eis_desc_to_text(&desc2));
    log_msg(INFO_MSG, moi, "desc3: %s\n", eis_desc_to_text(&desc3));

    int ret = 0;
    // Initialize the EIT and load the source bytes
    if (mop_init(&desc1, 1) != 0)
        return 1;

    if (mop_execute(&desc2, &desc3, 1) != 0)
        // return 1;
        ret = 1;        // BUG: return...

    //log_msg(WARN_MSG, moi, "Need to verify; auto breakpoint\n");
    //cancel_run(STOP_IBKPT);
    PPR.IC += 4;        // BUG: when should we bump IC?  probably not for faults, but probably yes for conditions
    return ret;
}

// ============================================================================
