/*
    eis_opu.cpp -- implements EIS multi-word instructions.

    Called by routines in opu.c

    Makes heavy use of functions in eis_desc.cpp for descriptor addressing
    and fetches/puts.

*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

using namespace std;
#include <iostream>

#include <ctype.h>  // for isprint
#include "hw6180.h"
#include "eis.hpp"

static int mop_copy(alpha_desc_t *dest_descp, int is_decimal, uint mop_byte);

// ============================================================================

int op_move_alphanum(const instr_t* ip, int fwd)
{
    const char* moi = (fwd) ? "OPU::mlr" : "OPU::mrl";

    // BUG: Detection of GBCD overpunch is not done

    cpu.irodd_invalid = 1;

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

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

    cpu.irodd_invalid = 1;

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, NULL, &word2, NULL, &word3) != 0)
        return 1;

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

    cpu.irodd_invalid = 1;

    uint fill = ip->addr >> 9;
    uint t = (ip->addr >> 8) & 1;
    uint mf2bits = ip->addr & MASKBITS(7);

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, &word3) != 0)
        return 1;

    alpha_desc_t desc1(ip->mods.mf1, word1, 1);
    alpha_desc_t desc2(mf2, word2, 1);
    uint addr3;
    if (get_eis_indir_addr(word3, &addr3) != 0)
        return 1;

    char buf[100];
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(buf));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", desc2.to_text(buf));
    log_msg(DEBUG_MSG, moi, "translation table at %#llo => %#o\n", word3, addr3);

    int ret = 0;

    // uint n = desc1.n;
    while (desc2.n() > 0) {
        uint m;
        if (desc1.n() == 0)
            m = fill & MASKBITS(desc2.width());
        else
            if (desc1.get(&m) != 0) {
                ret = 1;
                break;
            }
        uint t;
        if (get_table_char(addr3, m, &t) != 0) {
            log_msg(WARN_MSG, moi, "Unable to read table\n");
            ret = 1;
            break;
        }
        if (desc2.put(t) != 0) {
            ret = 1;
            break;
        }
    }

    if (ret == 0) {
        IR.truncation = desc1.n() != 0;
        if (IR.truncation) {
            // Harry & Charles report ascii.no_trim expects TRO instead of truncation
            log_msg(DEBUG_MSG, moi, "Setting truncation flag, but may need to also set tally-run-out.\n");
            //IR.tally_runout = 1;
        }
        if (IR.truncation && t) {
            fault_gen(overflow_fault);  // truncation
            ret = 1;
        }
    }
    // write unsaved data (if any)
    if (ret == 0)
        if (desc2.flush() != 0)
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

    cpu.irodd_invalid = 1;

    uint fill = ip->addr >> 9;
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    alpha_desc_t desc1(ip->mods.mf1, word1, 1);
    // force (ignored) type of descriptor 2 to match descriptor 1
    unsigned ta = getbits36(word1, 21, 2);
    word2 = setbits36(word2, 21, 2, ta);
    alpha_desc_t desc2(mf2, word2, 1);

    char buf[100];
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(buf));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", desc2.to_text(buf));

    int ret = 0;

    uint nib1, nib2;
    while (desc1.n() > 0 || desc2.n() > 0) {
        if (desc1.n() == 0)
            nib1 = fill & MASKBITS(desc1.width());
        else
            if (desc1.get(&nib1) != 0) {
                ret = 1;
                break;
            }
        if (desc2.n() == 0)
            nib2 = fill & MASKBITS(desc2.width());
        else
            if (desc2.get(&nib2) != 0) {
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

    cpu.irodd_invalid = 1;

    uint sign_ctl = ip->addr >> 17;

    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;
    log_msg(INFO_MSG, moi, "sign-ctl=%d\n", sign_ctl);

    alpha_desc_t desc1(ip->mods.mf1, word1, 1); // will be 9bit
    num_desc_t desc2(mf2, word2, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));

    if (desc2.sf() != 0) {
        log_msg(ERR_MSG, moi, "Descriptor 2 scaling factor must be zero\n");
        fault_gen(illproc_fault);
        return 1;
    }
    if (desc2.s() == 0) {
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
    //int w = desc1.width();
    //if (w != 9) log_msg(INFO_MSG, moi, "source is %d-bit.\n", w);
    const int w = 9;
    for (int nbits = 0; desc1.n() > 0; nbits += w) {
        uint nib;
        if (desc1.get(&nib) != 0) { // must fetch when needed
            return 1;
            break;
        }
        if (nbits >= 36) {
            src_hi <<= w;
            src_hi |= getbits36(src_lo, 0, w);
        }
        src_lo <<= w;
        src_lo |= nib;
    }
    src_lo &= MASK36;
    log_msg(INFO_MSG, moi, "source binary is { %#llo, %012llo}\n", src_hi, src_lo);

    int negate = 0;
    if (desc2.s() != 03)
        if (bit36_is_neg(src_hi)) {
            negate72(&src_hi, &src_lo);
            negate = 1;
            log_msg(INFO_MSG, moi, "source binary is negative, negating yields { %#llo, %012llo}\n", src_hi, src_lo);
        }
        
    // We'll initially compute the results into rhi and rlo via 4bit nibbles.
    // We only need just over 72 bits; note that we'll use all 64 bits of rlo,
    // not just 36 of them.
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

    // int needed = ndigits + (desc2.num.s() != 3);
    // if (needed > desc2.n) ... overflow
    
    int ret = 0;

    // put_eis_an_rev doesn't exist...
    uint results[64];
    int n = 0;

    if (desc2.s() == 2)
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
            results[n] |= 060;      // 9-bit unpacked BCD is ASCII
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
    if (desc2.s() == 1) {
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
            // log_msg(DEBUG_MSG, moi, "Storing byte %d from results[%d]==%#o\n", desc2.n(), desc2.n()-1, results[desc2.n()-1]);
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


#if 1
static int _op_dtb(const instr_t* ip);
int op_dtb(const instr_t* ip)
{
    extern DEVICE cpu_dev;
    int saved_debug = opt_debug;
    int saved_dctrl = cpu_dev.dctrl;
    ++ opt_debug; ++ cpu_dev.dctrl;
    int ret = _op_dtb(ip);
    opt_debug = saved_debug;
    cpu_dev.dctrl = saved_dctrl;
    return ret;
}
#endif
int _op_dtb(const instr_t* ip)
{
    const char* moi = "OPU::dtb";

    cpu.irodd_invalid = 1;

    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    num_desc_t desc1(ip->mods.mf1, word1, 1);
    num_desc_t desc2(mf2, word2, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf)); // 4 or 9-bit
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));  // 9 bit

    if (desc1.sf() != 0) {
        log_msg(ERR_MSG, moi, "Descriptor 1 scaling factor must be zero\n");
        fault_gen(illproc_fault);
        return 1;
    }
    if (desc1.s() == 0) {
        log_msg(ERR_MSG, moi, "Descriptor 1 sign/decimal-type type must not be zero\n");
        fault_gen(illproc_fault);
        return 1;
    }
    if (desc2.n() == 0 || desc2.n() > 8) {
        log_msg(ERR_MSG, moi, "Descriptor 2 length must in range 1..8\n");
        fault_gen(illproc_fault);
        return 1;
    }

    // First op is bcd with either 4bit or 9bit nibbles.
    // Second op is a binary number between 9bits and 72bits.
    // The maximum value is a nine digit string consisting of all nines.  The
    // corresponding unsigned binary value only requires 32 bits.
    
    // Calculate binary value
    unsigned val = 0;
    int negate = 0;
    int first = 1;
    int is_zero = 1;
    while (desc1.n() > 0) {
        uint nib;
        if (desc1.get(&nib) != 0) { // must fetch when needed
            return 1;
            break;
        }
        if ((first && desc1.s() == 1) || (desc1.n() == 0 && desc1.s() == 2)) {
            // leading sign or trailing sign
            if ((nib & 0xf) < 012) {
                log_msg(ERR_MSG, moi, "Bad sign byte, %#0.\n", nib);
                fault_gen(illproc_fault);
                return 1;
            }
            if ((nib & 0xf) == 015)
                negate = 1;
            log_msg(INFO_MSG, moi, "BCD sign byte is %#0 -- %s\n",
                nib, (negate) ? "negative" : "positive");
            first = 0;
            continue;
        }
        log_msg(INFO_MSG, moi, "source BCD nibble is %03o -> %d aka %d\n", nib, nib, nib & 0xf);
        if (desc1.width() == 9) {
            // convert from unpacked ASCII BCD to binary
            // nib -= 060;
            nib &= 0xf;
        }
        val = val * 10 + nib;
        if (nib != 0)
            is_zero = 0;
        first = 0;
    }
    if (negate)
        val = (~ val) + 1;  // val will not be max positive or max neg
    log_msg(INFO_MSG, moi, "source BCD value is %d -> %#o.\n", val, val);

    // We have a source value of at most 32 bits.  Dest is 9 to 72 bits;
    // see if it fits.

    int is_overflow;
    if (negate) {
        // All upper bits of val should be on
        unsigned mask = MASKBITS(desc2.n() * 9);
        is_overflow = (mask | val) != MASKBITS(sizeof(val) * 8);
    } else {
        // None of the upper bits of val should be on
        is_overflow = (val >> (desc2.n() * 9)) != 0;
    }

    int ret = 0;
    if (is_overflow) {
        log_msg(INFO_MSG, moi, "Overflow\n");
        IR.overflow = 1;
        fault_gen(illproc_fault);
    } else {
        IR.zero = is_zero;

        log_msg(DEBUG_MSG, moi, "Storing %d bytes\n", desc2.n());
        // put_eis_an_rev() doesn't exist, so write fwd, but access
        // appropriate bits of val
        for (int n = desc2.n(); n != 0; --n) {
            uint nib;
            if (n > 4)
                nib = 0;
            else 
                nib = val >> ((n - 1) * 9) & 0x1ff;
            if (desc2.put(nib) != 0) {
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

    cpu.irodd_invalid = 1;

    uint fill = ip->addr >> 17;
    // uint trunc_enable = (ip->addr >> 8) & 1; // Ignored for cmpb
    uint mf2bits = ip->addr & MASKBITS(7);
    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    bit_desc_t desc1(ip->mods.mf1, word1, 1);
    bit_desc_t desc2(mf2, word2, 1);
    char buf[100];
    log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(buf));
    log_msg(DEBUG_MSG, moi, "desc2: %s\n", desc2.to_text(buf));
    log_msg(DEBUG_MSG, moi, "fill bit %d\n", fill);

    int ret = 0;

    flag_t nib1, nib2;  // for this instruction, these are bit valued
    int zero = 1;
    int carry = 1;
    while (desc1.n() > 0 || desc2.n() > 0) {
        if (desc1.n() == 0)
            nib1 = fill & MASKBITS(desc1.width());
        else
            if (desc1.get(&nib1) != 0) {    // must fetch when needed
                ret = 1;
                break;
        }
        if (desc2.n() == 0)
            nib2 = fill & MASKBITS(desc2.width());
        else
            if (desc2.get(&nib2) != 0) {    // must fetch when needed
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

    alpha_desc_t desc1(ip->mods.mf1, word1, fwd);

    // force (ignored) type of desc2 to match desc1
    unsigned ta = getbits36(word1, 21, 2);
    word2 = setbits36(word2, 21, 2, ta);
    // force (ignored) count of desc2 to one
    word2 = setbits36(word2, 24, 12, 1);
    alpha_desc_t desc2(mf2, word2, 1);

    uint y3;
    if (get_eis_indir_addr(word3, &y3) != 0)
            return 1;
    
    if (opt_debug) {
        char buf[100];
        log_msg(DEBUG_MSG, moi, "mask = %03o\n", mask);
        log_msg(DEBUG_MSG, moi, "desc1: %s\n", desc1.to_text(buf));
        log_msg(DEBUG_MSG, moi, "desc2: %s\n", desc2.to_text(buf));
        log_msg(DEBUG_MSG, moi, "y3: %06o\n", y3);
    }

    uint test_nib;
    if (mf2.id == 0 && mf2.reg == 3) {  // 3 is ",du"
        test_nib = getbits36(word2, 0, 9);
        log_msg(DEBUG_MSG, moi, "test char specified via special-case ',du'\n");
    } else {
        log_msg(DEBUG_MSG, moi, "Getting test char\n");
        if (desc2.get(&test_nib) != 0)
            return 1;
    }
    if (opt_debug) {
        if (isprint(test_nib))
            log_msg(DEBUG_MSG, moi, "test char: %03o '%c'\n", test_nib, test_nib);
        else
            log_msg(DEBUG_MSG, moi, "test char: %03o\n", test_nib);
    }

    int ret = 0;
    uint n = desc1.n();
    uint i;
    for (i = 0; i < n; ++i) {
        uint nib;
        ret = desc1.get(&nib);
        if (ret != 0)
            break;
        uint z = ~mask & (nib ^ test_nib);
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "compare nibble %#o(%+d): value %03o yields %#o\n", i, i, nib, z);
        if (z == 0)
            break;
    }
    if (ret == 0) {
        IR.tally_runout = i == n;
        // t_uint64 word = setbits36(0, 12, 24, i);
        t_uint64 word = i;
        if (opt_debug)
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

    cpu.irodd_invalid = 1;

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

    char buf[100];
    bit_desc_t desc1(ip->mods.mf1, word1, 1);
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(buf));
    bit_desc_t desc2(mf2, word2, 1);
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(buf));

    int ret = 0;

    flag_t all_zero = 1;
    while (desc2.n() > 0) {
        flag_t bit1, bit2;
        if (desc1.n() == 0)
            bit1 = fill;
        else
            if (desc1.get(&bit1) != 0) {
                ret = 1;
                break;
            }
        // val() won't ever advance the pointer
        if (desc2.val(&bit2) != 0) {
            ret = 1;
            break;
        }
        flag_t r = (bolr >> (3 - ((bit1 << 1) | bit2))) & 1;    // like indexing into a truth table
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "nbits1=%d, nbits2=%d; %d op(%#o) %d => %d\n", desc1.n(), desc2.n(), bit1, bolr, bit2, r);
        if (desc2.put(r) != 0) {
            ret = 1;
            break;
        }
        if (r)
            all_zero = 0;
    }
    if (ret == 0)
        // write unsaved data (if any)
        if (desc2.flush() != 0)
            ret = 1;
    if (ret == 0) {
        IR.zero = all_zero;
        IR.truncation = desc1.n() > 0;
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
                if (descp->s() == 0 || descp->s() == 1) {
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
            } else if (descp->n() == 1 && descp->width() == 4 && descp->s() == 0  ) {
                // these second-to-last 4 bits are 1/2 of an 8 bit exponent
                mopinfo.exp = src << 4;
                log_msg(INFO_MSG, moi, "Got exp hi four bits %02o\n", src);
                continue;
            } else if (descp->n() == 0) {
                // last nibble
                if (descp->s() == 0) {
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
                if (descp->s() == 2) {
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

    log_msg(INFO_MSG, moi, "N = %d, flags: { es = %d, sn = %d, z = %d, bz = %d }, sign=%#o, exp=%#o(%d)\n",
            mopinfo.n_src,
            mopinfo.flags.es, mopinfo.flags.sn, mopinfo.flags.z, mopinfo.flags.bz,
            mopinfo.sign, mopinfo.exp, mopinfo.exp);
    {
    char msg[4*64+1];
    for (int i = 0; i < mopinfo.n_src; ++i) 
        sprintf(msg + i * 4, " %02o,", mopinfo.src[i] & 0xf);
    log_msg(DEBUG_MSG, moi, "SRC = {%s }\n", msg);
    }

    return 0;
}

// ============================================================================

static int mop_put(alpha_desc_t *dest_descp, int is_decimal, unsigned byte)
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
        if (dest_descp->width() == 4)
            byte &= 0xf;
        else if (dest_descp->width() == 6)
            byte &= 0x3f;
        else if (dest_descp->width() == 9) {
            // Caller should fill digits or send specials such as blanks
        }

        if (040 <= byte && byte <= 0176)
            log_msg(DEBUG_MSG, moi, "Writing %03o '%c'\n", byte, byte);
        else
            log_msg(DEBUG_MSG, moi, "Writing %03o\n", byte);
        if (dest_descp->put(byte) != 0)
            return 1;
        
    } else {
        log_msg(ERR_MSG, moi, "Alphanumeric unimplemented.\n");
        cancel_run(STOP_BUG);
        if (dest_descp->put(byte) != 0) // wrong
            return 1;
        log_msg(ERR_MSG, moi, "Alphanumeric unimplemented.\n");
    }
    return 0;
}

// ============================================================================

static int mop_exec_single(alpha_desc_t *mop_descp, alpha_desc_t *dest_descp, int is_decimal)
{
    const char *moi = "opu::MOP::exec";

    uint mop_byte;
    if (mop_descp->get(&mop_byte) != 0) {
        fault_gen(illproc_fault);
        return 1;
    }
    uint mop = (mop_byte >> 4) & 037;
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
                    log_msg(INFO_MSG, moi, "flags.es set on.\n");
                } else {
                    // no action
                }
            } else {
                if (!mopinfo.flags.es) {
                    unsigned sign = mopinfo.eit[5];     // normally a "*"
                    if (mop_put(dest_descp, is_decimal, sign) != 0)
                        return 1;
                    mopinfo.flags.es = 1;
                    log_msg(INFO_MSG, moi, "flags.es set on.\n");
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
                if (dest_descp->n() == 0)
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
            if (mop_descp->get(&m) != 0) {
                fault_gen(illproc_fault);
                return 1;
            }
            mopinfo.eit[mop_if] = m;
            log_msg(INFO_MSG, moi, "LTE: Setting EIT[%d] to %02o\n", mop_if, m);
            break;
        }
        case 006:   // mfls -- move with floating sign insertion
            return mop_copy(dest_descp, is_decimal, mop_byte);
        case 015:   // mvc -- move source chars
            return mop_copy(dest_descp, is_decimal, mop_byte);
        case 004:   // mvzb -- move with zero suppression and blank replacment
            {
            extern DEVICE cpu_dev; ++ opt_debug; ++ cpu_dev.dctrl;
            int ret = mop_copy(dest_descp, is_decimal, mop_byte);
            -- opt_debug; -- cpu_dev.dctrl;
            return ret;
            }
        case 003:   // ses -- Set End Suppression
            {
            int old_es = mopinfo.flags.es;
            mopinfo.flags.es =  (mop_if & 010) != 0;            // bit zero
            if (old_es != mopinfo.flags.es)
                log_msg(INFO_MSG, moi, "flags.es is now %s.\n", (mopinfo.flags.es) ? "on" : "off");
            if ((mop_if & 004) != 0)        // bit one
                mopinfo.flags.bz = 1;
            break;
            }
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
                    if (mop_descp->get(&ignored) != 0) {
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
                    if (mop_descp->get(&byte) != 0) {
                        fault_gen(illproc_fault);
                        return 1;
                    }
                    //if (is_decimal && dest_descp->nbits == 9) {
                    //  -- unclear if we should high order fill when pulling from mop string...
                    //  byte &= 0xf;
                    //  byte |= mopinfo.eit[8] & 0x1f0;
                    //}
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
 * Generic copy used by MOPS MVC, MBZA, MVZB, etc
 *
 */

static int mop_copy(alpha_desc_t *dest_descp, int is_decimal, uint mop_byte)
{
    const char *moi = "opu::MOP::exec";
    uint mop = (mop_byte >> 4) & 037;
    uint mop_if = mop_byte & 017;

    const char *mop_name; 
    switch(mop) {
        case 004: mop_name = "mvzb"; break;
        case 006: mop_name = "mfls"; break;
        case 015: mop_name = "mvc"; break;
        default: mop_name = ""; break;
    }

    if (mop_if == 0)
        mop_if = 16;
    log_msg(INFO_MSG, moi, "%s %#o(%dd)\n", mop_name, mop_if, mop_if);

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
        if (mop == 015) {
            // mvc - move source chars
            if (is_decimal && dest_descp->width() == 9) {
                src &= 0xf;
                src |= mopinfo.eit[8] & 0x1f0;
            }
        } else if (mop == 006) {
            // mfls -- move with floating sign insertion
            log_msg(DEBUG_MSG, moi, "Fetched src %#o\n", src);
            if (mopinfo.flags.es) {
                if (is_decimal && dest_descp->width() == 9) {
                    src &= 0xf;
                    src |= mopinfo.eit[8] & 0x1f0;
                }
            } else {
                if (src == 0) {
                    src = mopinfo.eit[1];   // normally " "
                    log_msg(DEBUG_MSG, moi, "Using EIT[1] == %#o\n", src);
                } else {
                    int which = (mopinfo.flags.sn) ? 4 : 3;
                    unsigned sign = mopinfo.eit[which];
                    log_msg(DEBUG_MSG, moi, "Using sign at EIT[%d] == %#o\n", which, sign);
                    if (mop_put(dest_descp, is_decimal, sign) != 0)
                        return 1;
                    // TODO: should we test for destination exhaustion here?
                    mopinfo.flags.es = 1;
                    log_msg(INFO_MSG, moi, "flags.es set on.\n");
                    src &= 0xf;
                    src |= mopinfo.eit[8] & 0x1f0;
                }
            }
        } else if (mop == 004) {
            // mvzb -- move with zero suppression and blank replacment
            log_msg(DEBUG_MSG, moi, "Fetched src %#o\n", src);
            if (mopinfo.flags.es) {
                // src unchanged
                if (is_decimal && dest_descp->width() == 9) {
                    src &= 0xf;
                    src |= mopinfo.eit[8] & 0x1f0;
                }
            } else {
                if (src == 0) {
                    src = mopinfo.eit[1];   // normally " "
                    log_msg(DEBUG_MSG, moi, "Using EIT[1] == %#o\n", src);
                } else {
                    mopinfo.flags.es = 1;
                    if (is_decimal && dest_descp->width() == 9) {
                        src &= 0xf;
                        src |= mopinfo.eit[8] & 0x1f0;
                    }
                }
            }
        } else {
            log_msg(ERR_MSG, moi, "Unimplemented MOP copy code %#o\n", mop);
        }
        if (mop_put(dest_descp, is_decimal, src) != 0)
            return 1;
        if (dest_descp->n() == 0)
            return 0;   // exhaustion of destination is the only normal termination
    }
    return 0;
}

// ============================================================================

/*
 *
 * Main work of MVNE and MVE instructions -- execute a list of MOPs
 *
 */

static int mop_execute(alpha_desc_t *mop_descp, alpha_desc_t *dest_descp, int is_decimal)
{
    const char *moi = "opu::MOP::execute";

    int ret = 0;

    while (mop_descp->n() > 0 && dest_descp->n() > 0) {
        if (mop_exec_single(mop_descp, dest_descp, is_decimal) != 0) {
            ret = 1;
            // break;       // BUG
        }
        if (ret) log_msg(ERR_MSG, moi, "MOP list %d remaining, dest %d remaining.\n", mop_descp->n(), dest_descp->n());
    }

    // write unsaved data (if any)
    // if (ret == 0)    // BUG
        if (dest_descp->flush() != 0)
            return 1;

    return ret;
}

// ============================================================================

#if 1
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

int _op_mvne(const instr_t* ip)
{
    const char* moi = "OPU::mvne";

    cpu.irodd_invalid = 1;

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

    num_desc_t desc1(ip->mods.mf1, word1, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    alpha_desc_t desc2(mf2, word2, 1);
    // NOTE: AL-39 says that the only the lower 6 bits of the lengths
    // may be non-zero, but also says that the lengths are treated
    // as modulo 64.  The latter probably just means that the higher order
    // bits are ignored and no fault is generated if they are set.
    desc2.mod64();
    alpha_desc_t desc3(mf3, word3, 1);
    desc3.mod64();
    char buf[100];
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(buf));
    log_msg(INFO_MSG, moi, "desc3: %s\n", desc3.to_text(buf));

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

extern "C" {
#include <decNumber.h>
}

static decNumber* new_decNumber(void)
{
    int ndigits = 64;
    int extra_digits = 64 - DECNUMUNITS * DECDPUN;
    if (extra_digits < 0)
        extra_digits = 0;
    int extra_space = sizeof(decNumberUnit) * (extra_digits / DECDPUN + 1);
    return (decNumber*) calloc(1, sizeof(decNumber) + extra_space);
}

static void debug_show(const decNumber& decnum, const char* msg);

// ----------------------------------------------------------------------------

static void multics_decContext(decContext& context, flag_t scaled)
{
    decContextDefault(&context, DEC_INIT_BASE);
    context.digits = 63;
    int exp_digits = scaled ? 6 : 8;
    context.emax = (2 << (exp_digits - 1)) - 1;
    context.emin = - (2 << (exp_digits -1));
    context.round = DEC_ROUND_HALF_UP;
    context.clamp = 0;
}


// ----------------------------------------------------------------------------


class dec_t {
public:
    // t_uint64 coe_val;
    unsigned coe[65];       // original raw data, 0..9 or '0' .. '9'
    // flag_t is_ascii;     // true if all the above are '0' .. '9'
    char coe_text[65];      // '0' .. '9'
    int width;              // width in bits of raw source data
    int n_lz;               // number of leading zeros
    int n_coe;
    unsigned sign_nibble;   // original raw 4-bit or 9-bit sign byte
    flag_t is_neg;          // sign byte intrepreted
    flag_t is_zero; 
    int exp;    // From floating point exponent or fixed point scale
    dec_t() {}
    dec_t(decNumber& decnum);
};

static int read_dec(num_desc_t& desc, dec_t& dec);
static int write_decimal(const dec_t& src, num_desc_t desc2, flag_t rounding, flag_t trunc_fe);

static int multics_to_dec(const dec_t& dec, decNumber& dnum);

// ============================================================================

#if 1
static int _op_mvn(const instr_t* ip);
int op_mvn(const instr_t* ip)
{
    extern DEVICE cpu_dev;
    int saved_debug = opt_debug;
    int saved_dctrl = cpu_dev.dctrl;
    ++ opt_debug; ++ cpu_dev.dctrl;
    int ret = _op_mvn(ip);
    opt_debug = saved_debug;
    cpu_dev.dctrl = saved_dctrl;
    return ret;
}
#endif

/*
    mvn

    Move decimal data.
    Note that the 4-bit encoding is essentially BCD and the 9-bit encoding
    is essentially ASCII.

    Decimal data comes in four formats:
        Unsigned scaled integer 
        Signed scaled integer (two formats -- leading or trailing sign)
        Floating point
    The scaled integers have the scale encoded in the instruction, it's
    not part of the data format.

    Decimal floating point doesn't use a fractional mantissa as per AL-39...
    So, there's no essential difference between the so-call floating
    numbers and the scaled numbers except that one has the exponent in
    the data and the other has the exponent (scale factor) as an implied
    value specified by the instruction.

    AL-39 says that the scale factor is ignored by the hardware.  That
    can't be completely true...
*/

int _op_mvn(const instr_t* ip)
{
    const char* moi = "OPU::mvn";

    cpu.irodd_invalid = 1;

    uint mf2bits = ip->addr & MASKBITS(7);
    flag_t sign_ctl = (ip->addr >> 17) & 1;
    flag_t trunc_fe = (ip->addr >> 8) & 1;
    flag_t rounding = (ip->addr >> 7) & 1;

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    t_uint64 word1, word2;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, NULL, NULL) != 0)
        return 1;

    log_msg(INFO_MSG, moi, "sign-ctl=%d, trunc-fault=%d, rounding=%d\n",
        sign_ctl, trunc_fe, rounding);
    num_desc_t desc1(ip->mods.mf1, word1, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    num_desc_t desc2(mf2, word2, 1);
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));

    // Determine how many byte positions are in the source and the
    // destination, not counting the sign bytes and exponent bytes

    int n_src = desc1.ndigits(); // digit bytes, excluding signs and exponents

    int n_dest = desc2.ndigits(); // digit bytes, excluding signs and exponents

    log_msg(DEBUG_MSG, moi, "Src is type %d with %d digits with %d total bytes.\n",
        desc1.s(), n_src, desc1.n());
    log_msg(DEBUG_MSG, moi, "Dst is type %d with %d digits with %d total bytes.\n",
        desc2.s(), n_dest, desc2.n());

    dec_t src;
    int ret = read_dec(desc1, src);
    if (ret == 0)
        ret = write_decimal(src, desc2, rounding, trunc_fe);

    log_msg(INFO_MSG, moi, "After copy finishes: desc1: %s\n", desc1.to_text(msg_buf));
    log_msg(INFO_MSG, moi, "After copy finishes: desc2: %s\n", desc2.to_text(msg_buf));

    log_msg(WARN_MSG, moi, "Untested; auto breakpoint\n");
    cancel_run(STOP_IBKPT);

    PPR.IC += 3;
    return ret;
}

// ----------------------------------------------------------------------------

/*
    write_decimal

    Originally part of mvn (move decimal data instruction), but broken out
    for use by other decimal instructions.
*/

static int write_decimal(const dec_t& src, num_desc_t desc2, flag_t rounding, flag_t trunc_fe)
{

    const char* moi = "OPU::write-decimal";

    /*
    examples

    integer -> integer
        right justify
        shift according to scale diff
            nshift = SF2 - SF1 // negative <<; positive >>
        <src scale>-><dest scale> == (scale diff) ==> example src -> result
        0->-1 ==(-1)==> 0019 -> 0190*10^-1
        0->-1 ==(-1)==> 1234 -> ????*10^-1 <overflow>
        0->+1 ==(+1)==> 0080 -> 0008*10^1
        0->+1 ==(+1)==> 0082 -> 0008*10^1 <w/o rounding, loss of digit 2>
        0->+1 ==(+1)==> 0085 -> 0009*10^1 <with rounding>
        0->+1 ==(+1)==> 9998 -> 1000*10^1 <with rounding>
        -1->0 ==(+1)==> 9310 -> 0931
        -1->0 ==(+1)==> 0045 -> 0004 <w/o rounding, loss of digit 5>
        -1->0 ==(+1)==> 0045 -> 0005 <with rounding>
        +1->0 ==(-1)==> 0201 -> 2010
        +1->0 ==(-1)==> 9000 -> <overflow>
    integer -> float
        right justify
        set exp to scale (scale is smaller range than exp, so always fits)
        maybe normalize?
    float -> integer
        right justify
        shift according to scale & exp
    */

    /*
        Determine fill types and the value of the output sign nibble
    */

    int src_sign = src.sign_nibble;
    int src_neg = src.is_neg;
    int auto_break = 0;

#if 0
    int dst_sign = (src_neg) ? 015
        : (desc2.width() == 9) ? 013
        : (src_sign) ? 013 : 014;
#else
    int dst_sign = (desc2.width() == 4)
        ? (src_neg)
            ? 015
            : ((src_sign) ? 013 : 014)
        : src_sign;
#endif

    // AL-39 says in the mvn description that high-order bits are only
    // filled when writing a 9-bit destination if the source was 4-bit.
    // That doesn't match what is said in Section 2, "Data Representation",
    // but is more specifc...
    int fill, sign_fill;
    if (src.width >= desc2.width()) {
        fill = sign_fill = 0;
    }
    else {
        // Moving 4-bit data to 9-bit destination
        fill = 0x30;    // ascii zero
        sign_fill = 0x20;   // ascii space
    }
    int zero_char = (desc2.width() == 4) ? 0 : 0x30;    // asci zero

    /*
        A scale and an exponent provide the same functionality.
        Compare the source and destinatation scales.

        If the scaling factors don't match, we'll either right shift
        or left shift the source.  Vacated or created positions will
        be filled with zeros.

        We'll also right shift the source (e.g. pad the source with
        leading zeros) if the destination has more bytes than the source.

        When we copy digits to the destination, we do the following:
            1) Send a leading sign byte if needed.
            2) Send any leading zeros implied by a right shift.
            3) Send as many of the source digits as will fit.  The
               number of digits that will fit is defined as the size
               of the destinaton minus bytes required by the other
               four criteria here.
            4) Send any trailing zeros implied by a left shift.
            5) Send any trailing sign or exponent.

        When not all of the source digits will fit in the destination:
            Digits lost due to right shifting because of differing
            scales represent loss of low order digits and result
            in setting the truncation flag unless rounding.  AL39
            doesn't mention whether truncating zeros should count
            as truncation.
            Other source digits that won't fit in the destinaton
            represent an overflow.  Note that leading zeros in the
            source are not counted in the source length for purposes
            of the overflow check.
    */

    int n_src = src.n_coe;      // digit bytes, excluding signs and exponents
    int n_src_lz = src.n_lz;    // number of leading zeros in source
    int n_src_val = n_src - n_src_lz;

    int sf1 = src.exp;
    int sf2 = (desc2.s() == 0) ? 0 : desc2.sf();
    if (sf1 != sf2) {
        auto_break = 1;
        log_msg(INFO_MSG, moi, "Non equal exponents; auto breakpoint.\n");
    }
    log_msg(DEBUG_MSG, moi, "Src has %d coefficient digits with %d leading zeros: %.100s.\n", n_src, n_src_lz, src.coe_text);
    log_msg(DEBUG_MSG, moi, "Src has exponent or scaling factor %d.\n", sf1);
    log_msg(DEBUG_MSG, moi, "Dest has %s %d.\n",
        (desc2.s() == 0) ? "exponent" : "scaling factor", sf2);
    int sf_diff = sf2 - sf1;
    int n_rshift, n_lshift;
    if (desc2.s() == 0) {
        // dest is float, so copy sf to exp
        n_rshift = 0;
        n_lshift = 0;
        // exp has more bits than scale factor does, so no overflow on
        // the exp here (but see below)
        sf2 = sf1;
        log_msg(DEBUG_MSG, moi, "Dest will have exponent %d.\n", sf2);
    } else {
        // dest is fixed scale, so shift
        n_rshift = (sf_diff > 0) ? sf_diff : 0;
        n_lshift = (sf_diff < 0) ? - sf_diff : 0;
    }
    log_msg(DEBUG_MSG, moi, "Left shift is %d; Right shift is %d\n",
        n_lshift, n_rshift);

    int n_dest = desc2.ndigits(); // digit bytes, excluding signs and exponents
    int n_dest_avail = n_dest - n_rshift;   // can't count leading must-be-zero
    n_dest_avail -= n_lshift;   // can't count low zeros shifted in
    log_msg(DEBUG_MSG, moi, "Src has %d significant digits.\n", n_src_val);
    log_msg(DEBUG_MSG, moi, "Dst has %d digit bytes available.\n", n_dest_avail);
    if (n_src_val < n_dest_avail) {
        n_rshift += n_dest_avail - n_src_val;
        n_dest_avail = n_src_val;
        log_msg(DEBUG_MSG, moi, "Right shift (left lz pad) changed to %d\n", n_rshift);
        log_msg(DEBUG_MSG, moi, "Dst now has %d digit bytes available.\n", n_dest_avail);
    } else if (n_src_val > n_dest_avail) {
        if (sf_diff < 0 || n_src_val - sf_diff > n_dest_avail) {
            if (desc2.s() == 0) {
                // floating point -- drop least significant digits and bump exp
                sf2 = sf1 + (n_src_val - n_dest_avail);
                log_msg(DEBUG_MSG, moi, "Dest will have exponent %d.\n", sf2);
            } else {
                // fixed point and it doesn't fit
                IR.overflow = 1;
                fault_gen(overflow_fault);
                return 1;
            }
        } // else will round or truncate below
    }

    // Write the output to a temp buffer
    //
    // TODO: read/access dest chars here to avoid instr restart and
    // resulting possibly incorrect overlapping string behavior
    // actually, we should preserve input or rather output -- the
    // interrupt handler could re-write our input

    uint out_ybuf[64];          // ops can be up to 63 4-bit or 9-bit nibbles
    int outp = 0;
    if (desc2.s() == 00 || desc2.s() == 1) {
        out_ybuf[outp++] = dst_sign | sign_fill;
        log_msg(DEBUG_MSG, moi, "Send leading sign.\n");
    }
    int inp = 0;
    for (int i = 0; i < n_dest; ++i) {
        if (n_rshift > 0) {
            -- n_rshift;
            out_ybuf[outp] = zero_char;
            log_msg(DEBUG_MSG, moi, "Send leading zero to position %d.\n", outp);
            ++ outp;
        } else if (n_dest_avail > 0) {
            unsigned digit = src.coe[inp++];
            out_ybuf[outp++] = digit | fill;
            log_msg(DEBUG_MSG, moi, "Send data byte %#o (digit %d) from coe position %d to dest pos %d\n",
                digit | fill, digit & 0xf, inp - 1, outp - 1);
        } else if (n_lshift > 0) {
            -- n_lshift;
            out_ybuf[outp++] = zero_char;
            log_msg(DEBUG_MSG, moi, "Send trailing zero to positon %d.\n", outp - 1);
        } else {
            log_msg(ERR_MSG, moi, "Internal error, no work, outp=%d, n_dest=%d.\n", outp, n_dest);
            cancel_run(STOP_BUG);
        }
    }

    // Check for rounding or truncation and update the temporary buffer
    int head = 0;
    int tail = src.n_coe - 1;
    if (inp > tail) {
        if (inp != tail + 1) {
            log_msg(ERR_MSG, moi, "Internal error, inp = %d, tail = %d\n", inp, tail);
            cancel_run(STOP_BUG);
        }
        IR.truncation = 0;  // inp == tail + 1, consumed all source
    } else {
        // Unused source digits (we tested for overflow earlier, these
        // are lost low digits).
        log_msg(DEBUG_MSG, moi, "Checking src bytes %d .. %d for rounding or trunc.\n", head, tail);
        log_msg(INFO_MSG, moi, "Unused source digit; auto breakpoint.\n");
        auto_break = 1;
        if (rounding) {
            if ((src.coe[inp] & 0xf) >= 5) {
                int out_head = (desc2.s() == 00 || desc2.s() == 1) ? 1 : 0;
                int p;
                for (p = outp - 1; p >= out_head; --p) {
                    if ((out_ybuf[p] & 0xf) == 9) {
                        out_ybuf[p] = 0;
                    } else {
                        ++ out_ybuf[p];     // works even if filled.
                        break;
                    }
                }
                if (p < out_head) {
                    // Found, for example, 99.5 => 100
                    if (desc2.s() == 0) {
                        // Change, for example, 100*10^(x) ==> 10*10^(x+1)
                        ++ out_ybuf[out_head];  // increment 0->1, preserve fill
                        ++ sf2;
                        log_msg(INFO_MSG, moi, "Floating point round yields one significant digit.\n");
                    } else {
                        IR.overflow = 1;
                        fault_gen(overflow_fault);
                        return 1;
                    }
                }
            }
            IR.truncation = 0;
        } else {
            int all_zero = 1;
            for (int i = inp; i <= tail; ++i) {
                if ((src.coe[i] & 0xf) != 0) {
                    all_zero = 0;
                    break;
                }
            }
            IR.truncation = ! all_zero;
            if (IR.truncation && trunc_fe) {
                fault_gen(overflow_fault);  // truncation
                return 1;
            }
        }
    }
    // Handle trailing sign or exponent byte(s), if any
    if (desc2.s() == 0) {
        if (desc2.s() == 0) {
            if (sf2 < -128) {
                IR.exp_underflow = 1;
                log_msg(INFO_MSG, moi, "Exponent underflow; auto breakpoint.\n");
                auto_break = 1;
            } else if (sf2 > 127) {
                IR.exp_overflow = 1;
                log_msg(INFO_MSG, moi, "Exponent overflow; auto breakpoint.\n");
                auto_break = 1;
            }
            if (desc2.width() == 4) {
                out_ybuf[outp++] = (sf2 >> 4) & 0xff;
                out_ybuf[outp++] = sf2 & 0xff;
                log_msg(DEBUG_MSG, moi, "Send two exponent nibbles.\n");
            } else {
                out_ybuf[outp++] = sf2 & 0x1ff;
                log_msg(DEBUG_MSG, moi, "Send exponent byte.\n");
            }
        }
    } else if (desc2.s() == 2) {
        out_ybuf[outp++] = dst_sign | sign_fill;
        log_msg(DEBUG_MSG, moi, "Send trailing sign.\n");
    }

    /*
        write output
    */

    outp = 0;
    int first = 0;
    int zero = 1;
    int n_exp = (desc2.width() == 9) ? 1 : 2;
    int ret = 0;
    while (desc2.n() > 0) {
        if (first && (desc2.s() == 0 || desc2.s() == 1)) {
            // sign byte
        } else if (desc2.s() == 0 && desc2.n() <= n_exp) {
            // exp byte(s)
        } else {
            if ((out_ybuf[outp] & 0xf) != 0)
                zero = 0;
        }
        ret = desc2.put(out_ybuf[outp++]);
        if (ret != 0)
            break;
        first = 0;
    }
    // write unsaved data (if any)
    if (ret == 0)
        if (desc2.flush() != 0)
            return 1;

    log_msg(DEBUG_MSG, moi, "Wrote %d bytes.\n", outp);

    return ret;
}

// ============================================================================

#if 1
static int _op_dv3d(const instr_t* ip);
int op_dv3d(const instr_t* ip)
{
    extern DEVICE cpu_dev;
    int saved_debug = opt_debug;
    int saved_dctrl = cpu_dev.dctrl;
    ++ opt_debug; ++ cpu_dev.dctrl;
    int ret = _op_dv3d(ip);
    opt_debug = saved_debug;
    cpu_dev.dctrl = saved_dctrl;
    return ret;
}
#endif


static int _op_dv3d(const instr_t* ip)
{
    const char* moi = "OPU::dv3d";

    // FIXME: leaks memory

    cpu.irodd_invalid = 1;

    flag_t sign_ctl = ip->addr >> 17;   // "P"
    flag_t trunc = (ip->addr >> 8) & 1; // "T"
    flag_t round = (ip->addr >> 7) & 1; // "R"

    log_msg(INFO_MSG, moi, "P(4-bit sign ctl) = %c, T(trunc flt enable) = %c, R(rounding) = %c.\n",
        sign_ctl ? 'Y' : 'N',
        trunc ? 'Y' : 'N',
        round ? 'Y' : 'N');

    uint mf2bits = ip->addr & MASKBITS(7);
    uint mf3bits = (ip->addr >> 9) & MASKBITS(7);

    log_msg(DEBUG_MSG, moi, "mf1 = %s\n", mf2text(&ip->mods.mf1));

    eis_mf_t mf2;
    (void) parse_mf(mf2bits, &mf2);
    log_msg(DEBUG_MSG, moi, "mf2 = %s\n", mf2text(&mf2));

    eis_mf_t mf3;
    (void) parse_mf(mf3bits, &mf3);
    log_msg(DEBUG_MSG, moi, "mf3 = %s\n", mf2text(&mf3));

    t_uint64 word1, word2, word3;
    if (fetch_mf_ops(&ip->mods.mf1, &word1, &mf2, &word2, &mf3, &word3) != 0)
        return 1;

    num_desc_t desc1(ip->mods.mf1, word1, 1);
    char msg_buf[100];
    log_msg(INFO_MSG, moi, "desc1: %s\n", desc1.to_text(msg_buf));
    num_desc_t desc2(mf2, word2, 1);
    log_msg(INFO_MSG, moi, "desc2: %s\n", desc2.to_text(msg_buf));
    num_desc_t desc3(mf3, word3, 1);
    log_msg(INFO_MSG, moi, "desc3: %s\n", desc3.to_text(msg_buf));

    // Determine how many byte positions are in the source and the
    // destination, not counting the sign bytes and exponent bytes

    int divisor_tot = desc1.n();
    int n_divisor = desc1.ndigits(); // digit bytes, excluding signs and exponents
    int num_tot = desc2.n();
    int n_num = desc2.ndigits();    // digit bytes, excluding signs and exponents

    int n_dest = desc3.ndigits();   // digit bytes, excluding signs and exponents

    log_msg(DEBUG_MSG, moi, "Dividend is type %d with %d digits with %d total bytes.\n",
        desc2.s(), n_num, desc2.n());
    log_msg(DEBUG_MSG, moi, "Divisor is type %d with %d digits with %d total bytes.\n",
        desc1.s(), n_divisor, desc1.n());
    log_msg(DEBUG_MSG, moi, "Quotient result is is type %d with %d digits with %d total bytes.\n",
        desc3.s(), n_dest, desc3.n());


    int ret = 0;
    dec_t n1;
    if (read_dec(desc1, n1) != 0)
        ret = 1;
    dec_t n2;
    if (read_dec(desc2, n2) != 0)
        ret = 1;

    decNumber* divisor_p = new_decNumber();
    decNumberZero(divisor_p);
    decNumber* divisor_copy_p = new_decNumber();
    if (multics_to_dec(n1, *divisor_p) != 0)
        ret = 1;
    memcpy(divisor_copy_p, divisor_p, sizeof(*divisor_p));
    if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
        log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
    }
    debug_show(*divisor_p, "divisor");
    if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
        log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
    }
    decNumber* dividend_p = new_decNumber();
    decNumberZero(dividend_p);
    if (multics_to_dec(n2, *dividend_p) != 0)
        ret = 1;
    if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
        log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
        //char buf1[100];
        //char buf2[100];
        //decNumberToString(&divisor_copy, buf1);
        //decNumberToString(&divisor, buf2);
        //log_msg(ERR_MSG, moi, "divisor damaged, line %d; was '%s', now '%s'!!!\n", __LINE__, buf1, buf2);
        debug_show(*divisor_copy_p, "divisor orig");
        debug_show(*divisor_p, "divisor now");
    }
    debug_show(*dividend_p, "dividend");
    if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
        log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
    }
    debug_show(*divisor_p, "divisor re-display");
    if (ret == 1) {
        fault_gen(illproc_fault);   // FIXME: somewhat bogus
        return 1;
    }
    flush_logs();
    {
        char buf1[1000];    // must be 14 chars more than number of
        char buf2[1000];    // coefficient digits

        if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
            log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
        }
        decNumberToString(dividend_p, buf2);
        log_msg(NOTIFY_MSG, moi, "DecNum dividend is '%s'.\n", buf2);
        flush_logs();

        if (memcmp(divisor_copy_p, divisor_p, sizeof(*divisor_p)) != 0) {
            log_msg(ERR_MSG, moi, "divisor damaged, line %d!!!\n", __LINE__);
        }
        decNumberToString(divisor_p, buf1);
        log_msg(NOTIFY_MSG, moi, "DecNum divisor is '%s'.\n", buf1);
        flush_logs();
        // log_msg(NOTIFY_MSG, moi, "DecNum values are '%s' divided by '%s'.\n", buf2, buf1);
    }
    flush_logs();

    decContext context;
    multics_decContext(context, 0);

    int nq;
    if (desc3.s() == 0) {
        nq = desc3.n();
        decNumber* cmp = new_decNumber();
        decNumberZero(cmp);
        decNumberCompare(cmp, divisor_p, dividend_p, &context);
        int32 c = decNumberToInt32(cmp, &context);
        if (c == 1)
            --nq;
        free(cmp);
    } else
        nq = (num_tot - n1.n_lz + 1)  - (divisor_tot - n2.n_lz) + (n2.exp - n1.exp - desc3.sf());
    if (round)
        ++ nq;

    if (n1.is_zero || nq > 63) {
        if (n1.is_zero)
            log_msg(ERR_MSG, moi, "Divide Check fault; divide by zero.\n");
        else
            log_msg(ERR_MSG, moi, "Divide Check fault; need more than 63 digits for quotient.\n");
        fault_gen(div_fault);
        return 1;
    }

    decNumber* result_p = new_decNumber();
    decNumberZero(result_p);
    multics_decContext(context, 0); // FIXME: Should we adjust emax,emin by SF?
    {
        char buf1[100]; // must be 14 chars more than number of
        char buf2[100]; // coefficient digits
        decNumberToString(dividend_p, buf2);
        decNumberToString(divisor_p, buf1);
        log_msg(NOTIFY_MSG, moi, "Calling decnum divide for %s/%s.\n", buf2, buf1);
        decNumberDivide(result_p, dividend_p, divisor_p, &context);
        char buf3[100];
        decNumberToString(result_p, buf3);
        log_msg(NOTIFY_MSG, moi, "Result of %s/%s is %s.\n", buf2, buf1, buf3);
    }


    // FIXME -- check if write_decimal constraints match dv3d as well as mvn

#if 0
    need = # of integer digits of result
example result format: 3 digits an scale of 2 --- ??? x 10^2
    100 -->  100 x 10^0
  1,000 -->  100 x 10^1
 10,000 -->  100 x 10^2
100,000 --> 1000 x 10^2 -- overflow
    need += result.exp - sf3
    desc3.
    n3
    if n3 s coeff num digits too small forresult coeff as scaled by sf3...
        overflow
    if n3 not loargs enough...
            if ! round
                truncate

    -- add char range tests to dividend/divisor reads & illproc
#endif

    dec_t quot(*result_p);
    if (write_decimal(quot, desc3, round, trunc) != 0)
        return 1;

    log_msg(INFO_MSG, moi, "After copy finishes: desc1: %s\n", desc1.to_text(msg_buf));
    log_msg(INFO_MSG, moi, "After copy finishes: desc2: %s\n", desc2.to_text(msg_buf));
    log_msg(INFO_MSG, moi, "After copy finishes: desc3: %s\n", desc3.to_text(msg_buf));

    log_msg(ERR_MSG, moi, "Lightly tested; auto breakpoint\n");
    cancel_run(STOP_IBKPT);

    free(divisor_p);
    free(divisor_copy_p);
    free(dividend_p);
    free(result_p);

    PPR.IC += 4;
    return 1;
}

// ============================================================================

/*
    read_dec

    Use a num_desc_t to read a decimal string

    FIXME: The mvn opcode *seems* to need to know if the source 9-bit data
    had been high-bit filled or not.
*/

static int read_dec(num_desc_t& desc, dec_t& dec)
{
    const char* moi = "APU::read-decimal";

    int len = desc.n();     // total number of digits
    int n_coe;
    {
        int n_sign = (desc.s() == 3) ? 0 : 1;
        int n_exp = 0;
        if (desc.s() == 0)
            n_exp = (desc.width() == 4) ? 2 : 1;
        n_coe = desc.ndigits(); // aka (len - n_sign - n_exp) memory
        log_msg(DEBUG_MSG, moi, "Descriptor is type %d with %d digits, %d sign bytes, and %d exponent chars for %d total bytes.\n", desc.s(), n_coe, n_sign, n_exp, len);
    }

    // First read in the chars (either 4-bit or 9-bit)
    uint* yptr = dec.coe;
    char *textp = dec.coe_text;
    int first = 1;
    uint sign = 0;
    uint exp = 0;
    uint prev = 0;
    int n_lz = 0;
    int is_zero = 1;
    // dec.is_ascii = 1;
    while (desc.n()) {
        uint nibble;
        int ret = desc.get(&nibble);
        if (ret != 0) {
            log_msg(ERR_MSG, moi, "Error reading input.\n");
            return 1;
        }
        // log_msg(DEBUG_MSG, moi, "Read %d-bit nibble %#o (%#x).\n", desc.width(), nibble, nibble);
        if (first) {
            first = 0;
            if (desc.s() == 0 || desc.s() == 1) {
                sign = nibble;
                continue;
            }
        }
        int n = desc.n();
        if (n == 1 && desc.s() == 0 && desc.width() == 4) {
            prev = nibble;
            continue;
        } else if (n == 0) {
            // last char
            if (desc.s() == 2) {
                if ((nibble & 0xf) < 10) {
                    log_msg(ERR_MSG, moi, "Bad sign in decimal string.\n");
                    fault_gen(illproc_fault);
                    return 1;
                }
                sign = nibble;
                continue;
            }
            if (desc.s() == 0) {
                // floating point with one or two char exponent
                if (desc.width() == 9)
                    exp = nibble & 0xff;
                else
                    exp = (prev << 4) | (nibble & 0xf);
                continue;
            }
        }
        //if ((nibble & 0xf) == nibble)
        //  dec.is_ascii = 0;
        if ((nibble & 0xf) > 9) {
            log_msg(ERR_MSG, moi, "Non char %#o in decimal string.\n", nibble);
            fault_gen(illproc_fault);
            return 1;
        }
        if (is_zero) {
            if ((nibble & 0xf) == 0)
                ++ n_lz;
            else
                is_zero = 0;
        }
        *yptr++ = nibble;
        *textp++ = nibble | '0';
    }

    *yptr = 0;
    *textp = 0;

    int is_neg = 0;
    if (sign != 0) {
        if (desc.width() == 4)
            is_neg = sign == 015;
        else
            is_neg = sign == 055;
    }

    dec.n_lz = n_lz;
    dec.n_coe = n_coe;
    dec.sign_nibble = sign;
    dec.is_neg = is_neg;
    dec.is_zero = is_zero;
    if (desc.s() == 0)
        dec.exp = bits2num(8, exp);
    else
        dec.exp = desc.sf();

    // 64-bit integer can hold an unsigned 19-digit decimal integer
    // dec.coe_val = 0;

    log_msg(DEBUG_MSG, moi, "Coefficient %s%s with %d leading zeros.\n",
        (desc.s() == 3) ? "" : (dec.is_neg) ? "-" : "+",
        dec.coe_text, dec.n_lz);
    if (desc.s() == 0)
        log_msg(DEBUG_MSG, moi, "Exponent %d %03o\n", exp, exp);
    else
        log_msg(DEBUG_MSG, moi, "Scale %d %03o\n", exp, exp);

    return 0;
}


// ============================================================================


static int multics_to_dec(const dec_t& dec, decNumber& dnum)
{
    const char* moi = "APU::to_dec";

    int n_coe = dec.n_coe;  // total number of digits

    // char ybuf[72];   // 64 digits, 1 sign, 1 "E", signed 3 digit exponent, a nul
    char ybuf[1000];    // 64 digits, 1 sign, 1 "E", signed 3 digit exponent, a nul
    char* yptr = ybuf;
    if (dec.is_neg)
        *yptr++ = '-';
    strncpy(yptr, dec.coe_text, 64);
    ybuf[65] = 0;
    if (dec.exp != 0)
        sprintf(ybuf + strlen(ybuf), "E%d", dec.exp);

    // 64-bit integer can hold an unsigned 19-digit decimal integer
    // So, we could store smaller values as int64...

    decContext context;
    multics_decContext(context, 0);

    log_msg(DEBUG_MSG, moi, "Calling decNumber convert for '%s'.\n", ybuf);
    decNumberZero(&dnum);
    decNumberFromString(&dnum, (char*) ybuf, &context);
    if ((context.status & DEC_Errors) != 0) {
        log_msg(ERR_MSG, moi, "Error converting '%s'.\n", ybuf);
        return 1;
    }
    log_msg(DEBUG_MSG, moi, "Converted result: %d digits.\n", dnum.digits);
    log_msg(DEBUG_MSG, moi, "Converted result: exponent %d.\n", dnum.exponent);
    log_msg(DEBUG_MSG, moi, "Converted result: flags %#x (negative = %c).\n",
        dnum.bits, decNumberIsNegative(&dnum) ? 'Y' : 'N');
#if 0
    for (int i = 0; i < dnum.digits; ++i) {
        log_msg(DEBUG_MSG, moi, "Converted result: digit %d: %#x\n", i, dnum.lsu[i]);
    }
#endif
    log_msg(DEBUG_MSG, moi, "Calling decNumberToString.\n");
    flush_logs();
    char buf[1000]; // coefficient digits
    decNumberToString(&dnum, buf);
    log_msg(DEBUG_MSG, moi, "Converted result: string value is: %s.\n", buf);
    flush_logs();

    return 0;
}

// ----------------------------------------------------------------------------

static void debug_show(const decNumber& dnum, const char* msg)
{
    const char* moi = "EIS";

    log_msg(DEBUG_MSG, moi, "Dumping DecNumber -- %s\n", msg);
    log_msg(DEBUG_MSG, moi, "Converted result: %d digits.\n", dnum.digits);
    log_msg(DEBUG_MSG, moi, "Converted result: exponent %d.\n", dnum.exponent);
    log_msg(DEBUG_MSG, moi, "Converted result: flags %#x (negative = %c).\n",
        dnum.bits, decNumberIsNegative(&dnum) ? 'Y' : 'N');
#if 0
    for (int i = 0; i < dnum.digits; ++i) {
        log_msg(DEBUG_MSG, moi, "Converted result: digit %d: %#x\n", i, dnum.lsu[i]);
    }
#endif
    log_msg(DEBUG_MSG, moi, "Calling decNumberToString.\n");
    flush_logs();
    char buf[1000]; // coefficient digits
    decNumberToString(&dnum, buf);
    log_msg(DEBUG_MSG, moi, "Converted result: string value is: %s.\n", buf);
    flush_logs();
}

// ============================================================================

dec_t::dec_t(decNumber& decnum)
{
    const char* moi = "EIS::decimal";

    // coe_val = ~ 0;
    // unsigned coe[65];        // original raw data, 0..9 or '0' .. '9'

    n_coe = decnum.digits;
    if (n_coe > 63) {
        log_msg(ERR_MSG, moi, "Source decnum has too many digits.\n");
        cancel_run(STOP_BUG);
    }
    uint8 bcd[63];
    decNumberGetBCD(&decnum, bcd);
    n_lz = 0;
    is_zero = 1;
    for (int i = 0; i < n_coe; ++i) {
        coe[i] = bcd[i];
        coe_text[i] = bcd[i] | '0';
        if (is_zero) {
            if (bcd[i] == 0)
                ++ n_lz;
            else
                is_zero = 0;
        }
    }
    coe_text[n_coe] = 0;

    width = 4;
    is_neg = decNumberIsNegative(&decnum);
    sign_nibble = (is_neg) ? 015 : 014;
    is_zero = decNumberIsZero(&decnum);
    exp = decnum.exponent;
}

// ============================================================================

