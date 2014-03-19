/*
    apu.c -- The appending unit (APU) -- "address appending"

    Provides addressing support including virtual memory support.
    See also eis_desc.cpp for address descriptors used by EIS multi-word
    instructions.

    Note that the typical way of changing from non-segmented, non-paging
    ABSOLUTE mode to segmenting and paging APPEND mode is via the address
    mode of an operand of an instruction.  The CPU stays in the new mode
    after the operand fetch.

    CAVEAT
        The documentation says that the appending unit HW controls fault
        recognition.  We handle recognition in the CPU source file in
        the cycle loop.
*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/


#include "hw6180.h"

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 };

// CPU allows [2^6 .. 2^12]; multics uses 2^10
static const int page_size = 1024;

typedef struct {    // TODO: having a temp CA is ugly and doesn't match HW
    flag_t more;
    enum atag_tm special;
} ca_temp_t;

static int compute_addr(const instr_t *ip, ca_temp_t *ca_tempp);
static int addr_append(t_uint64 *wordp);
static int do_its_itp(const instr_t* ip, ca_temp_t *ca_tempp, t_uint64 word01);
static int page_in(uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp);
static void decode_PTW(t_uint64 word, PTW_t *ptwp);
static int set_PTW_used(uint addr);
static void decode_SDW(t_uint64 word0, t_uint64 word1, SDW_t *sdwp);
// static SDW_t* get_sdw(void);
static SDWAM_t* page_in_sdw(void);
static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp);
static void register_mod(uint td, uint off, uint *bitnop, int nbits);
static void dump_descriptor_table(void);

//=============================================================================

static inline uint max3(uint a, uint b, uint c)
{
    return (a > b) ?
        ((a > c) ? a : c) :
        ((b > c) ? b : c);
}

//=============================================================================

/*
 * set_addr_mode()
 *
 * Put the CPU into the specified addressing mode.   This involves
 * setting a couple of IR flags and the PPR priv flag.
 *
 * TODO: set_addr_mode() probably belongs in the CPU source file.
 *
 */

void set_addr_mode(addr_modes_t mode)
{
    if (mode == ABSOLUTE_mode) {
        IR.abs_mode = 1;
        // FIXME: T&D tape section 3 wants not-bar-mode true in absolute mode,
        // but section 9 wants false?
        IR.not_bar_mode = 1;    
        PPR.P = 1;
        if (opt_debug) log_msg(DEBUG_MSG, "APU", "Setting absolute mode.\n");
    } else if (mode == APPEND_mode) {
        if (opt_debug) {
            if (! IR.abs_mode && IR.not_bar_mode)
                log_msg(DEBUG_MSG, "APU", "Keeping append mode.\n");
            else
                log_msg(DEBUG_MSG, "APU", "Setting append mode.\n");
        }
        IR.abs_mode = 0;
        IR.not_bar_mode = 1;
    } else if (mode == BAR_mode) {
        IR.abs_mode = 0;
        IR.not_bar_mode = 0;
        log_msg(WARN_MSG, "APU", "Setting bar mode.\n");
    } else {
        log_msg(ERR_MSG, "APU", "Unable to determine address mode.\n");
        cancel_run(STOP_BUG);
    }
}


//=============================================================================


/*
 * addr_modes_t get_addr_mode()
 *
 * Report what mode the CPU is in.
 * This is determined by examining a couple of IR flags.
 *
 * TODO: get_addr_mode() probably belongs in the CPU source file.
 *
 */

addr_modes_t get_addr_mode()
{
    if (IR.abs_mode)
        return ABSOLUTE_mode;

    if (IR.not_bar_mode == 0)
        return BAR_mode;

    return APPEND_mode;
}

//=============================================================================

/*
 * is_priv_mode()
 *
 * Report whether or or not the CPU is in privileged mode.
 * True if in absolute mode or if priv bit is on in segment TPR.TSR
 *
 * TODO: is_priv_mode() probably belongs in the CPU source file.
 *
 */

int is_priv_mode()
{
    if (IR.abs_mode)
        return 1;
    SDW_t *SDWp = get_sdw();    // Get SDW for segment TPR.TSR
    if (SDWp == NULL) {
        if (cpu.apu_state.fhld) {
            // TODO: Do we need to check cu.word1flags.oosb and other flags to
            // know what kind of fault to gen?
            fault_gen(acc_viol_fault);
            cpu.apu_state.fhld = 0;
        }
        log_msg(WARN_MSG, "APU::is-priv-mode", "Segment does not exist?!?\n");
        cancel_run(STOP_BUG);
        return 0;   // arbitrary
    }
    if (SDWp->priv)
        return 1;
    if(opt_debug>0)
        log_msg(DEBUG_MSG, "APU", "Priv check fails for segment %#o.\n", TPR.TSR);
    return 0;
}

//=============================================================================

/*
 * reg2text()
 *
 * Format a "register modification" value for display.
 *
 */

void reg2text(char* buf, uint r)
{
    switch(r) {
        case 0: *buf = 0; return;
        case 1: strcpy(buf, "au"); return;
        case 2: strcpy(buf, "qu"); return;
        case 3: strcpy(buf, "du"); return;
        case 4: strcpy(buf, "ic"); return;
        case 5: strcpy(buf, "al"); return;
        case 6: strcpy(buf, "ql"); return;
        case 7: strcpy(buf, "dl"); return;
        case 010: strcpy(buf, "x0"); return;
        case 011: strcpy(buf, "x1"); return;
        case 012: strcpy(buf, "x2"); return;
        case 013: strcpy(buf, "x3"); return;
        case 014: strcpy(buf, "x4"); return;
        case 015: strcpy(buf, "x5"); return;
        case 016: strcpy(buf, "x6"); return;
        case 017: strcpy(buf, "x7"); return;
        default:
            sprintf(buf, "<illegal reg mod td=%#o>", r);
    }
}

//=============================================================================

/*
 * mod2text()
 *
 * Format an instruction address modifer for display.
 *
 */

void mod2text(char *buf, uint tm, uint td)
{
    if (buf == NULL)
        return;
    switch(tm) {
        case 0: // R
            if (td == 0)
                *buf = 0;
            else {
                *buf = ',';
                reg2text(buf+1, td);
            }
            return;
        case 1: // RI
            *buf = ',';
            if (td == 3 || td == 7)
                sprintf(buf, "<illegal RI mod td=%#o>", td);
            else {
                reg2text(buf+1, td);
                strcpy(buf+strlen(buf), "*");
            }
            return;
        case 2:
            switch(td) {
                case 0: strcpy(buf, ",fl"); return;
                case 1: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
                case 2: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
                case 3: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
                case 4: strcpy(buf, ",sd"); return;
                case 5: strcpy(buf, ",scr"); return;
                case 6: strcpy(buf, ",f2"); return;
                case 7: strcpy(buf, ",f3"); return;
                case 010: strcpy(buf, ",ci"); return;
                case 011: strcpy(buf, ",i"); return;
                case 012: strcpy(buf, ",sc"); return;
                case 013: strcpy(buf, ",ad"); return;
                case 014: strcpy(buf, ",di"); return;
                case 015: strcpy(buf, ",dic"); return;
                case 016: strcpy(buf, ",id"); return;
                case 017: strcpy(buf, ",idc"); return;
                default: sprintf(buf, "<illegal IT tag td=%#o>", td);
            }
            return;
        case 3: // IR
            buf[0] = ',';
            buf[1] = '*';
            reg2text(buf+2, td);
            return;
        default:
            sprintf(buf, "<illegal tag tm=%#o td=%#o>", tm, td);
            return;
    }
}

//=============================================================================

/*
 * instr2text()
 *
 * Format a (decoded) instruction for display.
 *
 * WARNING
 *     Returned string is a static buffer and is only valid until the next call.
 */

char* instr2text(const instr_t* ip)
{
    static char buf[100];
    uint op = ip->opcode;
    char *opname = opcodes2text[op];
    if (opname == NULL) {
        strcpy(buf, "<illegal instr>");
    } else if (ip->is_eis_multiword) {
        uint32 offset = ip->addr;
        int32 soffset = sign18(ip->addr);
        sprintf(buf, "%s, variable 0%06o, inhibit %u, mf1={ar=%d, rl=%d, id=%d, reg=0%o}",
            opname, 
            offset, ip->inhibit,
            ip->mods.mf1.ar, ip->mods.mf1.rl, ip->mods.mf1.id, ip->mods.mf1.reg);
    } else {
        char mod[40];
        if (ip->mods.single.tag == 0)
            strcpy(mod, "\"\"");
        else
            mod2text(mod, ip->mods.single.tag >> 4, ip->mods.single.tag & 017);
        if (ip->mods.single.pr_bit == 0) {
            uint32 offset = ip->addr;
            int32 soffset = sign18(ip->addr);
            sprintf(buf, "%s, offset 0%06o(%+d), inhibit %u, tag %s",
                opname, 
                offset, soffset, 
                ip->inhibit, mod);
        } else {
            uint pr = ip->addr >> 15;
            int32 offset = ip->addr & MASKBITS(15);
            int32 soffset = sign15(offset);
            sprintf(buf, "%s, PR %d, offset 0%06o(%+d), inhibit %u, tag %s",
                opname, 
                pr, offset, soffset, 
                ip->inhibit, mod);
        }
    }
    return buf;
}

//=============================================================================

/*
 * print_instr()
 *
 * Format a word containing an (un-decoded) instruction for display.
 *
 * WARNING
 *     Returned string is a static buffer and is only valid until the next call.
 */

char* print_instr(t_uint64 word)
{
    instr_t instr;
    word2instr(word, &instr);
    return instr2text(&instr);
}

//=============================================================================

/*
 * get_address(y, xbits, ar, reg, nbits, addrp, bitnop, minaddrp, maxaddrp)
 *
 * Translate an EIS instruction's "y-address" into a 24-bit absolute
 * memory address.
 *
 * Called only (indirectly) by the OPU via get_eis_indir_ptr()
 *
 * Returns the absolute address given an address, 'ar' flag, 'reg' modifier,
 * and "nbits" width.  Nbits is the data size and is used only for reg
 * modifications.
 * Argument "ar" should be negative to use current TPR or non-negative to
 * use a pointer/address register.
 * Xbits is obsolete.  It allowed the caller to provide an additional offset
 *
 * WARNING
 *     Caller must not advanced the returned address below the minimum or
 *     maximum page or segment offsets.
 *
 * FIXME/TODO: This function should probably be replaced (or make use of)
 * the decode_eis_address() and get_ptr_address() functions that were
 * derived from it.
 */
    
int get_address(uint y, uint xbits, flag_t ar, uint reg, int nbits, uint *addrp, uint* bitnop, uint *minaddrp, uint* maxaddrp)
{
    // Note:
        // EIS indirect pointers to operand descriptors use PR registers.
        // However, operand descriptors use AR registers according to the
        // description of the AR registers and the description of EIS operand
        // descriptors.   However, the description of the MF field
        // claims that operands use PR registers.   The AR doesn't have a
        // segment field.  Emulation confirms that operand descriptors 
        // need to be fetched via segments given in PR registers.

    char *moi = "APU::get-addr";

    addr_modes_t addr_mode = get_addr_mode();
    uint offset;
    uint saved_PSR = 0, saved_PRR = 0, saved_CA = 0, saved_bitno = 0;

    if (ar) {
        if (addr_mode != APPEND_mode) {
            log_msg(WARN_MSG, moi, "Illegal usage of non append mode.\n");
            log_msg(INFO_MSG, moi, "Args were y=%06o, ar=%d, reg=%d, nbits=%d\n",
                y, ar, reg, nbits);
            cancel_run(STOP_BUG);
            return -1;
        } else {
            saved_PSR = TPR.TSR;
            saved_PRR = TPR.TRR ;
            saved_CA = TPR.CA;
            saved_bitno = TPR.TBR;
            //
            uint n = y >> 15;
            int32 soffset = sign15(y & MASKBITS(15));
            TPR.TSR = AR_PR[n].PR.snr;
            TPR.TRR = max3(AR_PR[n].PR.rnr, TPR.TRR, PPR.PRR);
            offset = AR_PR[n].wordno + soffset;
            TPR.TBR = AR_PR[n].PR.bitno;
            TPR.TBR += xbits;
            if (TPR.TBR >= 36) {
                offset += TPR.TBR / 36;
                TPR.TBR %= 36;
            }
            TPR.CA = offset; TPR.is_value = 0;
            if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Using PR[%d]: TSR=0%o, TRR=0%o, offset=0%o(0%o+0%o), bitno=0%o\n",
                n, TPR.TSR, TPR.TRR, offset, AR_PR[n].wordno, soffset, TPR.TBR);
            *(uint*)bitnop = TPR.TBR;
        }
    } else {
        offset = y;
        // int32 sofset = sign18(y);
        *bitnop = xbits;
    }

    if (reg != 0) {
        // Caller is specifying a register mod (from an EIS MF)
        saved_CA = TPR.CA;
        TPR.CA = 0;
        uint o = offset;
        uint bits = *bitnop;
        register_mod(reg, offset, bitnop, nbits);
        offset = TPR.CA;
        TPR.CA = saved_CA;
        if (bits != *bitnop || bits != 0 || *bitnop != 0) {
            int err = (int) *bitnop < 0 || *bitnop > 35 || (int) bits < 0 || bits > 35;
            log_msg(err ? ERR_MSG : DEBUG_MSG, moi, "Register mod 0%o: offset was 0%o, now 0%o; bit offset was %d, now %d.\n", reg, o, offset, bits, *bitnop);
            if (err) {
                log_msg(ERR_MSG, moi, "Bit offset %d and/or %d outside range of 0..35\n", bits, *bitnop);
                cancel_run(STOP_BUG);
            }
        } else
            log_msg(DEBUG_MSG, moi, "Register mod 0%o: offset was 0%o, now 0%o\n", reg, o, offset);
        //log_msg(WARN_MSG, moi, "Auto-breakpoint\n");
        //cancel_run(STOP_IBKPT);
    }

    uint perm = 0;  // FIXME: need to have caller specify
    int ret = 0;
    if (addr_mode == APPEND_mode) {
        ret = page_in(offset, perm, addrp, minaddrp, maxaddrp);
        if (ret != 0) {
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "page-in faulted\n");
        }
        if (ar) {
            TPR.TSR = saved_PSR;
            TPR.TRR = saved_PRR;
            TPR.CA = saved_CA;
            TPR.TBR = saved_bitno;
        }
    }

    return ret;
}

//=============================================================================

/*
 * decode_eis_address()
 *
 * See prior function (get_address) for comments
 *
 * FIXME: Have callers not pass ringp etc and have them inspect TPR instead
 */

int decode_eis_address(uint y, flag_t ar, uint reg, int nbits, uint *ringp, uint *segnop, uint *offsetp, uint *bitnop)
{
    char *moi = "APU::decode-eis-addr";

    addr_modes_t addr_mode = get_addr_mode();

    int offset;     // might be negative during intermediate calcuations
    if (ar) {
        if (addr_mode != APPEND_mode) {
            log_msg(WARN_MSG, moi, "Unexpected usage of non append mode.\n");
            cancel_run(STOP_BUG);
            return -1;
        }
        uint n = y >> 15;
        int32 soffset = sign15(y & MASKBITS(15));
        *segnop = AR_PR[n].PR.snr;
        *ringp = max3(AR_PR[n].PR.rnr, TPR.TRR, PPR.PRR);
        *bitnop = AR_PR[n].PR.bitno;
        offset = AR_PR[n].wordno + soffset;
        if (*bitnop >= 36) {
            offset += *bitnop / 36;
            *bitnop %= 36;
        }
        if(opt_debug>0) log_msg(DEBUG_MSG, moi,
            "Using PR[%d]: TSR=0%o, TRR=0%o, offset=0%o(0%o+0%o), bitno=0%o\n",
            n, *segnop, *ringp, offset, AR_PR[n].wordno, soffset, *bitnop);
    } else {
        *segnop = PPR.PSR;
        *ringp = max3(0, TPR.TRR, PPR.PRR);
        offset = y;     // unsigned
        *bitnop = 0;
        TPR.TSR = *segnop;
        TPR.TRR = *ringp;
        TPR.CA = y;
        TPR.TBR = *bitnop;
    }

    if (reg != 0) {
        // Caller is specifying a register mod (from an EIS MF)
        uint saved_CA = TPR.CA;
        uint saved_is_value = TPR.is_value; // probably unnecessary
        t_uint64 saved_value = TPR.value;   // probably unnecessary
        TPR.CA = 0;
        uint o = offset;
        uint bits = *bitnop;
        register_mod(reg, offset, bitnop, nbits);
#if 0
        if (TPR.is_value) {
            log_msg(WARN_MSG, moi, "Ignoring value type register mod; using CA %#o instead of value %#llo.\n", TPR.CA, TPR.value);
            cancel_run(STOP_WARN);
        }
#endif
        offset = TPR.CA;
        TPR.is_value = 0;

        if (bits != *bitnop || bits != 0 || *bitnop != 0) {
            int err = (int) *bitnop < 0 || *bitnop > 35 || (int) bits < 0 || bits > 35;
            log_msg(err ? ERR_MSG : DEBUG_MSG, moi, "Register mod 0%o: offset was 0%o, now 0%o; bit offset was %d, now %d.\n", reg, o, offset, bits, *bitnop);
            if (err) {
                log_msg(ERR_MSG, moi, "Bit offset %d and/or %d outside range of 0..35\n", bits, *bitnop);
                cancel_run(STOP_BUG);
            }
        } else
            log_msg(DEBUG_MSG, moi, "Register mod 0%o: offset was 0%o, now 0%o\n", reg, o, offset);
        //log_msg(WARN_MSG, moi, "Auto-breakpoint\n");
        //cancel_run(STOP_IBKPT);
    }
    *offsetp = offset;
    TPR.CA = offset;

    return 0;
}


//=============================================================================

/*
 * get_ptr_address()
 *
 * See prior function (get_address) for comments
 *
 * FIXME: Have callers not pass ringp etc and have them inspect TPR instead
 *
 */

int get_ptr_address(uint ringno, uint segno, uint offset, uint *addrp, uint *minaddrp, uint* maxaddrp)
{
    char *moi = "APU::get-ptr-addr";

    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != APPEND_mode) {
        log_msg(WARN_MSG, moi, "Illegal usage of non append mode.\n");
        log_msg(INFO_MSG, moi, "Args were ring=%d, seg=%d, offset=%#o\n",
            ringno, segno, offset);
        cancel_run(STOP_BUG);
        return -1;
    }

    uint saved_TSR = TPR.TSR;
    uint saved_TRR = TPR.TRR ;
    uint saved_bitno = TPR.TBR;

    uint saved_CA = TPR.CA;
    uint saved_is_value = TPR.is_value; // probably unnecessary
    t_uint64 saved_value = TPR.value;   // probably unnecessary

    TPR.TSR = segno;
    // TPR.TRR = max3(AR_PR[n].PR.rnr, TPR.TRR, PPR.PRR);
    TPR.TRR = ringno;
    TPR.TBR = 0;

    // if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Using TSR=0%o, TRR=0%o, offset=%#o.\n", TPR.TSR, TPR.TRR, offset);

    // TPR.CA = offset;

    uint perm = 0;  // FIXME: need to have caller specify
    int ret = page_in(offset, perm, addrp, minaddrp, maxaddrp);
    if (ret != 0) {
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "page-in faulted\n");
    }

    return ret;
}

//=============================================================================

/*
 * addr_mod()
 *
 * Called by OPU for most instructions
 * Generate 18bit computed address TPR.CA
 * Returns non-zero on error or group 1-6 fault
 *
 */

int addr_mod()
{

/*
    AL39,5-1: In abs mode, the appending unit is bypassed for instr
    fetches and *most* operand fetches and the final 18-bit computed
    address (TPR.CA) from addr prep becomes the main memory addr.

    ...
    Two modes -- absolute mode or appending mode.  [Various constucts] in
    absolute mode places the processor in append mode for one or more addr
    preparation cycles.  If a transfer of control is made with any of the
    above constructs, the proc remains in append mode after the xfer.
*/

    char *moi = "APU::addr-mod";
    const instr_t *ip = &cu.IR;

    // FIXME: do reg and indir word stuff first?

    TPR.is_value = 0;   // FIXME: Use "direct operand flag" instead
    TPR.value = 0135701234567;  // arbitrary junk ala 0xdeadbeef

    // Addr appending below

    addr_modes_t orig_mode = get_addr_mode();
    addr_modes_t addr_mode = orig_mode;
    int ptr_reg_flag = ip->mods.single.pr_bit;
    ca_temp_t ca_temp;  // FIXME: hack

    // FIXME: The bit 29 check below should only be done after a sequential
    // instr fetch, not after a transfer!  We're only called by do_op(),
    // so this criteria is *almost* met.   Need to detect transfers.
    // Figure 6-10 claims we update the TPR.TSR segno as instructed by a "PR"
    // bit 29 only if we're *not* doing a sequential instruction fetch.

    if (ptr_reg_flag == 0) {
        // TPR.TSR = PPR.PSR;   -- Loading TPR is done prior to instruction
        // TPR.TRR = PPR.PRR;   -- execution (see earlier comments)
        // The code to load TPR.CA is now in decode_instr()
    } else {
        if (cu.instr_fetch) {
            log_msg(ERR_MSG, moi,
                "Bit 29 is on during an instruction fetch.\n");
            cancel_run(STOP_WARN);
        }
        if (orig_mode != APPEND_mode) {
            log_msg(NOTIFY_MSG, moi,
                "Turning on APPEND mode for PR based operand.\n");
            set_addr_mode(addr_mode = APPEND_mode);
        }
        // The first page of AL-39, section 6 says the CA is loaded during
        // instruction decode.  However, the the discussion of "the use of
        // bit 29 in the instruction word" later in section 6 says that the
        // "preliminary step" by the CU also handles using the PR register
        // to set TPR.TSR and TPR.CA.  That seems to be a misstatement though
        // because the CU may not have access to the TPR and because the
        // initial part fig 8-1 shows that the APU handles the TPR.TSR.
        int32 offset = ip->addr & MASKBITS(15);
        int32 soffset = sign15(offset);
        uint pr = ip->addr >> 15;
        TPR.TSR = AR_PR[pr].PR.snr;
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR); // FIXME: Fig 8-1 excludes TPR
        TPR.CA = (AR_PR[pr].wordno + soffset) & MASK18;
        TPR.TBR = AR_PR[pr].PR.bitno;
        int err = (int) TPR.TBR < 0 || TPR.TBR > 35;
        if (opt_debug || err)
            log_msg(err ? WARN_MSG : DEBUG_MSG, moi,
                "Using PR[%d]: TSR=0%o, TRR=0%o, CA=0%o(0%o+0%o<=>%d+%d), bitno=0%o\n",
                pr, TPR.TSR, TPR.TRR, TPR.CA, AR_PR[pr].wordno, soffset,
                AR_PR[pr].wordno, soffset, TPR.TBR);
        if (err) {
            log_msg(ERR_MSG, moi, "Bit offset %d outside range of 0..35\n",
                TPR.TBR);
            cancel_run(STOP_BUG);
        }
    }

    if (cu.instr_fetch) {
        // Figure 8-1, below connector "L"
        uint p = is_priv_mode();    // Get priv bit from the SDW for TPR.TSR
        if (PPR.P != p)
            log_msg(INFO_MSG, moi, "PPR.P priv flag changing from %c to %c.\n",
                (PPR.P) ? 'Y' : 'N', p ? 'Y' : 'N');
        PPR.P = p;
    }

    int op = ip->opcode;
    int bit27 = op % 2;
    op >>= 1;
    if (bit27 == 0) {
        // The following instructions assign special meanings to the tag field.
        // Note that they still allow usage of PR registers though.
        if (op == opcode0_stca || op == opcode0_stcq)
            return 0;
        if (op == opcode0_stba || op == opcode0_stbq)
            return 0;
        if (op == opcode0_lcpr)
            return 0;
        if (op == opcode0_scpr)
            return 0;
    }

#if 0
    ???
    if eis multi-word
        variable = bits 0..17 (first 18 bits)
        int_inhibit = bit 28 // aka I
        mf1 = bits 29..36   // aka modification field
#endif

    ca_temp.more = 1;
    int mult = 0;

    while (ca_temp.more) {
        if (compute_addr(ip, &ca_temp) != 0) {
            if (ca_temp.more) log_msg(NOTIFY_MSG, moi, "Not-Final (not-incomplete) CA: 0%0o\n", TPR.CA);
            // return 1;
        }
        if (ca_temp.more)
            mult = 1;
        if (ca_temp.more)
            if(opt_debug>0)
                log_msg(DEBUG_MSG, moi, "Post CA: Continuing indirect fetches\n");
        if (ca_temp.more)
            mult = 1;
    }
    if (mult) {
            if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Final CA: 0%0o\n", TPR.CA);
    }

    addr_mode = get_addr_mode();    // may have changed

    if (addr_mode == BAR_mode) {
        if (addr_mode == BAR_mode && ptr_reg_flag == 0) {
            // Todo: Add CA to BAR.base; add in PR; check CA vs bound
        }
        // FIXME: Section 4 says make sure CA cycle handled AR reg mode and
        // constants
        if (TPR.is_value) {
            log_msg(WARN_MSG, moi, "BAR mode not fully implemented.\n");
            return 0;
        } else {
            log_msg(ERR_MSG, moi, "BAR mode not implemented.\n");
            cancel_run(STOP_BUG);
            return 1;
        }
    }

    if (addr_mode == ABSOLUTE_mode && ptr_reg_flag == 0) {
        // TPR.CA is the 18-bit absolute main memory addr
        if (orig_mode != addr_mode)
            log_msg(DEBUG_MSG, moi, "finished\n");
        return 0;
    }

    // APPEND mode handled by fetch_word() etc

    if (orig_mode != addr_mode)
        log_msg(DEBUG_MSG, moi, "finished\n");
    return 0;
}

//=============================================================================

/*
 * compute_addr()
 *
 * Perform a "CA" cycle as per figure 6-2 of AL39.
 *
 * Generate an 18-bit computed address (in TPR.CA) as specified in section 6
 * of AL39.
 * In our version, this may include replacing TPR.CA with a 36 bit constant or
 * other value if an appropriate modifier (e.g., du) is present.
 *
 */

static int compute_addr(const instr_t *ip, ca_temp_t *ca_tempp)
{
    ca_tempp->more = 0;

    // FIXME: Need to do ESN special handling if loop is continued

    // the and is a hint to the compiler for the following switch...
    enum atag_tm tm = (cu.IR.mods.single.tag >> 4) & 03;

    uint td = cu.IR.mods.single.tag & 017;

    ca_tempp->special = tm;

    if (cu.rpt || cu.rd) {
        // Check some requirements, but don't generate a fault (AL39 doesn't
        // say whether or not we should fault)
        if (tm != atag_r && tm != atag_ri)
            log_msg(ERR_MSG, "APU", "Repeated instructions must use register or register-indirect address modes.\n");
        else
            if (td == 0)
                log_msg(ERR_MSG, "APU", "Repeated instructions should not use X[0].\n");
    }

    switch(tm) {
        case atag_r: {
        // Tm=0 -- register (r)
            if (td != 0)
                reg_mod(td, sign18(TPR.CA));
            if (cu.rpt || cu.rd) {
                int n = td & 07;
                reg_X[n] = TPR.CA;
            }
            return 0;
        }
        case atag_ri: {
        // Tm=1 -- register then indirect (ri)
            if (td == 3 || td == 7) {
                // ",du" or ",dl"
                log_msg(WARN_MSG, "APU", "RI with td==0%o is illegal.\n", td);
                fault_gen(illproc_fault);   // need illmod sub-category
                return 1;
            }
            uint ca_orig = TPR.CA;
            reg_mod(td, sign18(TPR.CA));
            if(opt_debug)
                log_msg(DEBUG_MSG, "APU",
                    "RI: pre-fetch:  TPR.CA=0%o <==  TPR.CA=%o + 0%o\n",
                    TPR.CA, ca_orig, TPR.CA - ca_orig);
            t_uint64 word;
            if (cu.rpt || cu.rd) {
                int n = td & 07;
                reg_X[n] = TPR.CA;
                if (opt_debug>0 || cu.rd) {
                    log_msg((cu.rpt) ? DEBUG_MSG : INFO_MSG, "APU",
                        "RI for repeated instr: Setting X[%d] to CA 0%o(%d).\n",
                        n, reg_X[n], reg_X[n]);
                    log_msg((cu.rpt) ? DEBUG_MSG : INFO_MSG, "APU",
                        "RI for repeated instr: Not doing address appending on CA.\n");
                }
                if (! cu.repeat_first) {
                    log_msg(INFO_MSG, "APU", "Doing extra indir fetch(es) for repeated instr.  Reading %#o\n", TPR.CA);
                    cancel_run(STOP_WARN);
                }
                if (fetch_abs_word(TPR.CA, &word) != 0)
                    return 1;
            } else
                if (addr_append(&word) != 0)
                    return 1;
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU",
                "RI: fetch:  word at TPR.CA=0%o is 0%llo\n", TPR.CA, word);
            if (cu.rpt || cu.rd) {
                // ignore tag and don't allow more indirection
                // cu.IR.mods.single.tag = 0;   // good idea?  bad idea?
                return 0;
            }
            cu.IR.mods.single.tag = word & MASKBITS(6);
            if (TPR.CA % 2 == 0 && (cu.IR.mods.single.tag == 041 || cu.IR.mods.single.tag == 043)) {
                    int ret = do_its_itp(ip, ca_tempp, word);
                    if(opt_debug>0) log_msg(DEBUG_MSG, "APU",
                        "RI: post its/itp: TPR.CA=0%o, tag=0%o\n",
                        TPR.CA, cu.IR.mods.single.tag);
                    if (ret != 0) {
                        if (cu.IR.mods.single.tag != 0) {
                            log_msg(WARN_MSG, "APU",
                                "RI: post its/itp: canceling remaining APU cycles.\n");
                            cancel_run(STOP_WARN);
                        }
                        return ret;
                    }
            } else {
                TPR.CA = word >> 18;
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU",
                    "RI: post-fetch: TPR.CA=0%o, tag=0%o\n",
                    TPR.CA, cu.IR.mods.single.tag);
            }
            // break;   // Continue a new CA cycle
            ca_tempp->more = 1;     // Continue a new CA cycle
            // FIXME: flowchart says start CA, but we do ESN
            return 0;
        }
        case atag_it: {
        // Tm=2 -- indirect then tally (it)
            // FIXME: see "it" flowchart for looping (Td={15,17}
            switch(td) {
                case 0:
                    log_msg(WARN_MSG, "APU", "IT with Td zero not valid in instr word.\n");
                    fault_gen(f1_fault);    // This mode not ok in instr word
                    break;
                case 6:     
                    // This mode not ok in an instr word
                    log_msg(WARN_MSG, "APU", "IT with Td six is a fault.\n");
                    fault_gen(fault_tag_2_fault);
                    break;
                case 014: {
                    t_uint64 iword;
                    int ret;
                    int iloc = TPR.CA;
                    if ((ret = addr_append(&iword)) == 0) {
                        int addr = getbits36(iword, 0, 18);
                        int tally = getbits36(iword, 18, 12);
                        int tag = getbits36(iword, 30, 6);
                        ++tally;
                        tally &= MASKBITS(12);  // wrap from 4095 to zero
                        // FIXME: do we need to fault?
                        IR.tally_runout = (tally == 0);
                        if (IR.tally_runout)
                            log_msg(NOTIFY_MSG, "APU", "IT(di): tally runout\n");
                        --addr;
                        addr &= MASK18; // wrap from zero to 2^18-1
                        iword = setbits36(iword, 0, 18, addr);
                        iword = setbits36(iword, 18, 12, tally);
                        TPR.CA = addr;
                        if (opt_debug) {
                            // give context for appending msgs
                            log_msg(DEBUG_MSG, "APU",
                                "IT(di): addr now 0%o, tally 0%o\n",
                                addr, tally);
                        }
                        ret = store_word(iloc, iword);
                    }
                    return ret;
                }
                // case 015: more=1 depending upon tag
                case 016: {
                    // mode "id" -- increment addr and decrement tally
                    t_uint64 iword;
                    int ret;
                    int iloc = TPR.CA;
                    if ((ret = addr_append(&iword)) == 0) {
                        int addr = getbits36(iword, 0, 18);
                        int tally = getbits36(iword, 18, 12);
                        int tag = getbits36(iword, 30, 6);
                        TPR.CA = addr;
                        --tally;
                        tally &= MASKBITS(12);  // wrap from zero to 4095
                        IR.tally_runout = (tally == 0); // NOTE: The Bpush macro usage in bootload_0 implies that we should *not* fault
                        ++addr;
                        addr &= MASK18; // wrap from 2^18-1 to zero
                        iword = setbits36(iword, 0, 18, addr);
                        iword = setbits36(iword, 18, 12, tally);
                        if (opt_debug) {
                            // give context for appending msgs
                            log_msg(DEBUG_MSG, "APU", "IT(id): addr now 0%o, tally 0%o\n", addr, tally);
                        }
                        ret = store_word(iloc, iword);
                    }
                    return ret;
                }
                default:
                    log_msg(ERR_MSG, "APU",
                        "IT with Td 0%o not implemented.\n", td);
                    cancel_run(STOP_BUG);
                    return 1;
            }
            break;
        }
        case atag_ir: {
        // Tm=3 -- indirect then register (ir)
            int nloops = 0;
            while(tm == atag_ir || tm == atag_ri) {
                if (++nloops > 1)
                    log_msg(NOTIFY_MSG, "APU::IR", "loop # %d\n", nloops);
                if (tm == atag_ir)
                    cu.CT_HOLD = td;
                // FIXME: Maybe handle special tag (41 itp, 43 its).  Or post
                // handle?
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR",
                    "pre-fetch: Td=0%o, TPR.CA=0%o\n", td, TPR.CA);
                t_uint64 word;
                if (addr_append(&word) != 0)
                    return 1;
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR",
                    "fetched:  word at TPR.CA=0%o is 0%llo:\n",
                    TPR.CA, word);
                cu.IR.mods.single.tag = word & MASKBITS(6);
                if (TPR.CA % 2 == 0 &&
                    (cu.IR.mods.single.tag == 041 || cu.IR.mods.single.tag == 043))
                {
                        // TODO: this code should probably be moved into our caller to match AL39 Fig 6-10
                        int ret = do_its_itp(ip, ca_tempp, word);
                        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR",
                            "post its/itp: TPR.CA=0%o, tag=0%o\n",
                            TPR.CA, cu.IR.mods.single.tag);
                        if (ret != 0) {
                            if (cu.IR.mods.single.tag != 0) {
                                log_msg(WARN_MSG, "APU",
                                    "IR: post its/itp: canceling remaining APU cycles.\n");
                                cancel_run(STOP_WARN);
                            }
                            return ret;
                        }
                } else {
                    TPR.CA = word >> 18;
                    tm = (cu.IR.mods.single.tag >> 4) & 03;
                    td = cu.IR.mods.single.tag & 017;
                    if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR",
                        "post-fetch: TPR.CA=0%o, tag=0%o, new tm=0%o; td = %o\n",
                        TPR.CA, cu.IR.mods.single.tag, tm, td);
                    if (td == 0) {
                        // FIXME: Disallow a reg_mod() with td equal to
                        // NULL (AL39)
                        // Disallow always or maybe ok for ir?
                        log_msg(ERR_MSG, "APU::IR", "Found td==0 (for tm=0%o)\n", tm);
                        cancel_run(STOP_WARN);
                    }
                    switch(tm) {
                        case atag_ri:
                            log_msg(WARN_MSG, "APU::IR",
                                "IR followed by RI.  Not tested\n");
                            reg_mod(td, sign18(TPR.CA));
                            break;      // continue looping
                        case atag_r:
                            reg_mod(cu.CT_HOLD, sign18(TPR.CA));
                            return 0;
                        case atag_it:
                            //reg_mod(td, sign18(TPR.CA));
                            log_msg(ERR_MSG, "APU::IR", "Need to run normal IT algorithm, ignoring fault 1.\n"); // actually cannot have fault 1 if disallow td=0 above
                            cancel_run(STOP_BUG);
                            return 0;
                        case atag_ir:
                            log_msg(WARN_MSG, "APU::IR", "IR followed by IR, continuing to loop.  Not tested\n");
                            cu.CT_HOLD = cu.IR.mods.single.tag & MASKBITS(4);
                            break;      // keep looping
                    }
                }
                //log_msg(WARN_MSG, "APU::IR", "Finished, but unverified.\n");
                //cancel_run(STOP_WARN);
                log_msg(DEBUG_MSG, "APU::IR", "Finished.\n");
                return 0;
            }
        }
    }

    // FIXME: Need to do ESN special handling if loop is continued

    return 0;
}


//=============================================================================

/*
 * chars_to_words()
 *
 * Called only by register_mod().  Called after intermediate calculations
 * that might yield a bit offset of more than 35 bits.
 */

static void chars_to_words(int n, uint nbits, uint *offp, uint *bitnop)
{
    const char *moi = "APU::reg-mod";
    if (n == 0)
        return;

    uint o = *offp;
    uint b = *bitnop;

    int chars_per_word = 36 / nbits;
    *offp +=  n / chars_per_word;


    uint nchars = n % chars_per_word;
    if (nchars != 0) {
        log_msg(DEBUG_MSG, moi, "Reg mod for %d %d-bit chars: Value isn't evenly divisible by %d; result will contain a bit offset.\n", n, nbits, chars_per_word);
        *bitnop += nchars * nbits;
        if (*bitnop >= 36) {
            log_msg(DEBUG_MSG, moi, "Reg mod for %d %d-bit chars:  Dest is over 36 bits.  Wrapping.\n", n, nbits);
            *offp += *bitnop / 36;
            *bitnop = *bitnop % 36;
            log_msg(DEBUG_MSG, moi, "Wrap is +%d words and +%d bits.\n",
                (nchars * nbits) / 36, (nchars * nbits) % 36);
            log_msg(DEBUG_MSG, moi, "Dest changes from %06o+%02db to %06o+%02db.\n", 
                o, b, *offp, *bitnop);
        }
    } else {
        if (nbits != 36) {
            log_msg(DEBUG_MSG, "APU", "Reg mod for %d-bit data: Value %d/%d yields %d words\n", nbits, n, chars_per_word, n/chars_per_word);
        }
    }
}


//=============================================================================

/*
 * reg_mod()
 *
 * Perform "Td" register modification for 36-bit full words.
 * Results saved to TPR.{is_value, CA, value}.
 *
 * See register_mod() for the same functionality but with partial word arguments.
 *
 */

void reg_mod(uint td, int off)
{
    char *moi = "APU::reg-mod";

    uint bitno = 0;
    register_mod(td, off, &bitno, 36);
    if (bitno != 0) {
        log_msg(ERR_MSG, moi, "Unable to handle result with bit offset.\n");
        cancel_run(STOP_BUG);
    }
}

//=============================================================================

/*
 * register_mod()
 *
 * Performs "td" register modification.
 * Results saved to TPR.{is_value, CA, value}.
 */

static void register_mod(uint td, uint off, uint *bitnop, int nbits)
{
    // FIXME/TODO: addr_mod() isn't our only caller...
    // ; TPR.is_value = 0; // FIXME: Use "direct operand flag" instead

    char *moi = "APU::reg-mod";
    switch(td) {
        case 0:
            break;  // no mod
        case 1: // ,au
            chars_to_words(sign18(getbits36(reg_A, 0, 18)), nbits, &off, bitnop);
            TPR.CA = off;
            TPR.CA &= MASK18;
            break;
        case 2: // ,qu
            chars_to_words(sign18(getbits36(reg_Q, 0, 18)), nbits, &off, bitnop);
            TPR.CA = off;
            TPR.CA &= MASK18;
            break;
        case 3: // ,du
            TPR.is_value = td;  // FIXME: Use "direct operand flag" instead
            TPR.value = ((t_uint64) TPR.CA) << 18;
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Mod du: Value from offset 0%o is 0%llo\n", TPR.CA, TPR.value);
            break;
        case 4: // PPR.IC
            TPR.CA = off + PPR.IC;  // FIXME: IC assumed to be unsigned
            TPR.CA &= MASK18;
            break;
        case 5:
            {
            uint orig_off = off;
            chars_to_words(sign18(getbits36(reg_A, 18, 18)), nbits, &off, bitnop);
            TPR.CA = off;
            TPR.CA &= MASK18;
            if (opt_debug) {
                uint a = getbits36(reg_A, 18, 18);
                log_msg(DEBUG_MSG, "APU", "Tm=REG,Td=%02o: offset %06o(%d) + A=(36)%#llo=>(18)%06o(%+d decimal) ==> 0%o=>0%o(%+d)\n",
                    td, orig_off, orig_off, reg_A, a, sign18(a), TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
            }
            }
            break;
        case 6:
            chars_to_words(sign18(getbits36(reg_Q, 18, 18)), nbits, &off, bitnop);
            TPR.CA = off;
            TPR.CA &= MASK18;
            break;
        case 7: // ,dl
            TPR.is_value = td;  // FIXME: Use "direct operand flag" instead
            TPR.value = TPR.CA; // FIXME: Should we sign?
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Mod dl: Value from offset 0%o is 0%llo\n", TPR.CA, TPR.value);
            break;
        case 010:
        case 011:
        case 012:
        case 013:
        case 014:
        case 015:
        case 016:
        case 017:
            {
            uint orig = off;
            chars_to_words(sign18(reg_X[td&07]), nbits, &off, bitnop);
            TPR.CA = off;
            TPR.CA &= MASK18;
            if (opt_debug)
                log_msg(DEBUG_MSG, "APU", "Tm=REG,Td=%02o: offset 0%o + X[%d]=0%o(%+d decimal)==>0%o(+%d) yields 0%o (%+d decimal)\n",
                    td, orig, td&7, reg_X[td&7], reg_X[td&7], sign18(reg_X[td&7]), sign18(reg_X[td&7]), TPR.CA, TPR.CA);
            break;
            }
    }
}

//=============================================================================

// static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp)

/*
 * do_its_itp()
 *
 * Implements the portion of AL39 figure 6-10 that is below the "CA CYCLE" box
 *
 */

static int do_its_itp(const instr_t* ip, ca_temp_t *ca_tempp, t_uint64 word01)
{
    // FIXME: Caller should almost certainly provide a word02
    // Just did an "ir" or "ri" addr modification
    if (cu.IR.mods.single.tag == 041) {
        // itp
        t_uint64 word1, word2;
        // FIXME: are we supposed to fetch?
        int ret = fetch_pair(TPR.CA, &word1, &word2);   // bug: refetching word1
        if (ret != 0)
            return ret;
        if (word01 != word1) {
            log_msg(WARN_MSG, "APU:ITS/ITP", "Refetched word at %#o has changed.   Was %012llo, now %012llo.\n", word01, word1);
            cancel_run(STOP_BUG);
        }
        set_addr_mode(APPEND_mode);
        uint n = getbits36(word1, 0, 3);
        TPR.TSR = AR_PR[n].PR.snr;
        SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
        if (SDWp == NULL) {
            log_msg(WARN_MSG, "APU:ITS/ITP", "Segment missing.\n");
            cancel_run(STOP_BUG);
            return 1;
        }
        uint sdw_r1 = SDWp->r1;
        TPR.TRR = max3(AR_PR[n].PR.rnr, sdw_r1, TPR.TRR);
        TPR.TBR = getbits36(word2, 21, 6);
        cu.IR.mods.single.tag = word2 & MASKBITS(6);
        uint i_mod_tm = cu.IR.mods.single.tag >> 4;
        uint r;
        if (ca_tempp->special == atag_ir) {
            log_msg(DEBUG_MSG, "APU", "ITP: temp special is IR; Will use r from cu.CT_HOLD\n");
            r = cu.CT_HOLD;
        } else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
            uint i_mod_td = cu.IR.mods.single.tag & MASKBITS(4);
            // r = i_mod_td;
            r = 0;  // the tag will be used during the next cycle
        } else {
            log_msg(ERR_MSG, "APU", "ITP addr mod with undefined r-value (tm=0%o,new-tm=0%o)\n", ca_tempp->special, i_mod_tm);
            cancel_run(STOP_BUG);
            r = 0;
        }
        uint i_wordno = getbits36(word2, 0, 18);
        // TPR.CA = AR_PR[n].wordno + i_wordno + r;
        TPR.CA = (AR_PR[n].wordno + i_wordno) & MASK18;
        uint r_temp = TPR.CA;
        reg_mod(r, r_temp);
        r_temp = TPR.CA - r_temp;
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITP: CA = PR[%d].wordno=%#o + wordno=%#o + r=%#o => %#o\n", n, AR_PR[n].wordno, i_wordno, r_temp, TPR.CA);
        ca_tempp->more = 1;
        log_msg(DEBUG_MSG, "APU", "ITP done\n");
        //cancel_run(STOP_WARN);
        return 0;
    } else if (cu.IR.mods.single.tag == 043) {
        // its
        t_uint64 word1, word2;
        // FIXME: are we supposed to fetch?
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITS: CA initially 0%o\n", TPR.CA);
        int ret = fetch_pair(TPR.CA, &word1, &word2);   // bug: refetching word1
        if (ret != 0)
            return ret;
        if (word01 != word1) {
            log_msg(WARN_MSG, "APU:ITS/ITP", "Refetched word at %#o has changed.   Was %012llo, now %012llo.\n", word01, word1);
            cancel_run(STOP_BUG);
        }
        set_addr_mode(APPEND_mode);
        TPR.TSR =  getbits36(word1, 3, 15);
        uint its_rn = getbits36(word1, 18, 3);
        SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
        uint sdw_r1;
        if (SDWp == NULL) {
            // Note that constructing or copying "null" pointers will trigger this
            // condition.   We don't fault unless the instruction later tries to
            // dereference the "bad" pointer.
            if (! cpu.apu_state.fhld) {
                log_msg(WARN_MSG, "APU:ITS", "Segment %#o is missing.\n", TPR.TSR);
                cancel_run(STOP_BUG);
                ret = 1;
            } else
                log_msg(INFO_MSG, "APU:ITS", "Segment %#o is missing.\n", TPR.TSR);
            sdw_r1 = 7;     // FIXME/TODO: what ring should we use when there is none?  Currently using worst case -- 7
        } else
            sdw_r1 = SDWp->r1;
        TPR.TRR = max3(its_rn, sdw_r1, TPR.TRR);
        TPR.TBR = getbits36(word2, 21, 6);
        if (TPR.TBR > 35) {
            // NOTE: AL-39 says appending unit doesn't detect bad bit offsets but that decimal unit does.
            log_msg(ERR_MSG, "APU:ITS", "ITS specifies a bit offset of %d bits\n", TPR.TBR);
            cancel_run(STOP_WARN);
        }
        cu.IR.mods.single.tag = word2 & MASKBITS(6);

        uint i_mod_tm = cu.IR.mods.single.tag >> 4;
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITS: TPR.TSR = 0%o, rn=0%o, sdw.r1=0%o, TPR.TRR=0%o, TPR.TBR=0%o, tag=0%o(tm=0%o)\n",
            TPR.TSR, its_rn, sdw_r1, TPR.TRR, TPR.TBR, cu.IR.mods.single.tag, i_mod_tm);
        uint r; // type of "td" register mod to apply
        if (ca_tempp->special == atag_ir) {
            log_msg(DEBUG_MSG, "APU", "ITS: temp special is IR; Will use r from cu.CT_HOLD\n");
            r = cu.CT_HOLD;
        } else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
            // uint i_mod_td = cu.IR.mods.single.tag & MASKBITS(4);
            // r = i_mod_td;
            r = 0;  // do nothing now; the tag will be used during the next cycle
            log_msg(DEBUG_MSG, "APU", "ITS: temp special is RI; temp tag is r or ri; Will use r from special tag's td (next cycle)\n");
        } else {
            log_msg(ERR_MSG, "APU", "ITS addr mod with undefined r-value (tm=0%o,new-tm=0%o)\n", ca_tempp->special, i_mod_tm);
            cancel_run(STOP_BUG);
            r = 0;  // do nothing now; the tag will be used during the next cycle
        }
        uint i_wordno = getbits36(word2, 0, 18);
        TPR.CA = i_wordno;
        uint r_temp = TPR.CA;
        reg_mod(r, r_temp);
        r_temp = TPR.CA - r_temp;
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITS: CA = wordno=0%o + r=0%o => 0%o\n", i_wordno, r_temp, TPR.CA);
        ca_tempp->more = 1;
        return ret;
    }

#if 0
    // If we need an indirect word, we should return to the top of the ESN flow (AL39, figure 6-8)
    if (ca_tempp->more)
        return 0;

    if (ip->opcode == (opcode0_rtcd << 1)) {
        log_msg(ERR_MSG, "APU", "RTCD operand not implemented.\n");
        cancel_run(STOP_BUG);
    } else if (ip->opcode == (opcode0_call6 << 1) || is_transfer_op(ip->opcode)) {
        log_msg(ERR_MSG, "APU", "Call6 and transfer operands not implemented.\n");
        cancel_run(STOP_BUG);
    } else {
        // FIXME: What does the question "Appending unit data movement?" mean?
    }
#endif
    return 0;
}

//=============================================================================

static int addr_append(t_uint64 *wordp)
{
    // Implements AL39, figure 5-4
    // NOTE: ri mode is expecting a fetch
    return fetch_appended(TPR.CA, wordp);
}

//=============================================================================

/*
 * fetch_appended()
 *
 * Fetch a word at the given offset in the current segment (if possible).
 *
 * Implements AL39, figure 5-4
 *
 * Note that we allow an arbitrary offset not just TPR.CA.   This is to support
 * instruction fetches.
 *
 * FIXME: Need to handle y-pairs -- fixed?
 *
 * In BAR mode, caller is expected to have already added the BAR.base and
 * checked the BAR.bounds
 *
 * Returns non-zero if a fault in groups 1-6 detected
 */

int fetch_appended(uint offset, t_uint64 *wordp)
{
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode == ABSOLUTE_mode)
        return fetch_abs_word(offset, wordp);
#if 0
    if (addr_mode == BAR_mode) {
        log_msg(WARN_MSG, "APU::fetch_append", "APU not intended for BAR mode\n");
        offset += BAR.base << 9;
        cancel_run(STOP_WARN);
        return fetch_abs_word(offset, wordp);
    }
    if (addr_mode != APPEND_mode)
#else
    if (addr_mode != APPEND_mode && addr_mode != BAR_mode)
#endif
    {
        // impossible
        log_msg(ERR_MSG, "APU::append", "Unknown mode\n");
        cancel_run(STOP_BUG);
        return fetch_abs_word(offset, wordp);
    }

    t_uint64 simh_addr = addr_emul_to_simh(addr_mode, TPR.TSR, offset);
    if (sim_brk_summ) {
        if (sim_brk_test(simh_addr, SWMASK('M'))) {
            log_msg(WARN_MSG, "APU", "Memory Breakpoint on read.\n");
            cancel_run(STOP_IBKPT);
        }
    }

    uint addr;
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, 0, &addr, &minaddr, &maxaddr);
    if (ret == 0) {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::fetch_append", "Using addr 0%o\n", addr);
        ret = fetch_abs_word(addr, wordp);
        if (ret == 0) {
            if (cpu.cycle != FETCH_cycle && sim_brk_test (simh_addr, SWMASK ('D'))) {
                extern UNIT cpu_unit;   // FIXME
                out_sym(0, simh_addr, wordp, &cpu_unit, SWMASK('M') | SWMASK('A'));
            }
        }
    } else {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::fetch_append", "page-in faulted\n");
    }
    return ret;
}

//=============================================================================

int store_appended(uint offset, t_uint64 word)
{
    // Store a word at the given offset in the current segment (if possible).
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != APPEND_mode && addr_mode != BAR_mode) {
        // impossible
        log_msg(ERR_MSG, "APU::store-append", "Not APPEND mode\n");
        cancel_run(STOP_BUG);
    }

    t_uint64 simh_addr = addr_emul_to_simh(addr_mode, TPR.TSR, offset);
    if (sim_brk_summ) {
        if (sim_brk_test(simh_addr, SWMASK('W') | SWMASK('M'))) {
            log_msg(WARN_MSG, "APU", "Memory Breakpoint on write.\n");
            cancel_run(STOP_IBKPT);
        }
    }

    uint addr;
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, 0, &addr, &minaddr, &maxaddr);
    if (ret == 0) {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::store-append", "Using addr 0%o\n", addr);
        ret = store_abs_word(addr, word);
        if (ret == 0)
            if (sim_brk_test (simh_addr, SWMASK ('D'))) {
                extern UNIT cpu_unit;   // FIXME
                out_sym(1, simh_addr, &word, &cpu_unit, SWMASK('M') | SWMASK('A'));
            }
    } else
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::store-append", "page-in faulted\n");
    return ret;
}

//=============================================================================

int cmd_dump_vm(int32 arg, char *buf)
{
    // Dump VM info -- display the cache registers & descriptor table

    restore_from_simh();    // in case DSBR or other vars were updated

    out_msg("DSBR: addr=0%08o, bound=0%o(%d), unpaged=%c\n",
        cpup->DSBR.addr, cpup->DSBR.bound, cpup->DSBR.bound, cpup->DSBR.u ? 'Y' : 'N');
    if (cu.SD_ON)
        out_msg("SDWAM is enabled.\n");
    else
        out_msg("SDWAM is NOT enabled.\n");
    if (cu.PT_ON)
        out_msg("PTWAM is enabled.\n");
    else
        out_msg("PTWAM is NOT enabled.\n");
    if (cpup->DSBR.u)
        out_msg("DSBR: SDW for segment 'segno' is at %08o + 2 * segno.\n", cpup->DSBR.addr);
    else {
        out_msg("DSBR: Offset (y1) in page table for segment 'segno' is at (2 * segno)%%%d;\n", page_size);
        out_msg("DSBR: PTW for segment 'segno' is at 0%08o + (2 * segno - y1)/%d.\n", cpup->DSBR.addr, page_size);
        out_msg("DSBR: SDW for segment 'segno' is at PTW.addr<<6 + y1.\n");
    }
    for (int i = 0; i < ARRAY_SIZE(cpup->SDWAM); ++i) {
        out_msg("SDWAM[%d]: ptr(segno)=0%05o, is-full=%c, use=0%02o(%02d)\n",
            i, cpup->SDWAM[i].assoc.ptr, cpup->SDWAM[i].assoc.is_full ? 'Y' : 'N',
            cpup->SDWAM[i].assoc.use, cpup->SDWAM[i].assoc.use);
        SDW_t *sdwp = &cpup->SDWAM[i].sdw;
        out_msg("\tSDW for seg %#o: addr = 0%08o, r1=%o r2=%o r3=%o, f=%c, fc=0%o.\n",
            cpup->SDWAM[i].assoc.ptr, sdwp->addr, sdwp->r1, sdwp->r2, sdwp->r3, sdwp->f ? 'Y' : 'N', sdwp->fc);
        out_msg("\tbound = 0%05o(%d) => %06o(%u)\n",
            sdwp->bound, sdwp->bound, sdwp->bound * 16 + 16, sdwp->bound * 16 + 16);
        out_msg("\tr=%c e=%c w=%c, priv=%c, unpaged=%c, g=%c, c=%c, cl=%05o\n",
            sdwp->r ? 'Y' : 'N', sdwp->e ? 'Y' : 'N', sdwp->w ? 'Y' : 'N',
            sdwp->priv ? 'Y' : 'N', sdwp->u ? 'Y' : 'N', sdwp->g ? 'Y' : 'N',
            sdwp->c ? 'Y' : 'N', sdwp->cl);
    }
    out_msg("\n");
    dump_descriptor_table();
    return 0;
}

//=============================================================================

int apu_show_vm(FILE *st, UNIT *uptr, int val, void *desc)
{
    // FIXME: use FILE *st
    return cmd_dump_vm(0, NULL);
}

//=============================================================================

SDW_t* get_sdw()
{
    // Get SDW for TPR.TSR

    uint addr;
    SDWAM_t* SDWp = page_in_sdw();
    if (SDWp == NULL)
        return NULL;
    return &SDWp->sdw;
}


//=============================================================================

/*
 * addr_any_to_abs()
 *
 * Return the 24-bit absolute address
 *
 */

int addr_any_to_abs(uint *addrp, addr_modes_t mode, int segno, int offset)
{
    if (mode == ABSOLUTE_mode) {
        *addrp = offset;
        return 0;
    }
    if (mode == BAR_mode) {
        log_msg(WARN_MSG, "APU::addr_any_to_abs", "BAR mode not supported\n");
        return -1;
    }
    return convert_address(addrp, segno, offset, 0);
}

//=============================================================================

/*
 * convert_address()
 *
 * Return the 24-bit absolute address for an offset in the specified segment
 * Optionally disables (but reports) faults.
 * Capability to disable faults is intended for use by debug/display code.
 * Attempts to page-in the data.
 *
 */

int convert_address(uint* addrp, int seg, int offset, int fault)
{
    int saved_no_fault = fault_gen_no_fault;

    uint saved_tsr = -1;
    if (seg != -1) {
        saved_tsr = TPR.TSR;
        TPR.TSR = seg;
    }

    if (! fault)
        fault_gen_no_fault = 1;

    int ret = get_seg_addr(offset, 0, addrp);
    if (saved_tsr != -1)
        TPR.TSR = saved_tsr;
    if (! fault) {
        fault_gen_no_fault = saved_no_fault;
    }
    return ret;
}



//int fetch_seg(int segno, uint offset, t_uint64 *wordp)
//{
//static int page_in(uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp)
//}


//=============================================================================

/*
 * get_seg_addr()
 *
 * Return the 24-bit absolute address for an offset in the current segment
 * Attempts to page-in the data.
 *
 * FIXME: causes faults, but see fault_gen_no_fault global.  Now that null
 * pointers (-1, -2, etc) are handled differently, the faulting is probably
 * no longer a bug...
 *
 */

int get_seg_addr(uint offset, uint perm_mode, uint *addrp)
{
    if (addrp == NULL)
        return -1;
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != APPEND_mode && addr_mode != BAR_mode) {
        log_msg(WARN_MSG, "APU::get_seg_addr", "Not APPEND mode\n");
        return -1;
    }
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, perm_mode, addrp, &minaddr, &maxaddr);
    if (ret)
        log_msg(WARN_MSG, "APU::get_seg_addr", "page-in faulted\n");
    return ret;
}

//=============================================================================

/*
 * page_in()
 *
 * Implements AL39, figure 5-4
 *
 * Note that we allow an arbitrary offset not just TPR.CA.   This is to support
 * instruction fetches.
 * Manipulates the SDWAM and PTWAM to setup for access to the page containing
 * the desired address.
 * Note that directed faults will occur if a SDW is so flagged (e.g. when the
 * segment is on disk).
 *
 * FIXME/BUGS/Caveats:
 *     perm_mode is currently not checked by page_in_page().
 *
 * Results:
 *     addrp -- Resulting 24bit physical memory address
 *     minaddrp -- lowest 24bit physical address of the page
 *     maxaddrp -- highest 24bit physical address of the page
 *
 * Returns non-zero if a fault in groups 1-6 detected
 *
 */

static int page_in(uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp)
{
    const char *moi = "APU::append::page-in";
    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...
    if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting for Segno=0%o, offset=0%o.  (PPR.PSR is 0%o)\n", segno, offset, PPR.PSR);

    // ERROR: Validate that all PTWAM & SDWAM entries are always "full" and that use fields are always sane
    SDWAM_t* SDWp = page_in_sdw();
    if (SDWp == NULL) {
        if (cpu.apu_state.fhld) {
            // Do we need to check cu.word1flags.oosb and other flags to know what kind of fault to gen?
            fault_gen(acc_viol_fault);
            cpu.apu_state.fhld = 0;
        }
        log_msg(WARN_MSG, moi, "SDW not loaded\n");
        return 1;
    }
    int ret = page_in_page(SDWp, offset, perm_mode, addrp, minaddrp, maxaddrp);
    if (ret != 0) {
        log_msg(NOTIFY_MSG, moi, "page_in_page returned non zero.  Segno %#o, offset %#o(%d)\n", segno, offset, offset);
    }
    return ret;
}


//=============================================================================

/*
 * dump_descriptor_table()
 *
 * Hacked copy of page_in_sdw() for debug display
 *
 */

static void dump_descriptor_table()
{
    out_msg("Dump of descriptor table:\n");
    out_msg("\n");
    for (uint segno = 0; segno * 2 <= 16 * (cpup->DSBR.bound + 1); ++segno) {
        if (segno * 2 == 16 * (cpup->DSBR.bound + 1)) {
            out_msg("End of descriptor table.\n");
            break;
        }
        t_uint64 sdw_word0, sdw_word1;
        uint sdw_addr;
        if (cpup->DSBR.u) {
            // Descriptor table is unpaged
            if (segno * 2 >= 16 * (cpup->DSBR.bound + 1))
                break;
            sdw_addr = cpup->DSBR.addr + 2 * segno;
            if (fetch_abs_pair(cpup->DSBR.addr + 2 * segno, &sdw_word0, &sdw_word1) != 0)
                break;
        } else {
            // Descriptor table is paged
            if (segno * 2 >= 16 * (cpup->DSBR.bound + 1))
                break;
            // First, the DSPTW fetch cycle (PTWAM doesn't cache the DS?)
            uint y1 = (2 * segno) % page_size;          // offset within page table
            uint x1 = (2 * segno - y1) / page_size;     // offset within DS page
            PTW_t DSPTW;
            t_uint64 word;
            if (fetch_abs_word(cpup->DSBR.addr + x1, &word) != 0)   // We assume DS is 1024 words (bound=077->16*64=1024)
                break;
            decode_PTW(word, &DSPTW);
            if (DSPTW.f == 0) {
                out_msg("Segment %#o: lookup would result in DSPTW directed fault\n", segno);
                continue;
            }
            if (! DSPTW.u) {
                // MDSPTW cycle
                DSPTW.u = 1;
                //if (set_PTW_used(cpup->DSBR.addr + x1) != 0) {
                    // impossible -- we just read this absolute addresed word
                    //return NULL;
                //}
            }
            // PSDW cycle (Step 5 for case when Descriptor Segment is paged)
            // log_msg(DEBUG_MSG, moi, "Fetching SDW from 0%o<<6+0%o => 0%o\n", DSPTW.addr, y1, (DSPTW.addr<<6) + y1);
            sdw_addr = (DSPTW.addr<<6) + y1;
            if (fetch_abs_pair((DSPTW.addr<<6) + y1, &sdw_word0, &sdw_word1) != 0)
                continue;
        }
        SDW_t sdw;
        decode_SDW(sdw_word0, sdw_word1, &sdw);
        //out_msg("Descriptor entry at %06o for segment %03o: %s\n",
        //  sdw_addr, segno, sdw2text(&sdw));
        if (sdw.addr != 0) {
            out_msg("Descriptor entry at %06o for segment %03o:\n",
                sdw_addr, segno);
            out_msg("SDW: %s\n", sdw2text(&sdw));
            if (sdw.u) {
                uint bound = 16 * (sdw.bound + 1);
                out_msg("Segment %03o is unpaged and ranges from absolute %06o .. %06o\n",
                    segno, sdw.addr, sdw.addr + sdw.bound - 1);
            } else {
                /* TODO: loop through the page table */
                uint bound = 16 * (sdw.bound + 1);
                out_msg("Segment %03o is paged; %u words, page table at %06o\n",
                    segno, bound, sdw.addr);
                for (uint pageno = 0; pageno < bound / page_size; ++ pageno) {
                    t_uint64 word;
                    if (fetch_abs_word(sdw.addr + pageno, &word) != 0)
                        break;
                    PTW_t ptw;
                    decode_PTW(word, &ptw);
                    uint lo = (ptw.addr << 6);
                    out_msg("   page %d: %06o .. %06o\n", pageno, lo, lo + page_size - 1);
                }
            }
        }
        if (sdw.addr != 0) {
            get_seg_name(segno);
            out_msg("\n");
        }
    }
}

//=============================================================================

/*
 * page_in_sdw()
 *
 * Implements half of page_in() -- AL39, figure 5-4
 *
 * Note that some manipulations of NULL or other illegal pointer values are
 * allowed -- you can create such pointers but you can't later dereference
 * through them.
 *
 * Returns NULL if a fault in groups 1-6 detected
 *
 */

static SDWAM_t* page_in_sdw()
{
    const char* moi = "APU::append";
    //const char *moi = "APU::append::page-in::sdw";

    // FIXME/ERROR: Validate that all PTWAM & SDWAM entries are always "full"
    // and that use fields are always sane

    // TODO: Replace most of this with more efficient methods that match the
    // HW less well

    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...
    
    // FIXME: Need bounds checking at all cycles below except PSDW cycle

    // Check to see if SDW for segno is in SDWAM
    // Save results across invocations so that locality of reference avoids search
    static SDWAM_t *SDWp = 0;
    if (SDWp == 0) SDWp = cpup->SDWAM;  // FIXME: expose to reset? // BUG won't work for multiple CPUs
    int oldest_sdwam = -1;
    if (SDWp == NULL || SDWp->assoc.ptr != segno || ! SDWp->assoc.is_full) {    // todo: validate NULL as impossible
        SDWp = NULL;
        for (int i = 0; i < ARRAY_SIZE(cpup->SDWAM); ++i) {
            if (cpup->SDWAM[i].assoc.ptr == segno && cpup->SDWAM[i].assoc.is_full) {
                SDWp = cpup->SDWAM + i;
                // log_msg(DEBUG_MSG, moi, "Found SDW for segno 0%o in SDWAM[%d]\n", segno, i);
                break;
            }
            //if (! cpup->SDWAM[i].assoc.is_full) {
            //  log_msg(DEBUG_MSG, moi, "Found SDWAM[%d] is unused\n", i);
            //}
            if (cpup->SDWAM[i].assoc.use == 0)
                oldest_sdwam = i;
        }
    } else {
        // log_msg(DEBUG_MSG, moi, "SDW for segno 0%o is the MRU -- in SDWAM[%d]\n", segno, SDWp-SDWAM);
    }

    if (SDWp != NULL) {
        // SDW is in SDWAM; it moves to the end of the LRU queue
        // log_msg(DEBUG_MSG, moi, "SDW is in SDWAM[%d].\n", SDWp - cpup->SDWAM);
        if (SDWp->assoc.use != 15) {
            for (int i = 0; i < ARRAY_SIZE(cpup->SDWAM); ++i) {
                if (cpup->SDWAM[i].assoc.use > SDWp->assoc.use)
                    -- cpup->SDWAM[i].assoc.use;
            }
            SDWp->assoc.use = 15;
        }
        return SDWp;
    }

    // Fetch SDW and place into SDWAM
    if(opt_debug>0) log_msg(DEBUG_MSG, moi, "SDW for segno 0%o is not in SDWAM.  DSBR addr is 0%o\n", segno, cpup->DSBR.addr);
    t_uint64 sdw_word0, sdw_word1;
    if (cpup->DSBR.u) {
        // Descriptor table is unpaged
        // Do a NDSW cycle
        if (segno * 2 >= 16 * (cpup->DSBR.bound + 1)) {
            // Note that this test gets triggered when "null" pointers are created
            // or copied even though they're not being dereferenced.   We flag for
            // a held fault and don't actually generate the fault unless the
            // instruction attempts to dereference through the "bad" pointer.
            if (! fault_gen_no_fault)
                cu.word1flags.oosb = 1;         //FIXME? nothing clears
            log_msg(INFO_MSG, moi, "Initial check: Segno outside DSBR bound of 0%o(%u) -- OOSB fault now pending.\n", cpup->DSBR.bound, cpup->DSBR.bound);
            if (! fault_gen_no_fault)
                cpu.apu_state.fhld = 1;
            return NULL;
        }
        if(1) log_msg(DEBUG_MSG, moi, "Fetching SDW for unpaged descriptor table from 0%o\n", cpup->DSBR.addr + 2 * segno);
        if (fetch_abs_pair(cpup->DSBR.addr + 2 * segno, &sdw_word0, &sdw_word1) != 0)
            return NULL;
    } else {
        // Descriptor table is paged
        if (segno * 2 >= 16 * (cpup->DSBR.bound + 1)) {
            // See comments just above re legal usage of "bad" pointers
            //  12/05/2008 -- bootload_1.alm, instr 17 triggers this (FIXED)
            log_msg(INFO_MSG, moi, "Initial check: Segno outside paged DSBR bound of 0%o(%u) -- OOSB fault now pending.\n", cpup->DSBR.bound, cpup->DSBR.bound);
            if (! fault_gen_no_fault)
                cu.word1flags.oosb = 1;         // ERROR: nothing clears
            // fault_gen(acc_viol_fault);
            if (! fault_gen_no_fault)
                cpu.apu_state.fhld = 1;
            // cancel_run(STOP_WARN);
            return NULL;
        }

        // First, the DSPTW fetch cycle (PTWAM doesn't cache the DS?)
        uint y1 = (2 * segno) % page_size;          // offset within page table
        uint x1 = (2 * segno - y1) / page_size;     // offset within DS page
        PTW_t DSPTW;
        t_uint64 word;
        if(1) log_msg(DEBUG_MSG, moi, "Fetching DS-PTW for paged descriptor table from 0%o\n", cpup->DSBR.addr + x1);
        if (fetch_abs_word(cpup->DSBR.addr + x1, &word) != 0)   // We assume DS is 1024 words (bound=077->16*64=1024)
            return NULL;
        decode_PTW(word, &DSPTW);   // TODO: cache this
        if (DSPTW.f == 0) {
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "DSPTW directed fault\n");
            fault_gen(dir_flt0_fault + DSPTW.fc);   // Directed Faults 0..4 use sequential fault numbers
            return NULL;
        }
        if (! DSPTW.u) {
            // MDSPTW cycle
            DSPTW.u = 1;
            if (set_PTW_used(cpup->DSBR.addr + x1) != 0) {
                // impossible -- we just read this absolute addresed word
                return NULL;
            }
        }
        // PSDW cycle (Step 5 for case when Descriptor Segment is paged)
        // log_msg(DEBUG_MSG, moi, "Fetching SDW from 0%o<<6+0%o => 0%o\n", DSPTW.addr, y1, (DSPTW.addr<<6) + y1);
        if (fetch_abs_pair((DSPTW.addr<<6) + y1, &sdw_word0, &sdw_word1) != 0)
            return NULL;
    }

    // Allocate a SDWAM entry
    if (oldest_sdwam == -1) {
        log_msg(ERR_MSG, moi, "SDWAM had no oldest entry\n");
        cancel_run(STOP_BUG);
        return NULL;
    }
    for (int i = 0; i < ARRAY_SIZE(cpup->SDWAM); ++i) {
        -- cpup->SDWAM[i].assoc.use;
    }
    SDWp = cpup->SDWAM + oldest_sdwam;
    decode_SDW(sdw_word0, sdw_word1, &SDWp->sdw);
    SDWp->assoc.ptr = segno;
    SDWp->assoc.use = 15;
    SDWp->assoc.is_full = 1;
    if (opt_debug) {
        log_msg(DEBUG_MSG, moi, "Allocated SDWAM # %o for seg 0%o: addr - 0%o, bound = 0%o(%d), f=%d\n",
            oldest_sdwam, segno, SDWp->sdw.addr, SDWp->sdw.bound, SDWp->sdw.bound, SDWp->sdw.f);
    }

    //log_msg(DEBUG_MSG, moi, "SDW: addr - 0%o, bound = 0%o(%d), f=%d\n",
    //  SDWp->sdw.addr, SDWp->sdw.bound, SDWp->sdw.bound, SDWp->sdw.f);

    return SDWp;
}

//=============================================================================

static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp)
{
    // Second part of page_in()
    // 

    const char* moi = "APU::append::page-in-page";

    uint segno = TPR.TSR;   // TSR should be been loaded with PPR.PSR if this is an instr fetch...

    // Following done for either paged or unpaged segments
    if (SDWp->sdw.f == 0) {
        log_msg(INFO_MSG, "APU::append", "SDW directed fault\n");
        fault_gen(dir_flt0_fault + SDWp->sdw.fc);   // Directed Faults 0..4 use sequential fault numbers
        return 1;
    }
    uint bound = 16 * (SDWp->sdw.bound + 1);
    if (offset >= bound) {
        if (! fault_gen_no_fault)
            cu.word1flags.oosb = 1;         // ERROR: nothing clears
        log_msg(NOTIFY_MSG, "APU::append", "SDW: Offset=0%o(%u), bound = 0%o(%u) -- OOSB fault\n", offset, offset, SDWp->sdw.bound, SDWp->sdw.bound);
        if (! fault_gen_no_fault)
            fault_gen(acc_viol_fault);
        return 1;
    }

    // ERROR: check access bits of SDW.{r,e,etc} versus reference (perm_mode arg)
    if (perm_mode != 0) {
        log_msg(WARN_MSG, "APU::append", "Segment permission checking not implemented\n");
        cancel_run(STOP_WARN);
    }

    if (SDWp->sdw.u) {
        // Segment is unpaged (it is contiguous) -- FANP cycle
        *addrp = SDWp->sdw.addr + offset;
        *minaddrp = SDWp->sdw.addr;
        *maxaddrp = SDWp->sdw.addr + bound - 1;
        // log_msg(DEBUG_MSG, "APU::append", "Resulting addr is 0%o (0%o+0%o)\n", *addrp,  SDWp->sdw.addr, offset);
    } else {
        // Segment is paged -- find appropriate page
        // First, Step 10 -- get PTW
        // Check to see if PTW for segno is in PTWAM
        // Save results across invocations so that locality of reference helps
        uint y2 = offset % page_size;           // offset within page
        uint x2 = (offset - y2) / page_size;    // page number
        static PTWAM_t *PTWp = 0;
        if (PTWp == 0) PTWp = cpup->PTWAM;  // FIXME expose to reset? // BUG: won't work for multiple CPUs
        int oldest_ptwam = -1;
        // TODO: performance: cache last index instead and start search from there
        if (PTWp == NULL || PTWp->assoc.ptr != segno || PTWp->assoc.pageno != x2 || ! PTWp->assoc.is_full) {    // todo: validate NULL as impossible
            PTWp = NULL;
            for (int i = 0; i < ARRAY_SIZE(cpup->PTWAM); ++i) {
                if (cpup->PTWAM[i].assoc.ptr == segno && cpup->PTWAM[i].assoc.pageno == x2 && cpup->PTWAM[i].assoc.is_full) {
                    PTWp = cpup->PTWAM + i;
                    // log_msg(DEBUG_MSG, "APU::append", "Found PTW for (segno 0%o, page 0%o) in PTWAM[%d]\n", segno, x2, i);
                    break;
                }
                if (cpup->PTWAM[i].assoc.use == 0)
                    oldest_ptwam = i;
                //if (! cpup->PTWAM[i].assoc.is_full) {
                //  log_msg(DEBUG_MSG, "APU::append", "PTW[%d] is not full\n", i);
                //}
            }
        } else {
            // log_msg(DEBUG_MSG, "APU::append", "PTW for (segno %#o, page %#o) is the MRU -- in PTWAM[%d]\n", segno, x2, PTWp - cpup->PTWAM);
        }
        if (PTWp != NULL) {
            // PTW is in PTWAM; it becomes the LRU
            if (PTWp->assoc.use != 15) {
                for (int i = 0; i < ARRAY_SIZE(cpup->PTWAM); ++i) {
                    if (cpup->PTWAM[i].assoc.use > PTWp->assoc.use)
                        -- cpup->PTWAM[i].assoc.use;
                }
                PTWp->assoc.use = 15;
            }
        } else {
            // Fetch PTW and put into PTWAM -- PTW cycle
            if (oldest_ptwam == -1) {
                log_msg(ERR_MSG, "APU::append", "PTWAM had no oldest entry\n");
                cancel_run(STOP_BUG);
                return 1;
            }
            t_uint64 word;
            uint ptw_addr = SDWp->sdw.addr + x2;
            // log_msg(DEBUG_MSG, "APU::append", "Fetching PTW for (segno %#o, page %#o) from %#o(%#o+page)\n", segno, x2, ptw_addr, SDWp->sdw.addr);
            if (fetch_abs_word(ptw_addr, &word) != 0)
                return 1;
            for (int i = 0; i < ARRAY_SIZE(cpup->PTWAM); ++i) {
                -- cpup->PTWAM[i].assoc.use;
            }
            PTWp = cpup->PTWAM + oldest_ptwam;
            decode_PTW(word, &PTWp->ptw);
            PTWp->assoc.use = 15;
            PTWp->assoc.ptr = segno;
            PTWp->assoc.pageno = x2;
            PTWp->assoc.is_full = 1;
            if (PTWp->ptw.f == 0) {
                log_msg(INFO_MSG, "APU::append", "PTW directed fault in segment %#o for PTW at %#o\n", segno, ptw_addr);
                fault_gen(dir_flt0_fault + PTWp->ptw.fc);   // Directed Faults 0..4 use sequential fault numbers
                return 1;
            }
        }
        *minaddrp = (PTWp->ptw.addr << 6);
        *addrp = *minaddrp + y2;
        *maxaddrp = *minaddrp + page_size - 1;
        // log_msg(DEBUG_MSG, "APU::append", "Resulting addr is 0%o (0%o<<6+0%o)\n", *addrp,  PTWp->ptw.addr, y2);
    }

    if(opt_debug>0) log_msg(DEBUG_MSG, "APU::append", "Resulting addr for %s 0%o|0%o is 0%o; range is 0%o to 0%o\n", SDWp->sdw.u ? "unpaged" : "paged", segno, offset, *addrp, *minaddrp, *maxaddrp);

    return 0;
}

//=============================================================================

static void decode_PTW(t_uint64 word, PTW_t *ptwp)
{
    ptwp->addr = getbits36(word, 0, 18);
    ptwp->u = getbits36(word, 26, 1);
    ptwp->m = getbits36(word, 29, 1);
    ptwp->f = getbits36(word, 33, 1);
    ptwp->fc = getbits36(word, 34, 2);
}

//=============================================================================

static int set_PTW_used(uint addr)
{
    t_uint64 word;
    if (fetch_abs_word(addr, &word) != 0)
        return 1;
    word = setbits36(word, 26, 1, 1);
    return store_abs_word(addr, word);
}

//=============================================================================

char* print_ptw(t_uint64 word)
{
    static char buf[100];
    PTW_t ptw;
    decode_PTW(word, &ptw);
    sprintf(buf, "[addr=0%o, u=%d, m=%d, f=%d, fc=%d]",
        ptw.addr, ptw.u, ptw.m, ptw.f, ptw.fc);
    return buf;
}

//=============================================================================

static void decode_SDW(t_uint64 word0, t_uint64 word1, SDW_t *sdwp)
{
    sdwp->addr = getbits36(word0, 0, 24);
    sdwp->r1 = getbits36(word0, 24, 3);
    sdwp->r2 = getbits36(word0, 27, 3);
    sdwp->r3 = getbits36(word0, 30, 3);
    // ssdr instruction stores zeros to 'f' and 'fc'
    // Figure 5-5 indicates usage of 33..35. ssdp will store 'f'.
    sdwp->f = getbits36(word0, 33, 1);
    sdwp->fc = getbits36(word0, 34, 2);

    sdwp->bound = getbits36(word1, 1, 14);
    sdwp->r = getbits36(word1, 15, 1);
    sdwp->e = getbits36(word1, 16, 1);
    sdwp->w = getbits36(word1, 17, 1);
    sdwp->priv = getbits36(word1, 18, 1);
    sdwp->u = getbits36(word1, 19, 1);
    sdwp->g = getbits36(word1, 20, 1);
    sdwp->c = getbits36(word1, 21, 1);
    sdwp->cl = getbits36(word1, 22, 14);
}

//=============================================================================

char* print_sdw(t_uint64 word0, t_uint64 word1)
{
    SDW_t sdw;
    decode_SDW(word0, word1, &sdw);
    return sdw2text(&sdw);
}

//=============================================================================

char *sdw2text(const SDW_t *sdwp)
{
    static char buf[500];
#if 1
    uint bound = 16 * (sdwp->bound + 1);
    sprintf(buf, "[addr=0%o, r(123)=(0%o,0%o,0%o), f=%d, fc=%o; bound=0%o(%d)->%#o(%d), r=%d,e=%d,w=%d,p=%d,u=%d,g=%d,c=%d, cl=0%o]",
        sdwp->addr, sdwp->r1, sdwp->r2, sdwp->r3, sdwp->f, sdwp->fc,
        sdwp->bound, sdwp->bound, bound, bound, sdwp->r, sdwp->e, sdwp->w, sdwp->priv, sdwp->u, sdwp->g, sdwp->c, sdwp->cl);
#else
    sprintf(buf, "[addr=0%o, r(123)=(0%o,0%o,0%o), f=%d, fc=%o; bound=0%o(%d), r=%d,e=%d,w=%d,p=%d,u=%d,g=%d,c=%d, cl=0%o]",
        sdwp->addr, sdwp->r1, sdwp->r2, sdwp->r3, sdwp->f, sdwp->fc,
        sdwp->bound, sdwp->bound, sdwp->r, sdwp->e, sdwp->w, sdwp->priv, sdwp->u, sdwp->g, sdwp->c, sdwp->cl);
#endif
    return buf;
}

//=============================================================================
