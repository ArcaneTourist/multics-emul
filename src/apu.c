/*
    apu.c -- The appending unit (APU) -- "address appending"
    Provides addressing support including virtual memory support.
    See also eis_mw_desc.c for address descriptors used by EIS multi-word
    instructions.

*/

// Supposedly, appending unit HW controls fault recognition?

#include "hw6180.h"

// #define FWBUG    // useful for testing console input

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 };

typedef struct {    // BUG
    int32 soffset; // Signed copy of CA (15 or 18 bits if from instr; 18 bits if from indir word)
    uint32 tag;
    flag_t more;
    enum atag_tm special;
} ca_temp_t;

static const int page_size = 1024;      // CPU allows [2^6 .. 2^12]; multics uses 2^10

static int compute_addr(const instr_t *ip, ca_temp_t *ca_tempp);
static int addr_append(t_uint64 *wordp);
//static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp);
static int do_its_itp(const instr_t* ip, ca_temp_t *ca_tempp, t_uint64 word01);
static int page_in(uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp);
static void decode_PTW(t_uint64 word, PTW_t *ptwp);
static int set_PTW_used(uint addr);
static void decode_SDW(t_uint64 word0, t_uint64 word1, SDW_t *sdwp);
// static SDW_t* get_sdw(void);
static SDWAM_t* page_in_sdw(void);
static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp);
static void reg_mod_x(uint td, int off, int nbits);

//=============================================================================

#if 0
static t_bool is_transfer_op(int op)
{
    return 0;   // BUG
}
#endif

//=============================================================================

static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        return r;
    }
    else
        return x;
}

static int32 sign15(uint x)
{
    if (bit_is_neg(x,15)) {
        int32 r = - ((1<<15) - (x&MASKBITS(15)));
        return r;
    }
    else
        return x;
}

#if 0
static int32 sign12(uint x)
{
    if (bit_is_neg(x,12)) {
        int32 r = - ((1<<12) - (x&MASKBITS(12)));
        return r;
    }
    else
        return x;
}
#endif

static inline uint max3(uint a, uint b, uint c)
{
    return (a > b) ?
        ((a > c) ? a : c) :
        ((b > c) ? b : c);
}


//=============================================================================


void set_addr_mode(addr_modes_t mode)
{
    // BUG: set_addr_mode() probably belongs in CPU
    if (mode == ABSOLUTE_mode) {
        IR.abs_mode = 1;
        IR.not_bar_mode = 1;
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Setting absolute mode.\n");
    } else if (mode == APPEND_mode) {       // BUG: is this correct?
        IR.abs_mode = 0;
        IR.not_bar_mode = 1;
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Setting append mode.\n");
    } else if (mode == BAR_mode) {
        IR.abs_mode = 0;    // BUG: is this correct?
        IR.not_bar_mode = 0;
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Setting bar mode.\n");
    } else {
        log_msg(ERR_MSG, "APU", "Unable to determine address mode.\n");
        cancel_run(STOP_BUG);
    }
}


//=============================================================================


addr_modes_t get_addr_mode()
{
    // BUG: get_addr_mode() probably belongs in CPU

    if (IR.abs_mode)
        return ABSOLUTE_mode;

    // BUG: addr mode depends upon instr's operand

    if (IR.not_bar_mode == 0) {
        log_msg(WARN_MSG, "APU", "BAR mode is untested\n");
        cancel_run(STOP_WARN);
        return BAR_mode;
    }

    return APPEND_mode;
}

//=============================================================================

int is_priv_mode()
{
    if (IR.abs_mode)
        return 1;
    SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
    if (SDWp == NULL) {
        log_msg(WARN_MSG, "APU::is-priv-mode", "Segment does not exist?!?\n");
        cancel_run(STOP_BUG);
        return 0;   // arbitrary
    }
    if (SDWp->priv)
        return 1;
    if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Priv check fails\n");
    return 0;
}

//=============================================================================

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
        if (ip->mods.single.pr_bit == 0) {
            uint32 offset = ip->addr;
            int32 soffset = sign18(ip->addr);
            sprintf(buf, "%s, offset 0%06o(%+d), inhibit %u, pr=N, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                offset, soffset, 
                ip->inhibit, ip->mods.single.tag, ip->mods.single.tag >> 4, ip->mods.single.tag & 017);
        } else {
            uint pr = ip->addr >> 15;
            int32 offset = ip->addr & MASKBITS(15);
            int32 soffset = sign15(offset);
            sprintf(buf, "%s, PR %d, offset 0%06o(%+d), inhibit %u, pr=Y, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                pr, offset, soffset, 
                ip->inhibit, ip->mods.single.tag, ip->mods.single.tag >> 4, ip->mods.single.tag & 017);
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

//=============================================================================


int get_address(uint y, flag_t ar, uint reg, int nbits, uint *addrp, int* bitnop, uint *minaddrp, uint* maxaddrp)
{
    // Return absolute address given an address, 'ar' flag, 'reg' modifier, and
    // nbits.  Nbits is the data size and is used only for reg modifications.
    // Arg ar should be negative to use current TPR or non-negative to use a
    // pointer/address register.
    // BUG: some callers may keep results.  This isn't valid for multi-page segments.
    
    // BUG: handle BAR mode and abs mode as described in EIS indir doc
    addr_modes_t addr_mode = get_addr_mode();

    uint offset;
    uint saved_PSR = 0, saved_PRR = 0, saved_CA = 0, saved_bitno = 0;
    if (ar) {
        saved_PSR = TPR.TSR;
        saved_PRR = TPR.TRR ;
        saved_CA = TPR.CA;
        saved_bitno = TPR.TBR;
        //
        uint pr = y >> 15;
        int32 soffset = sign15(y & MASKBITS(15));
        TPR.TSR = AR_PR[pr].PR.snr;
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR);
        offset = AR_PR[pr].wordno + soffset;
        TPR.TBR = AR_PR[pr].PR.bitno;
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::get-addr", "Using PR[%d]: TSR=0%o, TRR=0%o, offset=0%o(0%o+0%o), bitno=0%o\n",
            pr, TPR.TSR, TPR.TRR, offset, AR_PR[pr].wordno, soffset, TPR.TBR);
        *(uint*)bitnop = TPR.TBR;
    } else {
        offset = y;
        // sofset = sign18(y);
        *bitnop = 0;
    }

    if (reg != 0) {
        saved_CA = TPR.CA;
        TPR.CA = 0;
        uint o = offset;
#ifdef FWBUG
        // generate buggy result for historical tracking purposes
        log_msg(ERR_MSG, "APU::get-addr", "Generating incorrect results for debugging.\n");
        if (TPR.TSR == 0401 && offset == 011)
            reg_mod(reg, offset);
        else
            reg_mod_x(reg, offset, nbits);
        cancel_run(STOP_BUG);
#else
        reg_mod_x(reg, offset, nbits);
#endif
        offset = TPR.CA;
        TPR.CA = saved_CA;
        // BUG: ERROR: Apply EIS reg mod -- is this already fixed?
        log_msg(DEBUG_MSG, "APU::get-addr", "Register mod 0%o: offset was 0%o, now 0%o\n", reg, o, offset);
        //log_msg(WARN_MSG, "APU::get-addr", "Auto-breakpoint\n");
        //cancel_run(STOP_IBKPT);
    }

    uint perm = 0;  // BUG: need to have caller specify
    int ret = page_in(offset, perm, addrp, minaddrp, maxaddrp);
    if (ret != 0) {
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU::get-addr", "page-in faulted\n");
    }

    if (ar) {
        TPR.TSR = saved_PSR;
        TPR.TRR = saved_PRR;
        TPR.CA = saved_CA;
        TPR.TBR = saved_bitno;
    }

    return ret;
}

//=============================================================================

static int temp_addr_mod(const instr_t *ip);
extern DEVICE cpu_dev;
int addr_mod(const instr_t *ip)
{
    int saved_debug = opt_debug;
    int saved_dctrl = cpu_dev.dctrl;
    int ret = temp_addr_mod(ip);
    opt_debug = saved_debug;
    cpu_dev.dctrl = saved_dctrl;
    return ret;
}

static int temp_addr_mod(const instr_t *ip)
{
    // Called by OPU for most instructions
    // Generate 18bit computed address TPR.CA
    // Returns non-zero on error or group 1-6 fault

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

    // BUG: do reg and indir word stuff first?

    TPR.is_value = 0;   // BUG: Use "direct operand flag" instead
    TPR.value = 0xdeadbeef;

    // Addr appending below

    addr_modes_t orig_mode = get_addr_mode();
    addr_modes_t addr_mode = orig_mode;
    int ptr_reg_flag = ip->mods.single.pr_bit;
    ca_temp_t ca_temp;  // BUG: hack

    // BUG: The following check should only be done after a sequential
    // instr fetch, not after a transfer!  We're only called by do_op(),
    // so this criteria is *almost* met.   Need to detect transfers.
    // Figure 6-10 claims we update the TPR.TSR segno as instructed by a "PR" bit 29
    // only if we're *not* doing a sequential instruction fetch.

    ca_temp.tag = ip->mods.single.tag;

    if (cu.rpt) {
        TPR.TBR = 0;
        cu.tag = ca_temp.tag;
        uint td = ca_temp.tag & 017;
        int n = td & 07;
        if (cu.repeat_first) {
            if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "RPT: First repetition; incr will be 0%o(%d).\n", ip->addr, ip->addr);
            TPR.CA = ip->addr;
            ca_temp.soffset = ip->addr; // BUG: do we need to sign-extend to allow for negative "addresses"?
        } else {
            // Note that we don't add in a delta for X[n] here.   Instead the CPU
            // increments X[n] after every instruction.  So, for instructions like
            // cmpaq, the index register points to the entry after the one found.
            //log_msg(DEBUG_MSG, "APU", "RPT: Non-first repetition; delta will be 0%o(%d).\n", cu.delta, cu.delta);
            //TPR.CA = cu.delta;
            //ca_temp.soffset = cu.delta;   // BUG: do we need to sign-extend to allow for negative "addresses"?
            TPR.CA = 0;
            ca_temp.soffset = 0;
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "RPT: X[%d] is 0%o(%d).\n", n, reg_X[n], reg_X[n]);
        }
    } else if (ptr_reg_flag == 0) {
        ca_temp.soffset = sign18(ip->addr);
        // TPR.TSR = PPR.PSR;   -- done prior to fetch_instr()
        // TPR.TRR = PPR.PRR;   -- done prior to fetch_instr()
        TPR.CA = ip->addr;
        TPR.TBR = 0;
    } else {
        set_addr_mode(addr_mode = APPEND_mode);
        // AL39: Page 341, Figure 6-7 shows 3 bit PR & 15 bit offset
        int32 offset = ip->addr & MASKBITS(15);
        ca_temp.soffset = sign15(offset);
        uint pr = ip->addr >> 15;
        TPR.TSR = AR_PR[pr].PR.snr;     // BUG: see comment above re figure 6-10
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR);
        TPR.CA = (AR_PR[pr].wordno + ca_temp.soffset) & MASK18;
        TPR.TBR = AR_PR[pr].PR.bitno;
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Using PR[%d]: TSR=0%o, TRR=0%o, CA=0%o(0%o+0%o<=>%d+%d), bitno=0%o\n",
            pr, TPR.TSR, TPR.TRR, TPR.CA, AR_PR[pr].wordno, ca_temp.soffset, AR_PR[pr].wordno, ca_temp.soffset, TPR.TBR);
        ca_temp.soffset = sign18(TPR.CA);

        // BUG: Enter append mode & stay if execute a transfer
    }

    int op = ip->opcode;
    int bit27 = op % 2;
    op >>= 1;
    if (bit27 == 0 && op == opcode0_stca)
        return 0;


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
            // if (ca_temp.more) log_msg(NOTIFY_MSG, "APU", "Final (incomplete) CA: 0%0o\n", TPR.CA);
            if (ca_temp.more) log_msg(NOTIFY_MSG, "APU", "Not-Final (not-incomplete) CA: 0%0o\n", TPR.CA);
            ca_temp.soffset = sign18(TPR.CA);
            // return 1;
        }
        if (ca_temp.more)
            mult = 1;
        ca_temp.soffset = sign18(TPR.CA);
        if (ca_temp.more)
            if(opt_debug>0)log_msg(DEBUG_MSG, "APU", "Post CA: Continuing indirect fetches\n");
#if 0
        if (ca_temp.more)
            log_msg(DEBUG_MSG, "APU", "Pre Seg: Continuing indirect fetches\n");
        if (do_esn_segmentation(ip, &ca_temp) != 0) {
            log_msg(DEBUG_MSG, "APU", "Final (incomplete) CA: 0%0o\n", TPR.CA);
            return 1;
        }
        if (ca_temp.more)
            log_msg(DEBUG_MSG, "APU", "Post Seg: Continuing indirect fetches\n");
#endif
        if (ca_temp.more)
            mult = 1;
    }
    if (mult) {
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Final CA: 0%0o\n", TPR.CA);
    }

    addr_mode = get_addr_mode();    // may have changed

    if (addr_mode == BAR_mode) {
        if (addr_mode == BAR_mode && ptr_reg_flag == 0) {
            // Todo: Add CA to BAR.base; add in PR; check CA vs bound
        }
        // BUG: Section 4 says make sure CA cycle handled AR reg mode & constants
        log_msg(ERR_MSG, "APU", "BAR mode not implemented.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    if (addr_mode == ABSOLUTE_mode && ptr_reg_flag == 0) {
        // TPR.CA is the 18-bit absolute main memory addr
        return 0;
    }

    // APPEND mode handled by fetch_word() etc

    return 0;
}


static int compute_addr(const instr_t *ip, ca_temp_t *ca_tempp)
{
    // Perform a "CA" cycle as per figure 6-2 of AL39.
    // Generate an 18-bit computed address (in TPR.CA) as specified in section 6
    // of AL39.
    // In our version, this may include replacing TPR.CA with a 36 bit constant or
    // other value if an appropriate modifier (e.g., du) is present.

    ca_tempp->more = 0;

    // BUG: Need to do ESN special handling if loop is continued

    enum atag_tm tm = (ca_tempp->tag >> 4) & 03;    // the and is a hint to the compiler for the following switch...

    uint td = ca_tempp->tag & 017;

    ca_tempp->special = tm;

    if (cu.rpt) {
        // Check some requirements, but don't generate a fault (AL39 doesn't say if we should fault)
        if (tm != atag_r && tm != atag_ri)
            log_msg(ERR_MSG, "APU", "Repeated instructions must use register or register-indirect address modes.\n");
        else
            if (td == 0)
                log_msg(ERR_MSG, "APU", "Repeated instructions should not use X[0].\n");
    }

    switch(tm) {
        case atag_r: {  // Tm=0 -- register (r)
            if (td != 0)
                reg_mod(td, ca_tempp->soffset);
            if (cu.rpt) {
                int n = td & 07;
                reg_X[n] = TPR.CA;
            }
            return 0;
        }
        case atag_ri: {     // Tm=1 -- register then indirect (ri)
            if (td == 3 || td == 7) {
                // ",du" or ",dl"
                log_msg(WARN_MSG, "APU", "RI with td==0%o is illegal.\n", td);
                fault_gen(illproc_fault);   // need illmod sub-category
                return 1;
            }
            int off = ca_tempp->soffset;
            uint ca = TPR.CA;
            reg_mod(td, off);
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "RI: pre-fetch:  TPR.CA=0%o <==  TPR.CA=%o + 0%o\n",
                TPR.CA, ca, TPR.CA - ca);
            t_uint64 word;
            if (cu.rpt) {
                int n = td & 07;
                reg_X[n] = TPR.CA;
                if (opt_debug>0) {
                    log_msg(DEBUG_MSG, "APU", "RI for repeated instr: Setting X[%d] to CA 0%o(%d).\n", n, reg_X[n], reg_X[n]);
                    log_msg(DEBUG_MSG, "APU", "RI for repeated instr: Not doing address appending on CA.\n");
                }
                if (fetch_abs_word(TPR.CA, &word) != 0)
                    return 1;
            } else
                if (addr_append(&word) != 0)
                    return 1;
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "RI: fetch:  word at TPR.CA=0%o is 0%Lo\n", TPR.CA, word);
            if (cu.rpt) {
                // ignore tag and don't allow more indirection
                return 0;
            }
            ca_tempp->tag = word & MASKBITS(6);
            if (TPR.CA % 2 == 0 && (ca_tempp->tag == 041 || ca_tempp->tag == 043)) {
                    // ++ opt_debug; ++ cpu_dev.dctrl;
                    int ret = do_its_itp(ip, ca_tempp, word);
                    if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "RI: post its/itp: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
                    if (ret != 0) {
                        if (ca_tempp->tag != 0) {
                            log_msg(WARN_MSG, "APU", "RI: post its/itp: canceling remaining APU cycles.\n");
                            cancel_run(STOP_WARN);
                        }
                        return ret;
                    }
            } else {
                TPR.CA = word >> 18;
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "RI: post-fetch: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
            }
            // break;   // Continue a new CA cycle
            ca_tempp->more = 1;     // Continue a new CA cycle
            // BUG: flowchart says start CA, but we do ESN
            return 0;
        }
        case atag_it: { // Tm=2 -- indirect then tally (it)
            // BUG: see "it" flowchart for looping (Td={15,17}
            switch(td) {
                case 0:
                    log_msg(WARN_MSG, "APU", "IT with Td zero not valid in instr word.\n");
                    fault_gen(f1_fault);    // This mode not ok in instr word
                    break;
                case 6:     
                    log_msg(WARN_MSG, "APU", "IT with Td six is a fault.\n");
                    fault_gen(fault_tag_2_fault);   // This mode not ok in instr word
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
                        IR.tally_runout = (tally == 0); // BUG: do we need to fault?
                        --addr;
                        addr &= MASK18; // wrap from zero to 2^18-1
                        iword = setbits36(iword, 0, 18, addr);
                        iword = setbits36(iword, 18, 12, tally);
                        TPR.CA = addr;
                        if (opt_debug) {
                            // give context for appending msgs
                            log_msg(DEBUG_MSG, "APU", "IT(di): addr now 0%o, tally 0%o\n", addr, tally);
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
                    log_msg(ERR_MSG, "APU", "IT with Td 0%o not implmented.\n", td);
                    cancel_run(STOP_BUG);
                    return 1;
            }
            break;
        }
        case atag_ir: { // TM=3 -- indirect then register (ir)
#if 0
            log_msg(ERR_MSG, "APU", "IR addr mod not implemented.\n");
            cancel_run(STOP_BUG);
            return 1;
#else
            int nloops = 0;
            while(tm == atag_ir || tm == atag_ri) {
                if (++nloops > 1)
                    log_msg(NOTIFY_MSG, "APU::IR", "loop # %d\n", nloops);
                if (tm == atag_ir)
                    cu.CT_HOLD = td;
                // BUG: Maybe handle special tag (41 itp, 43 its).  Or post handle?
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR", "pre-fetch: Td=0%o, TPR.CA=0%o\n", td, TPR.CA);
                t_uint64 word;
                if (addr_append(&word) != 0)
                    return 1;
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR", "fetched:  word at TPR.CA=0%o is 0%Lo:\n",
                    TPR.CA, word);
                ca_tempp->tag = word & MASKBITS(6);
                if (TPR.CA % 2 == 0 && (ca_tempp->tag == 041 || ca_tempp->tag == 043)) {
                        // ++ opt_debug; ++ cpu_dev.dctrl;
                        log_msg(NOTIFY_MSG, "APU::IR", "found ITS/ITP\n");  // BUG: remove this msg
                        int ret = do_its_itp(ip, ca_tempp, word);
                        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR", "post its/itp: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
                        if (ret != 0) {
                            if (ca_tempp->tag != 0) {
                                log_msg(WARN_MSG, "APU", "IR: post its/itp: canceling remaining APU cycles.\n");
                                cancel_run(STOP_WARN);
                            }
                            return ret;
                        }
                } else {
                    TPR.CA = word >> 18;
                    tm = (ca_tempp->tag >> 4) & 03;
                    td = ca_tempp->tag & 017;
                    if(opt_debug>0) log_msg(DEBUG_MSG, "APU::IR", "post-fetch: TPR.CA=0%o, tag=0%o, new tm=0%o; td = %o\n", TPR.CA, ca_tempp->tag, tm, td);
                    if (td == 0) {
                        // BUG: Disallow a reg_mod() with td equal to NULL (AL39)
                        // Disallow always or maybe ok for ir?
                        log_msg(ERR_MSG, "APU::IR", "Found td==0 (for tm=0%o)\n", tm);
                        cancel_run(STOP_WARN);
                    }
                    switch(tm) {
                        case atag_ri:
                            log_msg(WARN_MSG, "APU::IR", "IR followed by RI.  Not tested\n");
                            reg_mod(td, ca_tempp->soffset);
                            break;      // continue looping
                        case atag_r:
                            reg_mod(cu.CT_HOLD, ca_tempp->soffset);
                            return 0;
                        case atag_it:
                            //reg_mod(td, ca_tempp->soffset);
                            log_msg(ERR_MSG, "APU::IR", "Need to run normal IT algorithm, ignoring fault 1.\n"); // actually cannot have fault 1 if disallow td=0 above
                            cancel_run(STOP_BUG);
                            return 0;
                        case atag_ir:
                            log_msg(WARN_MSG, "APU::IR", "IR followed by IR, continuing to loop.  Not tested\n");
                            cu.CT_HOLD = ca_tempp->tag & MASKBITS(4);
                            break;      // keep looping
                    }
                }
                //log_msg(WARN_MSG, "APU::IR", "Finished, but unverified.\n");
                //cancel_run(STOP_WARN);
                log_msg(DEBUG_MSG, "APU::IR", "Finished.\n");
                return 0;
            }
#endif
        }
    }

    // BUG: Need to do ESN special handling if loop is continued

    return 0;
}


//=============================================================================

void reg_mod(uint td, int off)
{
    reg_mod_x(td, off, 36);
}

static int32 nbits2words(int x, int nbits)
{
    int div = 36 / nbits;
    if (x % div != 0) {
        log_msg(ERR_MSG, "APU", "Reg mod for %d-bit data: Value %d isn't evenly divisible by %d\n", nbits, x, div);
        cancel_run(STOP_BUG);
    }
    return x / div;
}

static void reg_mod_x(uint td, int off, int nbits)
{
    switch(td) {
        case 0:
            break;  // no mod
        case 1: // ,au
            TPR.CA = off + nbits2words(sign18(getbits36(reg_A, 0, 18)), nbits);
            TPR.CA &= MASK18;
            break;
        case 2: // ,qu
            TPR.CA = off + nbits2words(sign18(getbits36(reg_Q, 0, 18)), nbits);
            TPR.CA &= MASK18;
            break;
        case 3: // ,du
            TPR.is_value = td;  // BUG: Use "direct operand flag" instead
            TPR.value = ((t_uint64) TPR.CA) << 18;
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Mod du: Value from offset 0%o is 0%Lo\n", TPR.CA, TPR.value);
            break;
        case 4: // PPR.IC
            TPR.CA = off + PPR.IC;  // BUG: IC assumed to be unsigned
            TPR.CA &= MASK18;
            break;
        case 5:
            TPR.CA = off + nbits2words(sign18(getbits36(reg_A, 18, 18)), nbits);
            TPR.CA &= MASK18;
            if (opt_debug) {
                uint a = getbits36(reg_A, 18, 18);
                log_msg(DEBUG_MSG, "APU", "Tm=REG,Td=%02o: offset 0%o(%d) + A=0%Lo=>0%o(%+d decimal) ==> 0%o=>0%o(%+d)\n",
                    td, off, off, reg_A, a, sign18(a), TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
            }
            break;
        case 6:
            TPR.CA = off + nbits2words(sign18(getbits36(reg_Q, 18, 18)), nbits);
            TPR.CA &= MASK18;
            break;
        case 7: // ,dl
            TPR.is_value = td;  // BUG: Use "direct operand flag" instead
            TPR.value = TPR.CA; // BUG: Should we sign?
            if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "Mod dl: Value from offset 0%o is 0%Lo\n", TPR.CA, TPR.value);
            break;
        case 010:
        case 011:
        case 012:
        case 013:
        case 014:
        case 015:
        case 016:
        case 017:
            TPR.CA = off + nbits2words(sign18(reg_X[td&07]), nbits);
            TPR.CA &= MASK18;
            if (opt_debug)
                log_msg(DEBUG_MSG, "APU", "Tm=REG,Td=%02o: offset 0%o + X[%d]=0%o(%+d decimal)==>0%o(+%d) yields 0%o (%+d decimal)\n",
                    td, off, td&7, reg_X[td&7], reg_X[td&7], sign18(reg_X[td&7]), sign18(reg_X[td&7]), TPR.CA, TPR.CA);
            break;
    }
}

//=============================================================================

// static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp)

static int do_its_itp(const instr_t* ip, ca_temp_t *ca_tempp, t_uint64 word01)
{
    // Implements the portion of AL39 figure 6-10 that is below the "CA CYCLE" box

    // Just did an "ir" or "ri" addr modification
    if (ca_tempp->tag == 041) {
        // itp
        t_uint64 word1, word2;
        // BUG: are we supposed to fetch?
        int ret = fetch_pair(TPR.CA, &word1, &word2);   // bug: refetching word1
        if (ret != 0)
            return ret;
        if (word01 != word1) {
            log_msg(WARN_MSG, "APU:ITS/ITP", "Refetched word at %#o has changed.   Was %012Lo, now %012Lo.\n", word01, word1);
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
        ca_tempp->tag = word2 & MASKBITS(6);
        uint i_mod_tm = ca_tempp->tag >> 4;
        uint r;
        if (ca_tempp->special == atag_ir) {
            log_msg(DEBUG_MSG, "APU", "ITP: temp special is IR; Will use r from cu.CT_HOLD\n");
            r = cu.CT_HOLD;
        } else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
            uint i_mod_td = ca_tempp->tag & MASKBITS(4);
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
    } else if (ca_tempp->tag == 043) {
        // its
        t_uint64 word1, word2;
        // BUG: are we supposed to fetch?
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITS: CA initially 0%o\n", TPR.CA);
        int ret = fetch_pair(TPR.CA, &word1, &word2);   // bug: refetching word1
        if (ret != 0)
            return ret;
        set_addr_mode(APPEND_mode);
        TPR.TSR =  getbits36(word1, 3, 15);
        uint its_rn = getbits36(word1, 18, 3);
        SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
        uint sdw_r1;
        if (SDWp == NULL) {
            // BUG 12/05/2008 -- bootload_1.alm, instr 17 triggers this via an epp instr
            // BUG: it seems odd that we should ignore this fault
            log_msg(WARN_MSG, "APU:ITS/ITP", "Segment is missing.\n");
            cancel_run(STOP_BUG);
            sdw_r1 = 7;
            ret = 1;
        } else
            sdw_r1 = SDWp->r1;
        TPR.TRR = max3(its_rn, sdw_r1, TPR.TRR);
        TPR.TBR = getbits36(word2, 21, 6);
        ca_tempp->tag = word2 & MASKBITS(6);

        uint i_mod_tm = ca_tempp->tag >> 4;
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU", "ITS: TPR.TSR = 0%o, rn=0%o, sdw.r1=0%o, TPR.TRR=0%o, TPR.TBR=0%o, tag=0%o(tm=0%o)\n",
            TPR.TSR, its_rn, sdw_r1, TPR.TRR, TPR.TBR, ca_tempp->tag, i_mod_tm);
        uint r;
        if (ca_tempp->special == atag_ir) {
            log_msg(DEBUG_MSG, "APU", "ITS: temp special is IR; Will use r from cu.CT_HOLD\n");
            r = cu.CT_HOLD;
        } else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
            uint i_mod_td = ca_tempp->tag & MASKBITS(4);
            // r = i_mod_td;
            r = 0;  // the tag will be used during the next cycle
            log_msg(DEBUG_MSG, "APU", "ITS: temp special is RI; temp tag is r or ri; Will use r from special tag's td\n");
        } else {
            log_msg(ERR_MSG, "APU", "ITS addr mod with undefined r-value (tm=0%o,new-tm=0%o)\n", ca_tempp->special, i_mod_tm);
            cancel_run(STOP_BUG);
            r = 0;
        }
        uint i_wordno = getbits36(word2, 0, 18);
        // TPR.CA = i_wordno + r;
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
        // BUG: What does the question "Appending unit data movement?" mean?
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

int fetch_appended(uint offset, t_uint64 *wordp)
{
    // Implements AL39, figure 5-4
    // Returns non-zero if a fault in groups 1-6 detected
    // Note that we allow an arbitray offset not just TPR.CA.   This is to support
    // instruction fetches.
    // BUG: Need to handle y-pairs and writes

    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode == ABSOLUTE_mode)
        return fetch_abs_word(offset, wordp);
    if (addr_mode == BAR_mode) {
        log_msg(ERR_MSG, "APU::append", "BAR mode not implemented.\n");
        cancel_run(STOP_BUG);
        return fetch_abs_word(offset, wordp);
    }
    if (addr_mode != APPEND_mode) {
        // impossible
        log_msg(ERR_MSG, "APU::append", "Unknown mode\n");
        cancel_run(STOP_BUG);
        return fetch_abs_word(offset, wordp);
    }

    uint addr;
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, 0, &addr, &minaddr, &maxaddr);
    if (ret == 0) {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::fetch_append", "Using addr 0%o\n", addr);
        ret = fetch_abs_word(addr, wordp);
    } else {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::fetch_append", "page-in faulted\n");
    }
    return ret;
}


int store_appended(uint offset, t_uint64 word)
{
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != APPEND_mode) {
        // impossible
        log_msg(ERR_MSG, "APU::store-append", "Not APPEND mode\n");
        cancel_run(STOP_BUG);
    }

    uint addr;
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, 0, &addr, &minaddr, &maxaddr);
    if (ret == 0) {
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::store-append", "Using addr 0%o\n", addr);
        ret = store_abs_word(addr, word);
    } else
        if(opt_debug>0) log_msg(DEBUG_MSG, "APU::store-append", "page-in faulted\n");
    return ret;
}

//=============================================================================

void cmd_dump_vm()
{
    // Dump VM info -- However, we only know about what's in the cache registers

    printf("DSBR: addr=0%08o, bound=0%o(%d), unpaged=%c\n",
        DSBR.addr, DSBR.bound, DSBR.bound, DSBR.u ? 'Y' : 'N');
    if (cu.SD_ON)
        printf("SDWAM is enabled.\n");
    else
        printf("SDWAM is NOT enabled.\n");
    if (cu.PT_ON)
        printf("PTWAM is enabled.\n");
    else
        printf("PTWAM is NOT enabled.\n");
    if (DSBR.u)
        printf("DSBR: SDW for segment 'segno' is at %08o + 2 * segno.\n", DSBR.addr);
    else {
        printf("DSBR: Offset (y1) in page table for segment 'segno' is at (2 * segno)%%%d;\n", page_size);
        printf("DSBR: PTW for segment 'segno' is at 0%08o + (2 * segno - y1)/%d.\n", DSBR.addr, page_size);
        printf("DSBR: SDW for segment 'segno' is at PTW.addr<<6 + y1.\n");
    }
    for (int i = 0; i < ARRAY_SIZE(SDWAM); ++i) {
        printf("SDWAM[%d]: ptr(segno)=0%05o, is-full=%c, use=0%02o(%02d)\n",
            i, SDWAM[i].assoc.ptr, SDWAM[i].assoc.is_full ? 'Y' : 'N',
            SDWAM[i].assoc.use, SDWAM[i].assoc.use);
        SDW_t *sdwp = &SDWAM[i].sdw;
        printf("\tSDW for seg %d: addr = 0%08o, r1=%o r2=%o r3=%o, f=%c, fc=0%o.\n",
            SDWAM[i].assoc.ptr, sdwp->addr, sdwp->r1, sdwp->r2, sdwp->r3, sdwp->f ? 'Y' : 'N', sdwp->fc);
        printf("\tbound = 0%05o(%d) => %06o(%u)\n",
            sdwp->bound, sdwp->bound, sdwp->bound * 16 + 16, sdwp->bound * 16 + 16);
        printf("\tr=%c e=%c w=%c, priv=%c, unpaged=%c, g=%c, c=%c, cl=%05o\n",
            sdwp->r ? 'Y' : 'N', sdwp->e ? 'Y' : 'N', sdwp->w ? 'Y' : 'N',
            sdwp->priv ? 'Y' : 'N', sdwp->u ? 'Y' : 'N', sdwp->g ? 'Y' : 'N',
            sdwp->c ? 'Y' : 'N', sdwp->cl);
    }
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

int get_seg_addr(uint offset, uint perm_mode, uint *addrp)
{
    // BUG: causes faults
    uint minaddr, maxaddr;  // results unneeded
    int ret = page_in(offset, perm_mode, addrp, &minaddr, &maxaddr);
    if (ret)
        log_msg(WARN_MSG, "APU::get_seg_addr", "page-in faulted\n");
    return ret;
}

static int page_in(uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp)
{
    // Implements AL39, figure 5-4
    // Returns non-zero if a fault in groups 1-6 detected
    // Note that we allow an arbitrary offset not just TPR.CA.   This is to support
    // instruction fetches.
    // Resulting 24bit physical memory addr stored in addrp.

    const char *moi = "APU::append::page-in";
    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...
    if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting for Segno=0%o, offset=0%o.  (PPR.PSR is 0%o)\n", segno, offset, PPR.PSR);

    // ERROR: Validate that all PTWAM & SDWAM entries are always "full" and that use fields are always sane
    SDWAM_t* SDWp = page_in_sdw();
    if (SDWp == NULL) {
        log_msg(WARN_MSG, moi, "SDW not loaded\n");
        return 1;
    }
    int ret = page_in_page(SDWp, offset, perm_mode, addrp, minaddrp, maxaddrp);
    if (ret != 0) {
        log_msg(WARN_MSG, moi, "page_in_page returned non zero.  Segno 0%o, offset 0%o(%d)\n", segno, offset);
    }
    return ret;
}


static SDWAM_t* page_in_sdw()
{
    // Implements half of page_in() -- AL39, figure 5-4
    // Returns NULL if a fault in groups 1-6 detected

    // ERROR: Validate that all PTWAM & SDWAM entries are always "full" and that use fields are always sane

    // TODO: Replace most of this with more efficient methods that match the HW less well


    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...
    
    // BUG: Need bounds checking at all cycles below except PSDW cycle

    // Check to see if SDW for segno is in SDWAM
    // Save results across invocations so that locality of reference avoids search
    static SDWAM_t *SDWp = SDWAM;   // BUG: expose to reset?
    int oldest_sdwam = -1;
    if (SDWp == NULL || SDWp->assoc.ptr != segno || ! SDWp->assoc.is_full) {    // todo: validate NULL as impossible
        SDWp = NULL;
        for (int i = 0; i < ARRAY_SIZE(SDWAM); ++i) {
            if (SDWAM[i].assoc.ptr == segno && SDWAM[i].assoc.is_full) {
                SDWp = SDWAM + i;
                // log_msg(DEBUG_MSG, "APU::append", "Found SDW for segno 0%o in SDWAM[%d]\n", segno, i);
                break;
            }
            //if (! SDWAM[i].assoc.is_full) {
            //  log_msg(DEBUG_MSG, "APU::append", "Found SDWAM[%d] is unused\n", i);
            //}
            if (SDWAM[i].assoc.use == 0)
                oldest_sdwam = i;
        }
    } else {
        // log_msg(DEBUG_MSG, "APU::append", "SDW for segno 0%o is the MRU -- in SDWAM[%d]\n", segno, SDWp-SDWAM);
    }

    if (SDWp != NULL) {
        // SDW is in SDWAM; it becomes the LRU
        // log_msg(DEBUG_MSG, "APU::append", "SDW is in SDWAM[%d].\n", SDWp - SDWAM);
        if (SDWp->assoc.use != 15) {
            for (int i = 0; i < ARRAY_SIZE(SDWAM); ++i) {
                if (SDWAM[i].assoc.use > SDWp->assoc.use)
                    -- SDWAM[i].assoc.use;
            }
            SDWp->assoc.use = 15;
        }
        return SDWp;
    }

    // Fetch SDW and place into SDWAM
    if(opt_debug>0) log_msg(DEBUG_MSG, "APU::append", "SDW for segno 0%o is not in SDWAM.  DSBR addr is 0%o\n", segno, DSBR.addr);
    t_uint64 sdw_word0, sdw_word1;
    if (DSBR.u) {
        // Descriptor table is unpaged
        // Do a NDSW cycle
        if (segno * 2 >= 16 * (DSBR.bound + 1)) {
            cu.word1flags.oosb = 1;         // ERROR: nothing clears
            log_msg(WARN_MSG, "APU::append", "Initial check: Segno outside DSBR bound of 0%o(%u) -- OOSB fault\n", DSBR.bound, DSBR.bound);
            fault_gen(acc_viol_fault);
            return NULL;
        }
        if(1) log_msg(DEBUG_MSG, "APU::append", "Fetching SDW for unpaged descriptor table from 0%o\n", DSBR.addr + 2 * segno);
        if (fetch_abs_pair(DSBR.addr + 2 * segno, &sdw_word0, &sdw_word1) != 0)
            return NULL;
    } else {
        // Descriptor table is paged
        if (segno * 2 >= 16 * (DSBR.bound + 1)) {
            // BUG 12/05/2008 -- bootload_1.alm, instr 17 triggers this
            log_msg(WARN_MSG, "APU::append", "Initial check: Segno outside paged DSBR bound of 0%o(%u) -- OOSB fault\n", DSBR.bound, DSBR.bound);
            cu.word1flags.oosb = 1;         // ERROR: nothing clears
            fault_gen(acc_viol_fault);
            cancel_run(STOP_WARN);
            return NULL;
        }

        // First, the DSPTW fetch cycle (PTWAM doesn't cache the DS?)
        uint y1 = (2 * segno) % page_size;          // offset within page table
        uint x1 = (2 * segno - y1) / page_size;     // offset within DS page
        PTW_t DSPTW;
        t_uint64 word;
        if(1) log_msg(DEBUG_MSG, "APU::append", "Fetching DS-PTW for paged descriptor table from 0%o\n", DSBR.addr + x1);
        if (fetch_abs_word(DSBR.addr + x1, &word) != 0) // We assume DS is 1024 words (bound=077->16*64=1024)
            return NULL;
        decode_PTW(word, &DSPTW);   // TODO: cache this
        if (DSPTW.f == 0) {
            if (opt_debug>0) log_msg(DEBUG_MSG, "APU::append", "DSPTW directed fault\n");
            fault_gen(dir_flt0_fault + DSPTW.fc);   // Directed Faults 0..4 use sequential fault numbers
            return NULL;
        }
        if (! DSPTW.u) {
            // MDSPTW cycle
            DSPTW.u = 1;
            if (set_PTW_used(DSBR.addr + x1) != 0) {
                // impossible -- we just read this absolute addresed word
                return NULL;
            }
        }
        // PSDW cycle (Step 5 for case when Descriptor Segment is paged)
        // log_msg(DEBUG_MSG, "APU::append", "Fetching SDW from 0%o<<6+0%o => 0%o\n", DSPTW.addr, y1, (DSPTW.addr<<6) + y1);
        if (fetch_abs_pair((DSPTW.addr<<6) + y1, &sdw_word0, &sdw_word1) != 0)
            return NULL;
    }

    // Allocate a SDWAM entry
    if (oldest_sdwam == -1) {
        log_msg(ERR_MSG, "APU::append", "SDWAM had no oldest entry\n");
        cancel_run(STOP_BUG);
        return NULL;
    }
    for (int i = 0; i < ARRAY_SIZE(SDWAM); ++i) {
        -- SDWAM[i].assoc.use;
    }
    SDWp = SDWAM + oldest_sdwam;
    decode_SDW(sdw_word0, sdw_word1, &SDWp->sdw);
    SDWp->assoc.ptr = segno;
    SDWp->assoc.use = 15;
    SDWp->assoc.is_full = 1;
    if (opt_debug) {
        log_msg(DEBUG_MSG, "APU::append", "Allocated SDWAM # %o for seg 0%o: addr - 0%o, bound = 0%o(%d), f=%d\n",
            oldest_sdwam, segno, SDWp->sdw.addr, SDWp->sdw.bound, SDWp->sdw.bound, SDWp->sdw.f);
    }

    //log_msg(DEBUG_MSG, "APU::append", "SDW: addr - 0%o, bound = 0%o(%d), f=%d\n",
    //  SDWp->sdw.addr, SDWp->sdw.bound, SDWp->sdw.bound, SDWp->sdw.f);

    return SDWp;
}


static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp, uint *minaddrp, uint *maxaddrp)
{
    // Second part of page_in()
    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...

    // Following done for either paged or unpaged segments
    if (SDWp->sdw.f == 0) {
        if (opt_debug>0) log_msg(DEBUG_MSG, "APU::append", "SDW directed fault\n");
        fault_gen(dir_flt0_fault + SDWp->sdw.fc);   // Directed Faults 0..4 use sequential fault numbers
        return 1;
    }
    uint bound = 16 * (SDWp->sdw.bound + 1);
    if (offset >= bound) {
        cu.word1flags.oosb = 1;         // ERROR: nothing clears
        log_msg(NOTIFY_MSG, "APU::append", "SDW: Offset=0%o(%u), bound = 0%o(%u) -- OOSB fault\n", offset, offset, SDWp->sdw.bound, SDWp->sdw.bound);
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
        static PTWAM_t *PTWp = PTWAM;   // BUG: expose to reset?
        int oldest_ptwam = -1;
        // TODO: performance: cache last index instead and start search from there
        if (PTWp == NULL || PTWp->assoc.ptr != segno || PTWp->assoc.pageno != x2 || ! PTWp->assoc.is_full) {    // todo: validate NULL as impossible
            PTWp = NULL;
            for (int i = 0; i < ARRAY_SIZE(PTWAM); ++i) {
                if (PTWAM[i].assoc.ptr == segno && PTWAM[i].assoc.pageno == x2 && PTWAM[i].assoc.is_full) {
                    PTWp = PTWAM + i;
                    // log_msg(DEBUG_MSG, "APU::append", "Found PTW for (segno 0%o, page 0%o) in PTWAM[%d]\n", segno, x2, i);
                    break;
                }
                if (PTWAM[i].assoc.use == 0)
                    oldest_ptwam = i;
                //if (! PTWAM[i].assoc.is_full) {
                //  log_msg(DEBUG_MSG, "APU::append", "PTW[%d] is not full\n", i);
                //}
            }
        } else {
            // log_msg(DEBUG_MSG, "APU::append", "PTW for (segno 0%o, page 0%o) is the MRU -- in PTWAM[%d]\n", segno, x2, PTWp-PTWAM);
        }
        if (PTWp != NULL) {
            // PTW is in PTWAM; it becomes the LRU
            if (PTWp->assoc.use != 15) {
                for (int i = 0; i < ARRAY_SIZE(PTWAM); ++i) {
                    if (PTWAM[i].assoc.use > PTWp->assoc.use)
                        -- PTWAM[i].assoc.use;
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
            // log_msg(DEBUG_MSG, "APU::append", "Fetching PTW for (segno 0%o, page 0%o) from 0%o(0%o+page)\n", segno, x2, SDWp->sdw.addr + x2, SDWp->sdw.addr);
            if (fetch_abs_word(SDWp->sdw.addr + x2, &word) != 0)
                return 1;
            for (int i = 0; i < ARRAY_SIZE(PTWAM); ++i) {
                -- PTWAM[i].assoc.use;
            }
            PTWp = PTWAM + oldest_ptwam;
            decode_PTW(word, &PTWp->ptw);
            PTWp->assoc.use = 15;
            PTWp->assoc.ptr = segno;
            PTWp->assoc.pageno = x2;
            PTWp->assoc.is_full = 1;
            if (PTWp->ptw.f == 0) {
                if(opt_debug>0) log_msg(DEBUG_MSG, "APU::append", "PTW directed fault\n");
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


static int set_PTW_used(uint addr)
{
    t_uint64 word;
    if (fetch_abs_word(addr, &word) != 0)
        return 1;
    word = setbits36(word, 26, 1, 1);
    return store_abs_word(addr, word);
}

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

char* print_sdw(t_uint64 word0, t_uint64 word1)
{
    static char buf[100];
    SDW_t sdw;
    decode_SDW(word0, word1, &sdw);
    sprintf(buf, "[addr=0%o, r(123)=(0%o,0%o,0%o), f=%d, fc=%o; bound=0%o(%d), r=%d,e=%d,w=%d,p=%d,u=%d,g=%d,c=%d, cl=0%o]",
        sdw.addr, sdw.r1, sdw.r2, sdw.r3, sdw.f, sdw.fc,
        sdw.bound, sdw.bound, sdw.r, sdw.e, sdw.w, sdw.priv, sdw.u, sdw.g, sdw.c, sdw.cl);
    return buf;
}

//=============================================================================
