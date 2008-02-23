/*
    apu.c -- appending unit (APU) -- "address appending"
*/

// Supposedly, appending unit HW controls fault recognition?

#include "hw6180.h"

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 };

typedef struct {    // BUG
    int32 soffset; // Signed copy of CA (15 or 18 bits if from instr; 18 bits if from indir word)
    uint32 tag;
    flag_t more;
    enum atag_tm special;
} ca_temp_t;

static int compute_addr(instr_t *ip, ca_temp_t *ca_tempp);
static int addr_append(t_uint64 *wordp);
static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp);

//=============================================================================

static t_bool is_transfer_op(int op)
{
    return 0;   // BUG
}

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
        debug_msg("APU", "Setting absolute mode.\n";
    } else if (mode == APPEND_mode) {       // BUG: is this correct?
        IR.abs_mode = 0;
        IR.not_bar_mode = 1;
        debug_msg("APU", "Setting append mode.\n";
    } else if (mode == BAR_mode) {
        IR.abs_mode = 0;    // BUG: is this correct?
        IR.not_bar_mode = 0;
        debug_msg("APU", "Setting bar mode.\n";
    } else {
        complain_msg("APU", "Unable to determine address mode.\n");
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
        warn_msg("APU", "Unsure of mode -- seems to be BAR\n");
        cancel_run(STOP_WARN);
        return BAR_mode;
    }

    warn_msg("APU", "Unsure of mode -- seems to be APPEND\n");
    return APPEND_mode;
}

//=============================================================================


int decode_ypair_addr(instr_t* ip, t_uint64* addrp)
{
    // returns non-zero if fault in groups 1-6 detected
    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        complain_msg("APU", "Only ABS mode implemented.\n");
        cancel_run(STOP_BUG);
    }

    // NOTE: We assume that the no address modification is expected; the
    // supplied address is an 18bit absolute memory address.

    t_uint64 addr = ip->addr;
    if (addr % 2 == 1)
        -- addr;
    *addrp = addr;

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
    } else {
        if (ip->pr_bit == 0) {
            int32 offset = ip->addr;
            int32 soffset = sign18(ip->addr);
            sprintf(buf, "%s, offset 0%06o(%+d), inhibit %u, pr=N, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                offset, soffset, 
                ip->inhibit, ip->tag, ip->tag >> 4, ip->tag & 017);
        } else {
            uint pr = ip->addr >> 15;
            int32 offset = ip->addr & MASKBITS(15);
            int32 soffset = sign15(ip->addr);
            sprintf(buf, "%s, PR %d, offset 0%06o(%+d), inhibit %u, pr=Y, tag 0%03o(Tm=%u,Td=0%02o)",
                opname, 
                pr, offset, soffset, 
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

//=============================================================================

int addr_mod(instr_t *ip)
{
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

    TPR.CA = ip->addr;
    TPR.is_value = 0;   // BUG: Use "direct operand flag" instead
    TPR.value = 0xdeadbeef;

    // Addr appending below

    addr_modes_t orig_mode = get_addr_mode();
    addr_modes_t addr_mode = get_addr_mode();
    int ptr_reg_flag = ip->pr_bit;
    ca_temp_t ca_temp;  // BUG: hack

    // BUG: The following check should only be done after a sequential
    // instr fetch, not after a transfer!  We're only called by do_op(),
    // so this criteria is *almost* met.   Need to detect transfers.

    if (ptr_reg_flag == 0) {
        ca_temp.soffset = sign18(ip->addr);
    } else {
        set_addr_mode(addr_mode = APPEND_mode);
        // AL39: Page 341, Figure 6-7
        int32 offset = ip->addr & MASKBITS(15);
        ca_temp.soffset = sign15(offset);
        uint pr = ip->addr >> 15;
        TPR.TSR = AR_PR[pr].PR.snr;
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR);
        TPR.CA = AR_PR[pr].wordno + ca_temp.soffset;
        TPR.bitno = AR_PR[pr].PR.bitno;

        // BUG: Enter append mode & stay if execute a transfer
    }

    ca_temp.tag = ip->tag;


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
            debug_msg("APU", "Final (incomplete) CA: 0%0Lo\n", TPR.CA);
            return 1;
        }
        if (ca_temp.more)
            mult = 1;
        ca_temp.soffset = sign18(TPR.CA);
        if (ca_temp.more)
            debug_msg("APU", "Pre Seg: Continuing indirect fetches\n");
        if (do_esn_segmentation(ip, &ca_temp) != 0) {
            debug_msg("APU", "Final (incomplete) CA: 0%0Lo\n", TPR.CA);
            return 1;
        }
        if (ca_temp.more)
            debug_msg("APU", "Post Seg: Continuing indirect fetches\n");
        if (ca_temp.more)
            mult = 1;
    }
    if (mult) {
            debug_msg("APU", "Final CA: 0%0Lo\n", TPR.CA);
    }

    addr_mode = get_addr_mode();    // may have changed

    if (addr_mode == BAR_mode) {
        if (addr_mode == BAR_mode && ptr_reg_flag == 0) {
            // Todo: Add CA to BAR.base; add in PR; check CA vs bound
        }
        // BUG: Section 4 says make sure CA cycle handled AR reg mode & constants
        complain_msg("APU", "BAR mode not implemented.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    // Address appending does not occur in absolute mode unless bit 29
    // (aka ptr_reg_flag) is on or the its or itp modifiers appear in
    // an indirect word

    // BUG: detect its and itp -- DONE
    if (addr_mode == ABSOLUTE_mode && ptr_reg_flag == 0) {
        // TPR.CA is the 18-bit absolute main memory addr
        return 0;
    }

    // BUG: Do segment handling ala AL39 section 6 -- DONE

    warn_msg("APU", "addr_mod: None ABS mode may be incomplete.\n");
    cancel_run(STOP_WARN);



#if 0 // MOSTLY DONE?
    // From Section 4:
    if (addr_mode == BAR_mode && ptr_reg_flag == 0)
        // 18bit offset rel to the BAR
    if (addr_mode == APPEND_mode && ptr_reg_flag == 0)
        // 18bit offset rel to base of the current proc segment

    if ((addr_mode == ABSOLUTE_mode || addr_mode == APPEND_mode) && ptr_reg_flag == 1)
        3 bit ptr reg number n & 15bit offset rel to C(PRn.wordno)
    if (appropriate instr && ptr_reg_flag == 1)
        3 bit addr reg number n & 15bit offset rel to C(ARn)
    if (appropriate instr)
        18bit literal
    if (appropriate instr)
        8bit shift op count
    if (appropriate instr)
        18bit offset rel to current val of instr counter
#endif

    return 0;
}


static int compute_addr(instr_t *ip, ca_temp_t *ca_tempp)
{
    // Perform a "CA" cycle as per figure 6-2 of AL39.
    // Generate an 18-bit computed address (in TPR.CA) as specified in section 6
    // of AL39.
    // In our version, this may include replacing TPR.CA with a 36 bit constant or
    // other value if an appropriate modifier (e.g., du) is present.

    ca_tempp->more = 0;

    // BUG: Need to do ESN special handling if loop is continued

    // for (;;) {

        // uint tm = (ca_tempp->tag >> 4) & 03; // the and is a hint to the compiler for the following switch...
        enum atag_tm tm = (ca_tempp->tag >> 4) & 03;    // the and is a hint to the compiler for the following switch...

        uint td = ca_tempp->tag & 017;

        ca_tempp->special = tm;
    
        switch(tm) {
            case atag_r: {  // Tm=0 -- register (r)
                if (td == 0)
                    return 0;
                int off = ca_tempp->soffset;
                switch(td) {
                    case 0:
                        break;  // no mod (can't get here anyway)
                    case 1: // ,au
                        TPR.CA = off + sign18(getbits36(reg_A, 0, 18));
                        break;
                    case 2: // ,qu
                        TPR.CA = off + sign18(getbits36(reg_Q, 0, 18));
                        break;
                    case 3: // ,du
                        TPR.is_value = 1;   // BUG: Use "direct operand flag" instead
                        TPR.value = (t_uint64) TPR.CA << 18;
                        debug_msg("APU", "Mod du: Value from offset 0%Lo is 0%Lo\n", TPR.CA, TPR.value);
                        break;
                    case 4: // PPR.IC
                        TPR.CA = off + PPR.IC;  // BUG: IC assumed to be unsigned
                        break;
                    case 5:
                        TPR.CA = off + sign18(getbits36(reg_A, 18, 18));
                        break;
                    case 6:
                        TPR.CA = off + sign18(getbits36(reg_Q, 18, 18));
                        break;
                    case 7: // ,dl
                        TPR.is_value = 1;   // BUG: Use "direct operand flag" instead
                        TPR.value = TPR.CA; // BUG: Should we sign?
                        debug_msg("APU", "Mod dl: Value from offset 0%Lo is 0%Lo\n", TPR.CA, TPR.value);
                        break;
                    case 010:
                    case 011:
                    case 012:
                    case 013:
                    case 014:
                    case 015:
                    case 016:
                    case 017:
                        TPR.CA = off + sign18(reg_X[td&07]);
                        // if (td&7 == 5)
                            debug_msg("APU", "Tm=0%o,Td=%02o: offset 0%o + X[%d]=0%o(%+d decimal)==>0%o(+%d) yields 0%Lo (%+Ld decimal)\n",
                                tm, td, off, td&7, reg_X[td&7], reg_X[td&7], sign18(reg_X[td&7]), sign18(reg_X[td&7]), TPR.CA, TPR.CA);
                        break;
                }
                return 0;
            }
            case atag_ri: { // Tm=1 -- register then indirect (ri)
                if (td == 3 || td == 7) {
                    warn_msg("APU", "RI with td==0%o is illegal.\n", td);
                    fault_gen(illproc_fault);   // need illmod sub-category
                    return 1;
                }
                if (td == 0) {
                    debug_msg("APU", "IR: pre-fetch:  TPR.CA=0%Lo (no register)\n", TPR.CA);
                } else {
                    t_uint64 ca = TPR.CA;
                    TPR.CA += reg_X[td];    // BUG: do we need to treat as signed?
                    debug_msg("APU", "IR: pre-fetch:  TPR.CA=0%Lo <==  TPR.CA=%Lo + X[%d]=0%o\n",
                        TPR.CA, reg_X[td], ca);
                }
                t_uint64 word;
                if (addr_append(&word) != 0)
                    return 1;
                debug_msg("APU", "IR: fetch:  word at TPR.CA=0%Lo is 0%Lo\n",
                    TPR.CA, word);
                TPR.CA = word >> 18;
                ca_tempp->tag = word & MASKBITS(6);
                debug_msg("APU", "IR: post-fetch: TPR.CA=0%Lo, tag=0%o\n", TPR.CA, ca_tempp->tag);
                cancel_run(STOP_IBKPT); // compare to prior runs
                // break;   // Continue a new CA cycle
                ca_tempp->more = 1;     // Continue a new CA cycle
                // BUG: flowchart says start CA, but we do ESN
                return 0;
            }
            case atag_it: { // Tm=2 -- indirect then tally (it)
                // BUG: see "it" flowchart for looping (Td={15,17}
                switch(td) {
                    case 0:
                        warn_msg("APU", "IT with Td zero not valid in instr word.\n");
                        fault_gen(f1_fault);    // This mode not ok in instr word
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
                            ret = store_word(iloc, iword);
                        }
                        return ret;
                    }
                    // case 015: more=1 depending upon tag
                    case 016: {
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
                            IR.tally_runout = (tally == 0); // BUG: do we need to fault?
                            ++addr;
                            addr &= MASK18; // wrap from 2^18-1 to zero
                            iword = setbits36(iword, 0, 18, addr);
                            iword = setbits36(iword, 18, 12, tally);
                            ret = store_word(iloc, iword);
                        }
                        return ret;
                    }
                    default:
                        complain_msg("APU", "IT with Td 0%o not implmented.\n", td);
                        cancel_run(STOP_BUG);
                        return 1;
                }
                break;
            }
            case atag_ir: { // TM=3 -- indirect then register (ir)
                complain_msg("APU", "IR addr mod not implemented.\n");
                // BUG: Maybe handle special tag (41 itp, 43 its).  Or post handle?
                cancel_run(STOP_BUG);
                return 1;
            }
        }
        // BUG: Need to do ESN special handling if loop is continued
    //}

    return 0;
}


//=============================================================================

static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp)
{
    // Implements the portion of AL39 figure 6-10 that is below the "CA CYCLE" box

    if (ca_tempp->special && (TPR.CA % 2) == 0) {
        // Just did an "ir" or "ri" addr modification
        if (ca_tempp->tag == 041) {
            // itp
            set_addr_mode(APPEND_mode);
            t_uint64 word1, word2;
            // BUG: are we supposed to fetch?
            int ret = fetch_pair(TPR.CA, &word1, &word2);
            if (ret != 0)
                return ret;
            uint n = getbits36(word1, 0, 3);
            TPR.TSR = AR_PR[n].PR.snr;
            if (! SDWAM[TPR.TSR].enabled) {
                warn_msg("APU", "SDWAM[%d] is not enabled\n", TPR.TSR);
                cancel_run(STOP_WARN);
            }
            uint sdw_r1 = SDWAM[TPR.TSR].r1;
            TPR.TRR = max3(AR_PR[n].PR.rnr, sdw_r1, TPR.TRR);
            TPR.TBR = getbits36(word2, 21, 6);
            ca_tempp->tag = word2 & MASKBITS(6);
            uint i_mod_tm = ca_tempp->tag >> 4;
            uint r;
            if (ca_tempp->special == atag_ir)
                r = cu.CT_HOLD;
            else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
                uint i_mod_td = ca_tempp->tag & MASKBITS(4);
                r = i_mod_td;
            } else {
                complain_msg("APU", "ITP addr mod with undefined r-value (tm=0%o,new-tm=0%o)\n", ca_tempp->special, i_mod_tm);
                cancel_run(STOP_BUG);
                r = 0;
            }
            uint i_wordno = getbits36(word2, 0, 18);
            TPR.CA = AR_PR[n].wordno + i_wordno + r;
            ca_tempp->more = 1;
            complain_msg("APU", "ITP not fully understood?\n");
            cancel_run(STOP_BUG);
            return 0;
        } else if (ca_tempp->tag == 043) {
            // its
            set_addr_mode(APPEND_mode);
            t_uint64 word1, word2;
            // BUG: are we supposed to fetch?
            debug_msg("APU", "ITS: CA initially 0%o\n", TPR.CA);
            int ret = fetch_pair(TPR.CA, &word1, &word2);
            if (ret != 0)
                return ret;
            TPR.TSR =  getbits36(word1, 3, 15);
            uint its_rn = getbits36(word1, 18, 3);
            if (! SDWAM[TPR.TSR].enabled) {
                warn_msg("APU", "SDWAM[%d] is not enabled\n", TPR.TSR);
                cancel_run(STOP_WARN);
            }
            uint sdw_r1 = SDWAM[TPR.TSR].r1;
            TPR.TRR = max3(its_rn, sdw_r1, TPR.TRR);
            TPR.TBR = getbits36(word2, 21, 6);
            ca_tempp->tag = word2 & MASKBITS(6);

            uint i_mod_tm = ca_tempp->tag >> 4;
            debug_msg("APU", "ITS: TPR.TSR = 0%o, rn=0%o, sdw.r1=0%o, TPR.TRR=0%o, TPR.TBR=0%o, tag=0%o(tm=0%o)\n",
                TPR.TSR, its_rn, sdw_r1, TPR.TRR, TPR.TBR, ca_tempp->tag, i_mod_tm);
            uint r;
            if (ca_tempp->special == atag_ir)
                r = cu.CT_HOLD;
            else if (ca_tempp->special == atag_ri && (i_mod_tm == atag_r || i_mod_tm == atag_ri)) {
                uint i_mod_td = ca_tempp->tag & MASKBITS(4);
                r = i_mod_td;
            } else {
                complain_msg("APU", "ITS addr mod with undefined r-value (tm=0%o,new-tm=0%o)\n", ca_tempp->special, i_mod_tm);
                cancel_run(STOP_BUG);
                r = 0;
            }
            uint i_wordno = getbits36(word2, 0, 18);
            TPR.CA = i_wordno + r;
            debug_msg("APU", "ITS: CA = wordno=0%o + r=0%o => 0%o\n", i_wordno, r, TPR.CA);
            ca_tempp->more = 1;
            complain_msg("APU", "ITS not fully understood?\n");
            cancel_run(STOP_BUG);
            return 0;
        }
    }

    // If we need an indirect word, we should return to the top of the ESN flow (AL39, figure 6-8)
    if (ca_tempp->more)
        return 0;

    if (ip->opcode == (opcode0_rtcd << 1)) {
        complain_msg("APU", "RTCD operand not implemented.\n");
        cancel_run(STOP_BUG);
    } else if (ip->opcode == (opcode0_call6 << 1) || is_transfer_op(ip->opcode)) {
        complain_msg("APU", "Call6 and transfer operands not implemented.\n");
        cancel_run(STOP_BUG);
    } else {
        // BUG: What does the question "Appending unit data movement?" mean?
    }
    return 0;
}

//=============================================================================

static int addr_append(t_uint64 *wordp)
{
    // Implements AL39, figure 5-4
    // NOTE: ri mode is expecting a fetch

    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode == ABSOLUTE_mode)
        return fetch_word(TPR.CA, wordp);
    complain_msg("APU", "addr_append: Only ABS mode implemented.\n");
    cancel_run(STOP_BUG);
    return 0;
}

//=============================================================================

#if 0
addr_modes_t addr_mode = examine indicator register (IR)

opcode = bits 18..27 (10 bits)
if (basic or eis single word)
    addr = bits 0..17 (first 18 bits)
    int_inhibit = bit 28 // aka I
    ptr_reg_flag = bit 29 // aka A
    tag = bits 30..36
else // eis multi-word
    variable = bits 0..17 (first 18 bits)
    int_inhibit = bit 28 // aka I
    mf1 = bits 29..36   // aka modification field

sub get_addr (addr, mods)
    // analyze 18bit computed address TPR.CA
    if absolute mode
        if instr-fetch
            bypass append unit
        else if certian mods
            use append unit
        else
            bypass append unit
    else if append mode
            use append unit
    else error? // are we always in eithher abs or append?
    
    if bypassing append unit
        return addr as the absolute main mem addr

    // append unit processing
    see section 6 -- formation of a virt mem addr
        we have effective seg num segno and a computed
        addr (offset) in TPR.SNR and TPR.CA
    check segment boundries
    see fig 5-4 flowchart


section 6 -- virt addr formation

during instr decode for single-word instrs
    set TPR.CA = addr field of instr // computed address

    if form one of the two forms (maybe when bit 29 zero)
        // first type -- no explicit segment numbers
        // produces only C(TPR.CA); C(TPR.TSR) is unchanged
        check tag
            0: register (r)
            1: register then indirect (ri)
            2: indirect then tally (it)
            3: indirect then register (r)
    else
        // second type -- use a segment number in an indirect
        // word pair or in a ptr register

        // do prep
        // usually perform same algorithm as first type, but:
        // check for [...] special address modifier
    
    
#endif
