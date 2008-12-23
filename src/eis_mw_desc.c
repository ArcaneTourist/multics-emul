/*
    eis_mw_desc.c
    Routines related to descriptors for EIS multi-word instructions.
    Descriptors describe operand locations and looping controls used
    by many EIS multi-word instructions.
    This code was originally in apu.c because the descriptors are used
    for addressing.
*/

// Supposedly, appending unit HW controls fault recognition?

#include "hw6180.h"

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 }; // BUG: move to hdr

//static int get_via_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int index, uint *nib);
static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib, flag_t advance);
static void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits);
static int get_mf_an_addr(const eis_mf_t* mfp, uint y, int nbits, uint *addrp, int* bitnop, uint *minaddrp, uint *maxaddrp);
static int put_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib, flag_t advance);

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

//=============================================================================


eis_mf_t* parse_mf(uint mf, eis_mf_t* mfp)
{
    // mf &= MASKBITS(7); -- caller's responsibility

    mfp->ar = mf >> 6;
    mfp->rl = (mf >> 5) & 1;
    mfp->id = (mf >> 4) & 1;
    mfp->reg = mf & MASKBITS(4);
    return mfp;
}

//=============================================================================

int get_eis_indir_addr(t_uint64 word, uint* addrp)
{
    // Could be called by other EIS routines when an MF specifes that an indirect pointer to
    // and operand descriptor should be expected. -- But isn't.
    // Only called by OPU for those EIS multi-word instructions that use an indirect
    // pointer to specify the address of an operand word (for example scm).
    const char *moi = "APU::EIS-indir-ptr";

    uint y = word >> 18;
    uint indir = getbits36(word, 29, 1);    // "A" flag
    uint td = word & MASKBITS(4);

    // Debugging
    if (indir) {
        // pointer register
        uint pr = y >> 15;
        int32 offset = y & MASKBITS(15);
        int32 soffset = sign15(offset);
        log_msg(NOTIFY_MSG, moi, "Indir word %012Lo: pr=%#o, offset=%#o(%d); REG(Td)=%#o\n", word, pr, offset, soffset, td); 
    } else {
        // use 18 bit addr
        log_msg(NOTIFY_MSG, moi, "Indir word %012Lo: addr=%#o(%d); REG(Td)=%#o\n", word, y, sign18(y), td); 
    }

    uint bitno;
    uint minaddr, maxaddr;  // unneeded

    log_msg(NOTIFY_MSG, moi, "Calling get-address.\n");
    int ret = get_address(y, indir, 0, td, 36, addrp, &bitno, &minaddr, &maxaddr);
    if (ret == 0) {
        log_msg(NOTIFY_MSG, moi, "Resulting addr is %#o\n", *addrp);
        if (bitno != 0) {
            log_msg(ERR_MSG, moi, "Resulting addr (%#o) includes a bit offset of %#o(%+d) that cannot be returned.\n", *addrp, bitno, bitno);
            cancel_run(STOP_BUG);
        } else {
            // log_msg(WARN_MSG, moi, "Auto breakpoint\n");
            // cancel_run(STOP_IBKPT);
        }
    } else {
        log_msg(NOTIFY_MSG, moi, "Call to get-address(y=%#o,ar=%d,reg=%d,nbits=36,...) returns non-zero.\n", y, indir, td);
        cancel_run(STOP_WARN);
    }
    return ret;
}


static int fetch_mf_op(uint opnum, const eis_mf_t* mfp, t_uint64* wordp)
{
    // Fetch an EIS operand via a MF

    // uint addr = PPR.IC;
    if (fetch_word(PPR.IC + opnum, wordp) != 0)
        return 1;
    if (mfp != NULL && mfp->id) {
        // don't implement until we see an example
        log_msg(ERR_MSG, "APU", "Fetch EIS multi-word op: MF with indir bit set is unsupported:\n");
        uint addr = *wordp >> 18;
        uint a = getbits36(*wordp, 29, 1);
        uint td = *wordp & MASKBITS(4);
        // addr_modes_t addr_mode = get_addr_mode();
        if (a) {
            // indir via pointer register
            uint pr = addr >> 15;
            int32 offset = addr & MASKBITS(15);
            int32 soffset = sign15(offset);
            // PR[pr]
            // generalize once we have an example.  Call get_mf_an_addr()
            log_msg(ERR_MSG, "APU", "Indir word %012Lo: pr=%#o, offset=%#o(%d); REG(Td)=%#o\n", *wordp, pr, offset, soffset, td); 
        } else {
            // use 18 bit addr in all words -- fetch_word will handle
            log_msg(ERR_MSG, "APU", "Indir word %012Lo: offset=%#o(%d); REG(Td)=%#o\n", *wordp, addr, sign18(addr), td); 
        }
        // enum atag_tm tm = atag_r;
        // TPR.CA = 0;
        // reg_mod(td, 0);
        cancel_run(STOP_BUG);
    }
    return 0;
}


int fetch_mf_ops(const eis_mf_t* mf1p, t_uint64* word1p, const eis_mf_t* mf2p, t_uint64* word2p, const eis_mf_t* mf3p, t_uint64* word3p)
{
    // Read in the operands that follow an EIS multi-word instruction
    // Note that interpretation is opcode dependent -- we're just doing the two 
    // or three (possibly indirect) fetches here

    if (fetch_mf_op(1, mf1p, word1p) != 0)
        return 1;

    if (word2p != NULL)
        if (fetch_mf_op(2, mf2p, word2p) != 0)
            return 1;
    if (word3p != NULL)
        if (fetch_mf_op(3, mf3p, word3p) != 0)
            return 1;

    return 0;
}

//=============================================================================

const char* mf2text(const eis_mf_t* mfp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];
    sprintf(bufp, "{ar=%d, rl=%d, id=%d, reg=%#o}", mfp->ar, mfp->rl, mfp->id, mfp->reg);
    return bufp;
}

const char* eis_alpha_desc_to_text(const eis_mf_t* mfp, const eis_alpha_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    if (mfp == NULL)
        sprintf(bufp, "{y=%#o, char-no=%#o, ta=%o, n=%#o(%d)}",
            descp->address, descp->cn, descp->ta, descp->n, descp->n);
    else {
        char reg[40];
        if (mfp->rl) {
            char mod[30];
            mod2text(mod, 0, descp->n_orig);
            sprintf(reg, "%s=>%#o(%d)", mod, descp->n, descp->n);
        } else
            sprintf(reg, "%#o(%d)", descp->n, descp->n);

        if (mfp->ar) {
            uint pr = descp->address >> 15;
            uint offset = descp->address & MASKBITS(15);
            sprintf(bufp, "{y=%#o => PR[%d]|%#o, char-no=%#o, ta=%o, n=%s}",
                descp->address, pr, offset, descp->cn, descp->ta, reg);
        } else
            sprintf(bufp, "{y=%#o, char-no=%#o, ta=%o, n=%s}",
                descp->address, descp->cn, descp->ta, reg);
    }
    return bufp;
}

const char* eis_bit_desc_to_text(const eis_bit_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    sprintf(bufp, "{y=%#o, char-no=%#o, bit-no=%d n=%#o(%d)}",
        descp->address, descp->cn/9, descp->bitno, descp->n, descp->n);
    return bufp;
}

//=============================================================================


void parse_eis_alpha_desc(t_uint64 word, const eis_mf_t* mfp, eis_alpha_desc_t* descp)
{
    // Return absolute address for EIS operand in Alphanumeric Data Descriptor Format
    descp->address = getbits36(word, 0, 18);    // we'll check for PR usage later

    descp->cn = getbits36(word, 18, 3);
    descp->bitno = 0;
    descp->ta = getbits36(word, 21, 2); // data type
    descp->n_orig = getbits36(word, 24, 12);
    descp->n = descp->n_orig;
    descp->nbits = (descp->ta == 0) ? 9 : (descp->ta == 1) ? 6 : (descp->ta == 2) ? 4 : -1;
    if (descp->nbits == -1) {
        log_msg(ERR_MSG, "APU::EIS", "Illegal ta value in MF\n");
        fault_gen(illproc_fault);
        cancel_run(STOP_BUG);
    }

int n = descp->n;
    fix_mf_len(&descp->n, mfp, descp->nbits);
#if 0
    if ((descp->n >> 12) != 0) {
        // BUG: trim length to 12 bits even for register lookups
        //log_msg(WARN_MSG, "APU", "parse_eis_alpha_desc: desc: n orig %#o; After mod %#o => %#o\n", n, descp->n, descp->n & MASKBITS(12));
        //descp->n &= MASKBITS(12);
        // BUG FIX follows
        log_msg(NOTIFY_MSG, "APU", "parse_eis_alpha_desc: desc: n orig %#o; After mod %#o; prior bug would have truncated to %#o\n", n, descp->n, descp->n & MASKBITS(12));
    }
#endif
    if (descp->nbits == 9) {
        if ((descp->cn & 1) != 0) {
            log_msg(ERR_MSG, "APU::EIS", "TA of 0 (9bit) not legal with cn of %d\n", descp->cn);
            fault_gen(illproc_fault);
        } else
            descp->cn /= 2;
    }
    if (descp->nbits * descp->cn >= 36) {
        log_msg(ERR_MSG, "APU::EIS", "Data type TA of %d (%d bits) does not allow char pos of %d\n", descp->ta, descp->nbits, descp->cn);
        fault_gen(illproc_fault);
    }

    descp->first = 1;

    descp->area.addr = 0123;
    descp->area.bitpos = 0;
    // Set the "current" addr and bitpos in case we need to display them
    descp->curr.addr = 0123;
    descp->curr.bitpos = 0;
}

//=============================================================================

void parse_eis_bit_desc(t_uint64 word, const eis_mf_t* mfp, eis_bit_desc_t* descp)
{
    // Return absolute address for EIS operand in Bit String Operator Descriptor Format
    descp->address = getbits36(word, 0, 18);    // we'll check for PR usage later

    descp->cn = getbits36(word, 18, 2) * 9; // scale by 9 for alphanumeric functions that expect this to be in units of nbits
    descp->bitno = getbits36(word, 20, 4);
    if (descp->bitno > 8) {
        log_msg(ERR_MSG, "EIS", "Bit String Operand String has illegal bitno.\n");
        cancel_run(STOP_BUG);
    }
    descp->ta = 9;
    descp->n_orig = getbits36(word, 24, 12);
    descp->n = descp->n_orig;
    descp->nbits = 1;

    fix_mf_len(&descp->n, mfp, descp->nbits);

    descp->first = 1;

    descp->area.addr = 0123;
    descp->area.bitpos = 0;
    // Set the "current" addr and bitpos in case we need to display them
    descp->curr.addr = 0123;
    descp->curr.bitpos = 0;
}

//=============================================================================

static void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits)
{
    if (mfp->rl) {
        switch(*np) {
            // case 0: // illegal
             case 1:    // au
                *np = reg_A >> 18;  // AL39 shows extended range for "a", but not "au"
                return;
            case 05: // a
                if (nbits == 1)
                    *np = reg_A & MASKBITS(24);
                else if (nbits == 4)
                    *np = reg_A & MASKBITS(22);
                else if (nbits == 6)
                    *np = reg_A & MASKBITS(21);
                else if (nbits == 9)
                    *np = reg_A & MASKBITS(21);
                else {
                    // impossible
                    *np = reg_A;
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d, reg modifier 05: A=%#Lo => %#o\n", nbits, reg_A, *np);
                    cancel_run(STOP_IBKPT);
                }
                return;
            case 06: // q
                if (nbits == 1)
                    *np = reg_Q & MASKBITS(24);
                else if (nbits == 4)
                    *np = reg_A & MASKBITS(22);
                else if (nbits == 6)
                    *np = reg_A & MASKBITS(21);
                else if (nbits == 9)
                    *np = reg_Q & MASKBITS(21);
                else {
                    // impossible
                    *np = reg_Q;
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d, reg modifier 06: A=%#Lo => %#o\n", nbits, reg_A, *np);
                    cancel_run(STOP_IBKPT);
                }
                return;
            case 010:
            case 011:
            case 012:
            case 013:
            case 014:
            case 015:
            case 016:
            case 017:
                *np = reg_X[*np&7];
                return;
            default:
                log_msg(ERR_MSG, "APU", "MF len: reg modifier %#o not implemented.\n", *np);
                cancel_run(STOP_BUG);
        }
    }
}

//=============================================================================

static int get_mf_an_addr(const eis_mf_t* mfp, uint y, int nbits, uint *addrp, int* bitnop, uint *minaddrp, uint *maxaddrp)
{
    // Return absolute address for EIS operand in Alphanumeric Data Descriptor Format
    // BUG: some callers may keep results.  This isn't valid for multi-page segments.
    //
    const char *moi = "eis::mf";

    log_msg(DEBUG_MSG, moi, "Calling get-address.\n");
    int ret = get_address(y, 0, mfp->ar, mfp->reg, nbits, addrp, (uint *) bitnop, minaddrp, maxaddrp);
    if (ret) {
        log_msg(NOTIFY_MSG, moi, "Call to get-address(y=%#o,ar=%d,reg=%d,nbits=%d,...) returns non-zero.\n", y, mfp->ar, mfp->reg, nbits);
    }
    if (opt_debug) {
        if (mfp->ar)
            log_msg(DEBUG_MSG, moi, "Using PR results in address %06o\n", *addrp);
        if (*bitnop != 0)
            log_msg(DEBUG_MSG, moi, "Initial bit offset is %d\n", *bitnop);
    }
    return ret;
}

//=============================================================================

// Regarding all of the eis_an routines
    // TODO, flag_t advance:
    // Would be efficient for VM system to expose the bounds of the page we're
    // addressing.  That would avoid using the VM for every single word.
    // Could have page-in always set a global.   The fetch/store y-block
    // routines could also benefit.
    
    // BUG: We get a base addr and assume everything we want is paged in

//=============================================================================


static int get_eis_an_base(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{
    // This function is called just prior to the first usage of a
    // descriptor.  Translates the descriptor's address field into an
    // absolute memory address.  Sets the area and curr member structs of
    // the descriptor.

    const char* moi = "APU::eis-an-addr(base)";

    if (get_mf_an_addr(mfp, descp->address, descp->nbits, &descp->area.addr, &descp->area.bitpos, &descp->area.min_addr, &descp->area.max_addr) != 0) {
        log_msg(WARN_MSG, moi, "Failed: get_mf_an_addr\n");
        return 1;
    }

    if (opt_debug) {
        if (descp->area.bitpos == 0)
            log_msg(DEBUG_MSG, moi, "Base address is %#o (with no bit offset); range is %#o .. %#o\n", descp->area.addr, descp->area.min_addr, descp->area.max_addr);
        else
            log_msg(DEBUG_MSG, moi, "Base address is %#o with bit offset %d; range is %#o .. %#o\n", descp->area.addr, descp->area.bitpos, descp->area.min_addr, descp->area.max_addr);
    }

    descp->curr.addr = descp->area.addr;
    descp->curr.bitpos = descp->area.bitpos;

    if (descp->cn != 0 || descp->bitno != 0) {
        // Descriptor initially points to a non-zero bit offset
        descp->curr.bitpos += descp->cn * descp->nbits + descp->bitno;
        if (descp->curr.bitpos >= 36) {
            log_msg(NOTIFY_MSG, moi, "Too many offset bits for a single word.  Base address is %#o with bit offset %d; CN offset is %d (%d bits).  Advancing to next word.\n", descp->area.addr, descp->area.bitpos, descp->cn, descp->cn * descp->nbits);
            // cancel_run(STOP_WARN);
            descp->curr.addr += descp->curr.bitpos / 36;
            if (descp->curr.addr > descp->area.max_addr) {
                // BUG: need to advance to next page
                log_msg(ERR_MSG, moi, "Too many bits for last word of page.\n");
                cancel_run(STOP_BUG);
            }
            descp->curr.bitpos %= 36;
        }
        if (opt_debug>0) {
            if (descp->bitno == 0)
                log_msg(DEBUG_MSG, moi, "Cn is %d, so initial bitpos becomes %d\n", descp->cn, descp->curr.bitpos);
            else
                log_msg(DEBUG_MSG, moi, "Cn is %d and bitno is %d, so initial bitpos becomes %d\n", descp->cn, descp->bitno, descp->curr.bitpos);
        }
    }

    return 0;
}


// ----------------------------------------------------------------------------


static int get_eis_an_next_page(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{
    // Called when a descriptor hits either bound of a paged
    // segment.

    const char* moi = "APU::eis-an-addr(next-page)";

    int offset = descp->curr.addr - descp->area.addr;   // possibly negative
    uint addr = descp->address + offset;

    log_msg(DEBUG_MSG, moi, "Determining next address.  Offset from prior translation is %04o(%+d)\n", offset, offset);

    if (get_mf_an_addr(mfp, addr, descp->nbits, &descp->area.addr, &descp->area.bitpos, &descp->area.min_addr, &descp->area.max_addr) != 0) {
        log_msg(WARN_MSG, moi, "Failed: get_mf_an_addr\n");
        return 1;
    }

    if (offset < 0) {
        if (descp->area.addr != descp->area.max_addr) {
            log_msg(WARN_MSG, moi, "Next address is not at end of page: next = %06o, min = %06o, max = %06o\n", descp->area.addr, descp->area.min_addr, descp->area.max_addr);
            cancel_run(STOP_WARN);
        }
    } else
        if (descp->area.addr != descp->area.min_addr) {
            log_msg(WARN_MSG, moi, "Next address is not at start of page: next = %06o, min = %06o, max = %06o\n", descp->area.addr, descp->area.min_addr, descp->area.max_addr);
            cancel_run(STOP_WARN);
        }

    if (opt_debug) log_msg(DEBUG_MSG, moi, "Next page is %06o .. %06o\n", descp->area.min_addr, descp->area.max_addr);

    descp->curr.addr = descp->area.addr;

    return 0;
}


//=============================================================================


int get_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 


    //log_msg(WARN_MSG, "APU::eis-an-get", "Starting at IC %#o\n", PPR.IC);
    //return get_via_eis_an(mfp, descp, -1, nib);
    return get_eis_an_fwd(mfp, descp, nib, 1);
}


static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib, flag_t advance)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get-f";

    int saved_debug = opt_debug;
#if 0
    if (descp->curr.addr >= descp->area.max_addr || descp->first)
        if (opt_debug == 0) ++ opt_debug;
#endif

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        opt_debug = saved_debug;
        return 1;
    }

    uint need_fetch = 0;
    if (descp->first) {
        if (get_eis_an_base(mfp, descp) != 0)
            return 1;
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "First char is at addr %#o, bit offset %d\n", descp->curr.addr, descp->curr.bitpos);
        need_fetch = 1;
    } else {
        need_fetch = descp->curr.bitpos == 36;
        if (need_fetch)
            descp->curr.bitpos = 0;
    }

    if (need_fetch) {
        // fetch a new src word
        // log_msg(DEBUG_MSG, moi, "Fetching src word.\n");
        
        // NOTE: Could move this test to else portion of !first test
        // above -- if we're sure that the addr>max_addr test will
        // never initially fail.
        if (descp->curr.addr > descp->area.max_addr) {
            if (opt_debug) log_msg(DEBUG_MSG, moi, "Address %o is past segment or page range of %#o .. %#o\n", descp->curr.addr, descp->area.min_addr, descp->area.max_addr);
            if (get_eis_an_next_page(mfp, descp) != 0) {
                log_msg(ERR_MSG, moi, "Cannot access next page\n");
                cancel_run(STOP_BUG);
                opt_debug = saved_debug;
                return 1;
            }
        }
        if (fetch_abs_word(descp->curr.addr, &descp->word) != 0) {
            log_msg(WARN_MSG, moi, "Failed: fetch word %#o\n", descp->curr.addr);
            opt_debug = saved_debug;
            return 1;
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at %#o %012Lo\n", descp->curr.addr, descp->word);

        if (advance)
            ++ descp->curr.addr;
        else
            if (!descp->first) {
                log_msg(DEBUG_MSG, moi, "impossible\n");
                cancel_run(STOP_BUG);
            }
        if (descp->first)
            descp->first = 0;
    }

    *nib = getbits36(descp->word, descp->curr.bitpos, descp->nbits);
    //if (opt_debug)
    //  log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012Lo, value is %#o\n", descp->nbits, descp->bitpos, descp->word, *nib);
    if (advance) {
        -- descp->n;
        descp->curr.bitpos += descp->nbits;
    }
    opt_debug = saved_debug;
    return 0;
}

int get_eis_an_rev(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get-r";

    int saved_debug = opt_debug;
#if 0
    if (descp->curr.addr >= descp->area.max_addr || descp->first)
        if (opt_debug == 0) ++ opt_debug;
#endif

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n <= 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        opt_debug = saved_debug;
        return 1;
    }

    //int saved_debug = opt_debug;
//  if (descp->cn != 0) ++ opt_debug;

    uint need_fetch = 0;
    if (descp->first) {
        if (descp->cn != 0 || descp->bitno != 0) ++ opt_debug;
        if (get_eis_an_base(mfp, descp) != 0) {
            cancel_run(STOP_WARN);
            opt_debug = saved_debug;
            return 1;
        }
        // reverse sequential
        if (descp->cn || descp->bitno) {
            log_msg(WARN_MSG, moi, "Non zero CN for reverse string access.  Untested.\n");
            cancel_run(STOP_WARN);
        }
        int nparts = 36 / descp->nbits; // "chars" per word
        // ndx is the offset in chars from the first word, e.g. a ptr to the
        // last char that the descriptor points to
        int ndx = descp->n + descp->area.bitpos / descp->nbits;
        ndx += descp->cn;
        -- ndx; // zero based index, not length
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Base at %#o.  Using offset of %d (len=%d - 1 + b=%dc - cn=%d) %dbit chars\n",
            descp->area.addr, ndx, descp->n, descp->area.bitpos / descp->nbits, descp->cn, descp->nbits);
        
        // Note that ndx includes offset due to cn and initial bitpos
        descp->curr.addr = descp->area.addr + ndx / nparts;
        descp->curr.bitpos = (ndx % nparts) * descp->nbits;
        // BUG: ignoring bitno
        if (descp->bitno != 0) {
            log_msg(DEBUG_MSG, moi, "Bitno %d in descriptor might be ignored\n", descp->bitno);
            cancel_run(STOP_BUG);
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Last char is at addr %#o, bit offset %d\n", descp->curr.addr, descp->curr.bitpos);
        need_fetch = 1;
    } else {
        need_fetch = descp->curr.bitpos < 0;
        if (need_fetch)
            descp->curr.bitpos = 36 - descp->nbits; // offset of last char
    }


    if (need_fetch) {
        // fetch a new src word

        if (descp->curr.addr < descp->area.min_addr || descp->curr.addr > descp->area.max_addr) {
            log_msg(DEBUG_MSG, moi, "Address %o outside segment or page range of %#o .. %#o\n", descp->curr.addr, descp->area.min_addr, descp->area.max_addr);
            if (get_eis_an_next_page(mfp, descp) != 0) {
                log_msg(ERR_MSG, moi, "Cannot access next page\n");
                cancel_run(STOP_BUG);
                return 1;
            }
        }

        // log_msg(DEBUG_MSG, moi, "Fetching src word.\n");
        if (fetch_abs_word(descp->curr.addr, &descp->word) != 0) {
            log_msg(WARN_MSG, moi, "Failed: fetch word %#o\n", descp->curr.addr);
            cancel_run(STOP_WARN);
            opt_debug = saved_debug;
            return 1;
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at %#o %012Lo\n", descp->curr.addr, descp->word);

        if (descp->first)
            descp->first = 0;
        -- descp->curr.addr;
    }

    *nib = getbits36(descp->word, descp->curr.bitpos, descp->nbits);
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012Lo, value is %#o\n", descp->nbits, descp->curr.bitpos, descp->word, *nib);
    -- descp->n;
    descp->curr.bitpos -= descp->nbits;
    opt_debug = saved_debug;
    return 0;
}

//=============================================================================

int put_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib)
{
    return put_eis_an_x(mfp, descp, nib, 1);
}

static int put_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib, flag_t advance)
{
    // Save a single char via an EIS descriptor.  Stores words when needed.  Call
    // save_eis_an() to force a store.

    const char* moi = "APU::eis-an-put";

    int saved_debug = opt_debug;
#if 0
    if (descp->curr.addr >= descp->area.max_addr || descp->first)
        if (opt_debug == 0) ++ opt_debug;
#endif

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        opt_debug = saved_debug;
        return 1;
    }

    //int saved_debug = opt_debug;
    if (descp->first) {
        //++ opt_debug;
        if (get_eis_an_base(mfp, descp) != 0) {
            opt_debug = saved_debug;
            return 1;
        }
        //opt_debug = saved_debug;
        // forward sequential
        if (descp->curr.bitpos == 0) {
            descp->word = 0;    // makes debug output easier to read
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Initial output at addr %#o is at offset zero, so not loading initial word\n", descp->curr.addr);
        } else {
            // Read in contents of target word because we won't be overwriting all of it
            //log_msg(DEBUG_MSG, moi, "First put, bit pos is %d -- loading in partial word\n", descp->bitpos);
            if (fetch_abs_word(descp->curr.addr, &descp->word) != 0) {
                log_msg(WARN_MSG, moi, "Failed.\n");
                return 1;
            }
            //opt_debug = 1;
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched target word %012Lo from addr %#o because output starts at bit %d.\n", descp->word, descp->curr.addr, descp->curr.bitpos);
            opt_debug = saved_debug;
        }
        descp->first = 0;
    }

    if (descp->curr.bitpos == 36) {
        // BUG: this might be possible if page faulted on earlier write attempt?
        log_msg(WARN_MSG, moi, "Internal error, prior call did not flush buffer\n");
        cancel_run(STOP_WARN);
        if (save_eis_an(mfp, descp) != 0) {
            opt_debug = saved_debug;
            return 1;
        }
    }

    t_uint64 tmp = descp->word;
    descp->word = setbits36(descp->word, descp->curr.bitpos, descp->nbits, nib);
    if (opt_debug > 1)
        log_msg(DEBUG_MSG, moi, "Setting %d-bit char at position %2d of %012Lo to %#o: %012Lo\n",
            descp->nbits, descp->curr.bitpos, tmp, nib, descp->word);

    if (advance) {
        -- descp->n;
        descp->curr.bitpos += descp->nbits;

        if (descp->curr.bitpos == 36) {
            // Word is full, so output it
            // Note: Bitno is irrelevent -- if necessary, we folded in the first word
            if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012Lo to addr=%#o\n", descp->word, descp->curr.addr);
            if (descp->curr.addr > descp->area.max_addr) {
                log_msg(DEBUG_MSG, moi, "Address %o is past segment or page range of %#o .. %#o\n", descp->curr.addr, descp->area.min_addr, descp->area.max_addr);
                if (get_eis_an_next_page(mfp, descp) != 0) {
                    log_msg(ERR_MSG, moi, "Cannot access next page\n");
                    cancel_run(STOP_BUG);
                    opt_debug = saved_debug;
                    return 1;
                }
            }
            if (store_abs_word(descp->curr.addr, descp->word) != 0) {
                log_msg(WARN_MSG, moi, "Failed.\n");
                opt_debug = saved_debug;
                return 1;
            }
            ++ descp->curr.addr;
            descp->curr.bitpos = 0;
        }
    }

    opt_debug = saved_debug;
    return 0;
}


//=============================================================================

int save_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{
    // Save buffered output to memory via an EIS descriptor.  Call this at the
    // end of loops in EIS opcodes.

    const char* moi = "APU::eis-an-save";

    if (descp->curr.bitpos == -1)
        return 0;
    if (descp->curr.bitpos == 0)
        return 0;

    if (descp->first) {
        log_msg(WARN_MSG, moi, "Odd, descriptor is not initialized!  Desc bitpos is %d.\n", descp->curr.bitpos);
    }

    int saved_debug = opt_debug;

    if (descp->area.min_addr > descp->curr.addr || descp->curr.addr > descp->area.max_addr) {
#if 0
        ++ opt_debug;
#endif
        log_msg(DEBUG_MSG, moi, "Address %o outside segment or page range of %#o .. %#o\n", descp->curr.addr, descp->area.min_addr, descp->area.max_addr);
        if (get_eis_an_next_page(mfp, descp) != 0) {
            log_msg(ERR_MSG, moi, "Cannot access next page\n");
            cancel_run(STOP_BUG);
            return 1;
        }
    }

    if (descp->curr.bitpos == 36) {
        log_msg(WARN_MSG, moi, "Odd, dest is a full word.\n");  // why didn't we write it during loop?
        cancel_run(STOP_WARN);
    } else {
        //log_msg(DEBUG_MSG, moi, "Dest word isn't full.  Loading dest from memory %#o and folding.\n", addr2);
        t_uint64 word;
        if (fetch_abs_word(descp->curr.addr, &word) != 0) {
            log_msg(WARN_MSG, moi, "Failed.\n");
            opt_debug = saved_debug;
            return 1;
        }
        t_uint64 tmp = descp->word;
        descp->word = setbits36(descp->word, descp->curr.bitpos, 36 - descp->curr.bitpos, word);
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Combined buffered dest %012Lo with fetched %012Lo: %012Lo\n", tmp, word, descp->word);
    }

    //opt_debug = 1;
    if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012Lo to addr=%#o.\n", descp->word, descp->curr.addr);
    opt_debug = 0;
    if (store_abs_word(descp->curr.addr, descp->word) != 0) {
        log_msg(WARN_MSG, moi, "Failed.\n");
        opt_debug = saved_debug;
        return 1;
    }
    opt_debug = saved_debug;
    ++ descp->curr.addr;
    descp->curr.bitpos = 0;
    return 0;
}


//=============================================================================

int get_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp, flag_t *bitp)
{
    // Return a single bit via an EIS descriptor.  Fetches new words as needed.

    return get_eis_an_fwd(mfp, descp, bitp, 1);
}

int retr_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp, flag_t *bitp)
{
    // Return a single bit via an EIS descriptor.  No auto-advance to next bit after each retrieval.

    return get_eis_an_fwd(mfp, descp, bitp, 0);
}

int put_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp, flag_t bitval)
{
    return put_eis_an_x(mfp, descp, bitval, 1);
}

int save_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp)
{
    return save_eis_an(mfp, descp);
}

//=============================================================================

int addr_mod_eis_addr_reg(instr_t *ip)
{
    // Called by OPU for selected EIS instructions.  OPU calls addr_mod() for
    // other instructions.

    // TODO: instr2text() also needs to understand this
    uint op = ip->opcode;
    int bit27 = op % 2;
    op >>= 1;

    uint ar = ip->addr >> 15;
    int soffset = sign15(ip->addr & MASKBITS(15));
    uint a = ip->mods.single.pr_bit;

    // BUG: Using reg_mod() to load a value that we'll later divide.  Is that right?
    // enum atag_tm tm = atag_r;
    uint td = ip->mods.single.tag & MASKBITS(4);
    TPR.CA = 0;         // BUG: what does reg mod mean for these instr?
    reg_mod(td, 0);     // BUG: what does reg mod mean for these instr?
    if (td != 0) {
        if (TPR.is_value)
            log_msg(DEBUG_MSG, "APU", "Reg mod for EIS yields value %#Lo\n", TPR.value);
        else
            log_msg(DEBUG_MSG, "APU", "Reg mod for EIS yields %#o.\n", TPR.CA);
    }


    switch (op) {
        case opcode1_a9bd: {
            int oops = sign18(TPR.CA) < 0;
            // if (oops) {
            {
                if (oops) ++ opt_debug;
                if (oops) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Enabling debug for negative CA.\n");
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "CA = %#o => %#o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Initial AR[%d]:   wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                    ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            }
            if (a == 0) {
                AR_PR[ar].wordno = soffset + sign18(TPR.CA) / 4;
                AR_PR[ar].AR.charno = TPR.CA % 4;
                if (AR_PR[ar].AR.bitno != 0) {
                    // BUG: AL39 doesn't say to clear bitno, but it seems we'd want to
                    log_msg(WARN_MSG, "APU::eis-addr-reg", "a9bd: AR bitno is non zero\n");
                    cancel_run(STOP_WARN);
                }
                // handle anomaly (AL39 AR description)
                AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
            } else {
                AR_PR[ar].wordno += soffset + (sign18(TPR.CA) + AR_PR[ar].AR.charno) / 4;
                AR_PR[ar].AR.charno = (TPR.CA + AR_PR[ar].AR.charno) % 4;
                AR_PR[ar].AR.bitno = 0;
                // handle anomaly (AL39 AR description)
                AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
            }
            if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Addr mod: AR[%d]: wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
if (AR_PR[ar].wordno == 010000005642) {
                log_msg(WARN_MSG, "APU", "a=%d; CA is %#o\n", a, TPR.CA);
                log_msg(WARN_MSG, "APU", "soffset is %d\n", soffset);
                cancel_run(STOP_BUG);
}
            if (oops) -- opt_debug;
            return 0;
        }
        case opcode1_a4bd:
        case opcode1_a6bd:
        default:
            log_msg(ERR_MSG, "APU", "internal error: opcode %03o(%d) not valid for EIS address register arithmetic\n", op, bit27);
            cancel_run(STOP_BUG);
    }
    return 1;
}

//=============================================================================
