/*
    eis_mw_desc.c

    Routines related to descriptors for EIS multi-word instructions.
    Descriptors describe operand locations and looping controls used
    by many EIS multi-word instructions.

    This code was originally in apu.c because the descriptors are used
    for addressing.
*/

// Supposedly, appending unit HW controls fault recognition?

/*
    Regarding all of the eis_an routines

    TODO, flag_t advance:

    Paging:
    We call the APU to translate segmented addresses into absolute
    memory addresses.  We get the page boundries from the APU and
    most or all functions avoid violating the page boundries.  This
    is more efficient than using the VM for every single word.  However,
    note that instruction restart could not assume anything previously
    retrieved is still valid -- pages may have been paged out or moved.
    Instead of mixing the functionality into the EIS descriptors, we
    should probably create an addressing object similar to an AR_PR that
    automatically only calls the APU when needed.

    BUG: We get a base addr and assume everything we want is paged in
    instead of checking for faults?  Fixed?
*/

/*
 * All three descriptor types are jumbled together in a single struct.
 * At the least, it needs some cleanup.
 * We provide forward and reverse "get", but only forward "put"
 */

/*
    Notes on "put"

    put_eis_an()
        Allows writing a single "char" (of whatever size).  The chars
        are buffered in the descriptor.  If the first character isn't
        destined for postion zero of the destination word, then the
        destination word is read and loaded into the buffer.
        BUG: Maybe we should record the starting offset and combine
        during write.
        Whenever the buffer fills it is written.  When the instruction
        terminates, it should call save_eis_an() to flush any remaining
        partial buffer.
    save_eis_an()
        Flushes a full or partial buffer. If the buffer isn't full the
        destination word is read and merged.
*/

/*
 * BUG: OPU::csl calls a mix of retr and put. It used to also call get
 * sometimes instead of put to advance the index when a put wasn't needed.
 * However, that was unsafe.   Need to re-check that mixing get and put
 * works as expected.
 */

//=============================================================================

#include "hw6180.h"

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 }; // BUG: move to hdr

static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib, flag_t advance);
static void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits);
static int get_mf_an_addr(const eis_mf_t* mfp, uint y, int nbits, uint *addrp, int* bitnop, uint *minaddrp, uint *maxaddrp);
static int put_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib, flag_t advance);
static int save_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, flag_t complain_full);
static int fetch_mf_op(uint opnum, const eis_mf_t* mfp, t_uint64* wordp);

//=============================================================================

/*
 * parse_mf()
 *
 * Convert a 7-bit quantity into a eis_mf_t struct.
 */

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

/*
 * fetch_mf_ops()
 *
 * Fetch the multiple operands that follow an EIS multi-word instruction.  Note
 * that interpretation of the operand words is opcode dependent -- we're
 * just doing the two or three (possibly indirect) fetches here.
 * The MF arguments control whether the words are assumed to be operands
 * or treated as indirect pointers.
 */

int fetch_mf_ops(const eis_mf_t* mf1p, t_uint64* word1p, const eis_mf_t* mf2p, t_uint64* word2p, const eis_mf_t* mf3p, t_uint64* word3p)
{

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

/*
 * get_eis_indir_addr
 *
 * Interpret an EIS indirect pointer word.  EIS multi-word instructions follow
 * a format where several words of "operand descriptors" or "indirect
 * pointers to operands" follow the EIS instruction.  (Flags in the
 * instruction word control which words are operand descriptors and
 * which words are pointers.)   This function interprets the given
 * word as an indirect pointer to an operand and returns the associated
 * 24-bit absolute memory address.
 *
 * Only called by OPU for those EIS multi-word instructions that use an
 * indirect pointer to specify the address of an operand word (for example
 * scm).
 *
 * This function could be called by other EIS routines when an MF specifes
 * that an indirect pointer to an operand descriptor should be expected.
 * However, it isn't...
 *
 * WARNING: Does not return any information about segment or page
 * boundries to the caller.
 * BUG: Caller may safely use the returned address to access a single
 * word, but using the returned address to access a table or other
 * multi-word object may result in violating page boundries.
 *
 */

int get_eis_indir_addr(t_uint64 word, uint* addrp)
{
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
        log_msg(INFO_MSG, moi, "Indir word %012llo: pr=%#o, offset=%#o(%d); REG(Td)=%#o\n", word, pr, offset, soffset, td); 
    } else {
        // use 18 bit addr
        log_msg(INFO_MSG, moi, "Indir word %012llo: addr=%#o(%d); REG(Td)=%#o\n", word, y, sign18(y), td); 
    }

    uint bitno;
    uint minaddr, maxaddr;  // unneeded

    log_msg(INFO_MSG, moi, "Calling get-address.\n");
    int ret = get_address(y, indir, 0, td, 36, addrp, &bitno, &minaddr, &maxaddr);
    if (ret == 0) {
        log_msg(INFO_MSG, moi, "Resulting addr is %#o\n", *addrp);
        if (bitno != 0) {
            log_msg(ERR_MSG, moi, "Resulting addr (%#o) includes a bit offset of %#o(%+d) that cannot be returned.\n", *addrp, bitno, bitno);
            cancel_run(STOP_BUG);
        } else {
            // log_msg(WARN_MSG, moi, "Auto breakpoint\n");
            // cancel_run(STOP_IBKPT);
        }
    } else {
        log_msg(INFO_MSG, moi, "Call to get-address(y=%#o,ar=%d,reg=%d,nbits=36,...) returns non-zero.\n", y, indir, td);
        cancel_run(STOP_WARN);
    }
    return ret;
}

//=============================================================================

/*
 * fetch_mf_op()
 *
 * Fetch a single operand that follows an EIS multi-word instruction.  The
 * opnum argument provides an offset against the IC for fetching any of
 * the multiple operands.  Note that interpretation of operand words is
 * instruction dependent -- we're just doing the (possibly indirect) fetch
 * here.  The MF argument controls whether the word following the EIS
 * instruction is assumed to be an operand or is treated as an indirect
 * pointer.
 */

static int fetch_mf_op(uint opnum, const eis_mf_t* mfp, t_uint64* wordp)
{
    // Fetch an EIS operand via a MF

    // uint addr = PPR.IC;
    if (fetch_word(PPR.IC + opnum, wordp) != 0)
        return 1;
    if (mfp != NULL && mfp->id) {
        // don't implement until we see an example
        // Should probably use get_eis_indir_addr()
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
            log_msg(ERR_MSG, "APU", "Indir word %012llo: pr=%#o, offset=%#o(%d); REG(Td)=%#o\n", *wordp, pr, offset, soffset, td); 
        } else {
            // use 18 bit addr in all words -- fetch_word will handle
            log_msg(ERR_MSG, "APU", "Indir word %012llo: offset=%#o(%d); REG(Td)=%#o\n", *wordp, addr, sign18(addr), td); 
        }
        // enum atag_tm tm = atag_r;
        // TPR.CA = 0;
        // reg_mod(td, 0);
        cancel_run(STOP_BUG);
    }
    return 0;
}

//=============================================================================

/*
 * mf2text()
 *
 * Display a MF
 */

const char* mf2text(const eis_mf_t* mfp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];
    sprintf(bufp, "{ar=%d, rl=%d, id=%d, reg=%#o}", mfp->ar, mfp->rl, mfp->id, mfp->reg);
    return bufp;
}


//-----------------------------------------------------------------------------

/*
 * mf_desc_to_text()
 *
 * Internal function to do part of the work of displaying
 * a MF and EIS descriptor.
 */

static void mf_desc_to_text(char *abuf, char *rbuf, const eis_mf_t* mfp, const eis_alpha_desc_t* descp)
{
    if (mfp->rl) {
        char mod[30];
        mod2text(mod, 0, descp->n_orig);
        sprintf(rbuf, "%s=>%#o(%d)", mod, descp->n, descp->n);
    } else
        sprintf(rbuf, "%#o(%d)", descp->n, descp->n);

    if (mfp->ar) {
        uint pr = descp->address >> 15;
        uint offset = descp->address & MASKBITS(15);
        sprintf(abuf, "%#o => PR%d|%#o", descp->address, pr, offset);
    } else
        sprintf(abuf, "%#o", descp->address);
}


//-----------------------------------------------------------------------------


/*
 * eis_alpha_desc_to_text()
 * 
 * Display an EIS alpha-numeric descriptor
 */

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
        char abuf[60];
        char reg[40];
        mf_desc_to_text(abuf, reg, mfp, descp);
        sprintf(bufp, "{y=%s, char-no=%#o, ta=%o, len=%s}", abuf, descp->cn, descp->ta, reg);
    }
    return bufp;
}


//-----------------------------------------------------------------------------


/*
 * eis_bit_desc_to_text()
 * 
 * Display an EIS bit descriptor
 */

const char* eis_bit_desc_to_text(const eis_mf_t* mfp, const eis_bit_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    if (mfp == NULL)
        sprintf(bufp, "{y=%#o, char-no=%#o, bit-no=%d n=%#o(%d)}",
            descp->address, descp->cn/9, descp->bitno, descp->n, descp->n);
    else {
        char abuf[60];
        char reg[40];
        mf_desc_to_text(abuf, reg, mfp, descp);
        sprintf(bufp, "{y=%s, char-no=%#o, bit-no=%d len=%s}",
            abuf, descp->cn/9, descp->bitno, reg);
    }
    return bufp;
}


//-----------------------------------------------------------------------------


/*
 * eis_num_desc_to_text()
 * 
 * Display an EIS numeric descriptor
 */

const char* eis_num_desc_to_text(const eis_mf_t* mfp, const eis_num_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    if (mfp == NULL)
        sprintf(bufp, "{y=%#o, char-no=%#o, %d-bits, sign-ctl=%o, sf=%02o, n=%#o(%d)}",
            descp->address, descp->cn, descp->nbits, descp->num.s, descp->num.scaling_factor, descp->n, descp->n);
    else {
        char abuf[60];
        char reg[40];
        mf_desc_to_text(abuf, reg, mfp, descp);
        sprintf(bufp, "{y=%s, char-no=%#o, %d-bits, sign-ctl=%o, sf=%02o, n=%s}",
            abuf, descp->cn, descp->nbits, descp->num.s, descp->num.scaling_factor, reg);
    }
    return bufp;
}

//=============================================================================

/*
 * parse_eis_alpha_desc()
 *
 * Interpret given word as an EIS alpha-numeric descriptor.
 * The MF controls interpretation of the length field.
 */

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
        log_msg(INFO_MSG, "APU", "parse_eis_alpha_desc: desc: n orig %#o; After mod %#o; prior bug would have truncated to %#o\n", n, descp->n, descp->n & MASKBITS(12));
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

//-----------------------------------------------------------------------------

/*
 * parse_eis_bit_desc()
 *
 * Interpret given word as an EIS bit descriptor.
 * The MF controls interpretation of the length field.
 */

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
    descp->ta = 7;  // arbitrary impossible value for 2 bit field
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

//-----------------------------------------------------------------------------

/*
 * parse_eis_num_desc()
 *
 * Interpret given word as an EIS numeric descriptor.
 * The MF controls interpretation of the length field.
 */

void parse_eis_num_desc(t_uint64 word, const eis_mf_t* mfp, eis_num_desc_t* descp)
{
    // Parse EIS operand in Numeric Operator Descriptor Format
    descp->address = getbits36(word, 0, 18);    // we'll check for PR usage later

    descp->cn = getbits36(word, 18, 3);
    descp->bitno = 0;
    descp->ta = getbits36(word, 21, 1); // data type
    descp->num.s = getbits36(word, 22, 2); 
    descp->num.scaling_factor = getbits36(word, 24, 6); 
    descp->n_orig = getbits36(word, 30, 6);
    descp->n = descp->n_orig;
    descp->nbits = (descp->ta == 0) ? 9 : 4;

int n = descp->n;
            fix_mf_len(&descp->n, mfp, descp->nbits);
            if (descp->nbits == 9) {
        #if 1
                if ((descp->cn & 1) != 0) {
                    log_msg(ERR_MSG, "APU::EIS", "TA of 0 (9bit) not legal with cn of %d\n", descp->cn);
                    fault_gen(illproc_fault);
                } else
                    descp->cn /= 2;
        #else
                // The btd instruction doesn't seem to adhere to the numeric CN conventions
                log_msg(WARN_MSG, "APU::EIS", "CN of %0o (%d) for numeric operand.\n", descp->cn, descp->cn);
                if (descp->cn > 3) {
                    log_msg(ERR_MSG, "APU::EIS", "Bad CN %d\n", descp->cn);
                    cancel_run(STOP_BUG);
                }
        #endif
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

/*
 * fix_mf_len()
 *
 * Update the length field of an EIS operand descriptor according
 * to the register length flag in the MF.  See AL-39, table 4-1
 * and the description of EIS "MF" modification fields.
 */

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
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d; expecting 1,4,6, or 9.\n", nbits);
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(DEBUG_MSG, "APU", "MF len: nbits=%d, reg modifier 05: A=%#llo => %#o\n", nbits, reg_A, *np);
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
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d; expecting 1,4,6, or 9.\n", nbits);
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(DEBUG_MSG, "APU", "MF len: nbits=%d, reg modifier 06: A=%#llo => %#o\n", nbits, reg_A, *np);
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

/*
 * get_mf_an_addr()
 *
 * Internal function to translate the address as represented by a an EIS
 * Alphanumeric Data Descriptor into an absolute 24-bit address.
 *
 * BUG: some callers may keep results.  This isn't valid for multi-page
 * segments.  However, minaddrp and maxaddrp indicate the range of
 * addresses for the page containing the returned address.
 */

static int get_mf_an_addr(const eis_mf_t* mfp, uint y, int nbits, uint *addrp, int* bitnop, uint *minaddrp, uint *maxaddrp)
{
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


/*
 * get_eis_an_base()
 *
 * This function is called just prior to the first usage of a descriptor.
 * Translates the descriptor's address field into an absolute memory address.
 * Sets the area and curr member structs of the descriptor to reflect
 * page boundries.
 */

static int get_eis_an_base(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{

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
        int curr_bit = descp->curr.bitpos;
        descp->curr.bitpos += descp->cn * descp->nbits + descp->bitno;
        if (descp->curr.bitpos >= 36) {
            log_msg(INFO_MSG, moi, "Too many offset bits for a single word.  Base address is %#o with bit offset %d; CN offset is %d (%d bits).  Advancing to next word.\n", descp->area.addr, descp->area.bitpos, descp->cn, descp->cn * descp->nbits);
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
                log_msg(DEBUG_MSG, moi, "Cn is %d, addr is bit %d, so initial bitpos becomes %d\n", descp->cn, curr_bit, descp->curr.bitpos);
            else
                log_msg(DEBUG_MSG, moi, "Cn is %d, addr is bit %d, and bitno is %d, so initial bitpos becomes %d\n", descp->cn, curr_bit, descp->bitno, descp->curr.bitpos);
        }
    }

    return 0;
}


//=============================================================================

/*
 * get_eis_an_next_page()
 *
 * Called when a descriptor hits either bound of a paged segment.
 *
 * Determines the absolute addresses of the next (higher or lower) page.
 *
 */

static int get_eis_an_next_page(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{

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
        if (get_eis_an_base(mfp, descp) != 0) {
            cancel_run(STOP_WARN);
            return 1;
        }
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
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at %#o %012llo\n", descp->curr.addr, descp->word);

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
// ERROR:    getbits36:          bad args (72000000000,i=36,n=1)

    //if (opt_debug)
    //  log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012llo, value is %#o\n", descp->nbits, descp->bitpos, descp->word, *nib);
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
            log_msg(NOTIFY_MSG, moi, "Non zero CN for reverse string access.  Lightly tested.\n");
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
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at %#o %012llo\n", descp->curr.addr, descp->word);

        if (descp->first)
            descp->first = 0;
        -- descp->curr.addr;
    }

    *nib = getbits36(descp->word, descp->curr.bitpos, descp->nbits);
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012llo, value is %#o\n", descp->nbits, descp->curr.bitpos, descp->word, *nib);
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

/*
 * put_eis_an_x()
 *
 * Internal function to save a single char via an EIS descriptor.  Results
 * are internall buffered and stored when the buffer fills. Call save_eis_an()
 * flush the buffer and force a store.
 */

static int put_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib, flag_t advance)
{

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
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched target word %012llo from addr %#o because output starts at bit %d.\n", descp->word, descp->curr.addr, descp->curr.bitpos);
            opt_debug = saved_debug;
        }
        descp->first = 0;
    }

    if (descp->curr.bitpos == 36) {
        // BUG: this might be possible if page faulted on earlier write attempt?
        log_msg(WARN_MSG, moi, "Internal error, prior call did not flush buffer\n");
        cancel_run(STOP_WARN);
        if (save_eis_an_x(mfp, descp, 0) != 0) {
            opt_debug = saved_debug;
            return 1;
        }
    }

    t_uint64 tmp = descp->word;
    descp->word = setbits36(descp->word, descp->curr.bitpos, descp->nbits, nib);
    if (opt_debug > 1)
        log_msg(DEBUG_MSG, moi, "Setting %d-bit char at position %2d of %012llo to %#o: %012llo\n",
            descp->nbits, descp->curr.bitpos, tmp, nib, descp->word);

    if (advance) {
        -- descp->n;
        descp->curr.bitpos += descp->nbits;

        if (descp->curr.bitpos == 36) {
            // Word is full, so output it
            // Note: Bitno is irrelevent -- if necessary, we folded in the first word
            if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012llo to addr=%#o\n", descp->word, descp->curr.addr);
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
    return save_eis_an_x(mfp, descp, 1);
}

static int save_eis_an_x(const eis_mf_t* mfp, eis_alpha_desc_t *descp, flag_t complain_full)
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
        if (complain_full) {
            // BUG: Due to OPU::csl mixing retr, get, and put
            log_msg(WARN_MSG, moi, "Odd, dest is a full word.\n");  // why didn't we write it during loop?
            cancel_run(STOP_WARN);
        }
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
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Combined buffered dest %012llo with fetched %012llo: %012llo\n", tmp, word, descp->word);
    }

    //opt_debug = 1;
    if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012llo to addr=%#o.\n", descp->word, descp->curr.addr);
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

//-----------------------------------------------------------------------------

int retr_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp, flag_t *bitp)
{
    // Return a single bit via an EIS descriptor.  No auto-advance to next bit after each retrieval.

    return get_eis_an_fwd(mfp, descp, bitp, 0);
}

//-----------------------------------------------------------------------------

int put_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp, flag_t bitval)
{
    return put_eis_an_x(mfp, descp, bitval, 1);
}

//-----------------------------------------------------------------------------

int save_eis_bit(const eis_mf_t* mfp, eis_bit_desc_t *descp)
{
    return save_eis_an(mfp, descp);
}

//=============================================================================

/*
 * addr_mod_eis_addr_reg()
 *
 * Called by OPU for selected EIS instructions -- instructions in the
 * EIS Address register Special Arithmetic group such as a9bd, awd, etc.
 * See the first part of the code below for a full list of opcodes.
 * Note that the OPU calls addr_mod() for the instructions that aren't
 * handled here.
 *
 * TODO: instr2text() also needs to understand this
 */

int addr_mod_eis_addr_reg(instr_t *ip)
{
    uint op = ip->opcode;
    int bit27 = op % 2;
    op >>= 1;

    int nbits;
    switch (op) {
        case opcode1_awd: nbits = 36; break;
        case opcode1_a9bd: nbits = 9; break;
        case opcode1_a6bd: nbits = 6; break;
        case opcode1_a4bd: nbits = 4; break;
        case opcode1_abd: nbits = 1; break;
        case opcode1_s9bd: nbits = 9; break;
        default:
            log_msg(ERR_MSG, "APU", "internal error: opcode %03o(%d) not implemented for EIS address register arithmetic\n", op, bit27);
            cancel_run(STOP_BUG);
    }

    uint ar = ip->addr >> 15;
    int soffset = sign15(ip->addr & MASKBITS(15));
    uint a = ip->mods.single.pr_bit;

    // BUG: Using reg_mod() to load a value that we'll later divide.  Is that right?
    // enum atag_tm tm = atag_r;
    uint td = ip->mods.single.tag & MASKBITS(4);
    TPR.CA = 0;         // BUG: what does reg mod mean for these instr?
    reg_mod(td, 0);     // BUG: Do we need to use the same width special cases as for MF fields? (prob not)
    if (td != 0) {
        if (TPR.is_value)
            log_msg(DEBUG_MSG, "APU", "Reg mod for EIS yields value %#llo\n", TPR.value);
        else
            log_msg(DEBUG_MSG, "APU", "Reg mod for EIS yields %#o.\n", TPR.CA);
    }


    switch (op) {
        case opcode1_awd:
            if (a == 0)
                AR_PR[ar].wordno = soffset + sign18(TPR.CA);
            else
                AR_PR[ar].wordno += soffset + sign18(TPR.CA);
            AR_PR[ar].PR.bitno = 0;
            AR_PR[ar].AR.charno = 0;
            AR_PR[ar].AR.bitno = 0;
            return 0;
        case opcode1_s9bd:
            cancel_run(STOP_IBKPT);
            // fall through
        case opcode1_a9bd:
        {
            int sign = (op == opcode1_a9bd) ? 1 : -1;
            log_msg(INFO_MSG, "APU::eis-addr-reg", "%s\n", (sign == 1) ? "a9bd" : "s9bd");
            int oops = sign18(TPR.CA) < 0 || sign < 0;
    oops = 1;
            // if (oops)
            {
                if (oops) ++ opt_debug;
                if (oops) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Enabling debug for negative CA.\n");
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "CA = %#o => %#o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Initial AR[%d]:   wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                    ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            }
            if (a == 0) {
                int wordno = soffset + sign18(TPR.CA) / 4;
                unsigned charno = TPR.CA % 4;
                AR_PR[ar].wordno = (sign == 1)? wordno : negate18(wordno);
                AR_PR[ar].AR.charno = (sign == 1) ? charno : ((- charno) &  3);
            } else {
                if (sign == 1) {
                    AR_PR[ar].wordno += soffset + (sign18(TPR.CA) + AR_PR[ar].AR.charno) / 4;
                    AR_PR[ar].AR.charno = (AR_PR[ar].AR.charno + TPR.CA) % 4;
                } else {
                    AR_PR[ar].wordno -= soffset;
                    AR_PR[ar].wordno += (AR_PR[ar].AR.charno - sign18(TPR.CA)) / 4;
                    AR_PR[ar].AR.charno = ((int) AR_PR[ar].AR.charno - TPR.CA % 4) & 3;
                }
            }
            AR_PR[ar].AR.bitno = 0;
            // handle anomaly (AL39 AR description)
            AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
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
#if 1
        case opcode1_abd: {
            int oops = sign18(TPR.CA) < 0;
            {
                if (oops) ++ opt_debug;
                if (oops) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Enabling debug for negative CA.\n");
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "CA = %#o => %#o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Initial AR[%d]:   wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                    ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            }
            if (a == 0) {
                int creg = sign18(TPR.CA);
                AR_PR[ar].wordno = soffset + creg / 36;
                AR_PR[ar].AR.charno = (creg % 36) / 9;
                AR_PR[ar].AR.bitno = creg % 9;
            } else {
                int creg = sign18(TPR.CA);
                // AR_PR[ar].wordno += soffset + (9 *  AR_PR[ar].AR.charno + 36 * creg + AR_PR[ar].AR.bitno) / 36;
                // The text of AL39 is correct, but the equation is wrong; multiplying creg by 36 makes no sense
                int nbits = 9 * AR_PR[ar].AR.charno + creg + AR_PR[ar].AR.bitno;
                AR_PR[ar].wordno += soffset + nbits / 36;
                AR_PR[ar].AR.charno = (nbits % 36) / 9;
                AR_PR[ar].AR.bitno = nbits % 9;
            }
            // handle anomaly (AL39 AR description)
            AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
            if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Addr mod: AR[%d]: wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            if (oops) -- opt_debug;
            return 0;
        }
#endif
        case opcode1_a4bd:
        case opcode1_a6bd:
        default:
            log_msg(ERR_MSG, "APU", "internal error: opcode %03o(%d) not implemented for EIS address register arithmetic\n", op, bit27);
            cancel_run(STOP_BUG);
    }
    return 1;
}

//=============================================================================
