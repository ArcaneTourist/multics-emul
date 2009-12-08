/*
    eis_desc.cpp

    Routines related to descriptors for EIS multi-word instructions.
    Descriptors describe operand locations and looping controls used
    by many EIS multi-word instructions.

    Paging:

        We call the APU to translate segmented addresses into absolute
        memory addresses.  We get the page boundries from the APU and
        most or all functions avoid violating the page boundries.  This
        is more efficient than using the VM for every single word.  However,
        note that instruction restart could not assume anything previously
        retrieved is still valid -- pages may have been paged out or moved.
        We may need to move update the valid() test of ptr_t.

    Mixing put() with get() or val()

        Calls to put() and val() may be intermingled.   However, calling
        both put() a get() for the same descriptor might be undefined
        behavior.
*/

/*
    OLD:

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
 * Real HW would touch all pages of EIS args before starting operation
 * (possibly to simplify/reduce restart handling) ?
 */

//=============================================================================

using namespace std;
#include <iostream>
#include "hw6180.h"

enum atag_tm { atag_r = 0, atag_ri = 1, atag_it = 2, atag_ir = 3 }; // BUG: move to hdr

#include "eis.hpp"

//=============================================================================

/* static functions */

static void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits);
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
    int ret = get_address(y, 0, indir, td, 36, addrp, &bitno, &minaddr, &maxaddr);
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


//=============================================================================

void desc_t::len_to_text(char* bufp) const
{
    if (_mf.rl) {
        char mod[30];
        mod2text(mod, 0, _count);
        sprintf(bufp, "\"%s\"->%#o(%d)", mod, _n, _n);
    } else
        sprintf(bufp, "%#o(%d)", _n, _n);
}

//-----------------------------------------------------------------------------

void desc_t::yaddr_to_text(char* bufp) const
{
    if (_mf.ar) {
        uint pr = _addr >> 15;
        uint offset = _addr & MASKBITS(15);
        sprintf(bufp, "%#o => PR%d|%#o", _addr, pr, offset);
    } else
        sprintf(bufp, "%#o", _addr);
}

//=============================================================================

/*
 * alpha_desc_t::to_text()
 *
 * BUG/TODO/CAVEAT: Prints initial value; should probably print current value
 *
 */

char* alpha_desc_t::to_text(char *bufp) const
{
    char addr_buf[60];
    char reg_buf[40];

    len_to_text(reg_buf);
    yaddr_to_text(addr_buf);

    sprintf(bufp, "{y=%s, char-no=%#o, ta=%o, len=%s}",
        addr_buf, first_char.cn, ta(), reg_buf);
    return bufp;
}

//-----------------------------------------------------------------------------

/*
 * bit_desc_t::to_text()
 *
 * BUG/TODO/CAVEAT: Prints initial value; should probably print current value
 *
 */

char* bit_desc_t::to_text(char *bufp) const
{
    char addr_buf[60];
    char reg_buf[40];

    len_to_text(reg_buf);
    yaddr_to_text(addr_buf);

    sprintf(bufp, "{y=%s, char-no=%#o, bit-no=%d, len=%s}",
        addr_buf, first_char.cn / 9, first_char.bitno, reg_buf);
    return bufp;
}

//-----------------------------------------------------------------------------

/*
 * num_desc_t::to_text()
 *
 * BUG/TODO/CAVEAT: Prints initial value; should probably print current value
 *
 */

char* num_desc_t::to_text(char *bufp) const
{
    char addr_buf[60];
    char reg_buf[40];

    len_to_text(reg_buf);
    yaddr_to_text(addr_buf);

    sprintf(bufp, "{y=%s, char-no=%#o, %d-bits, sign-ctl=%o, sf=%02o, len=%s}",
        addr_buf, first_char.cn, _width, s, scaling_factor, reg_buf);
    return bufp;
}

//=============================================================================

const char* eis_desc_to_text(const eis_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    desc_t *dp = (desc_t*) descp->objp;
    dp->to_text(bufp);
    return bufp;
}

//=============================================================================

/*
 * desc_t::desc_t() -- constructor
 *
 */

#if 0
desc_t::desc_t()
{
    dummyp = this + 1;
    ptr_init = 0;
    _addr = 0;
    _width = 0;
    _count = 0;
}
#endif

//=============================================================================

/*
 * desc_t::init()
 *
 */

void desc_t::init(const eis_mf_t& mf, int y_addr, int width, int cn, int bit_offset, int nchar, int is_fwd)
{
    dummyp = this + 1;

    first_char.cn = cn;
    first_char.bitno = bit_offset;

    _addr = y_addr;
    ptr_init = 0;       // BUG/TODO: call init() here.  Or wait for cn fixup...
    _width = width;

    _mf = mf;
    _count = nchar;     // TODO: rename to "rl" or similar
    _n = _count;
    fix_mf_len(&_n, &_mf, width);

    // _is_read = is_read;
    _is_fwd = is_fwd;

    _curr.set(_mf.ar, _mf.reg, _width, _addr);

    if (_width == 9) {
        if ((first_char.cn & 1) != 0) {
            log_msg(ERR_MSG, "APU::EIS", "TA of 0 (9bit) not legal with cn of %d\n", first_char.cn);
            fault_gen(illproc_fault);
        } else
            first_char.cn /= 2;
    }

    if (_width * first_char.cn >= 36) {
        log_msg(ERR_MSG, "APU::EIS", "Data type TA of %d (%d bits) does not allow char pos of %d\n", ta(), width, first_char.cn);
        fault_gen(illproc_fault);
    }

    if (_width * first_char.cn + first_char.bitno >= 36) {
        log_msg(WARN_MSG, "APU::EIS", "First bit outside of word: width %d, cn %d, bitno %d\n", _width, first_char.cn, first_char.bitno);
        cancel_run(STOP_WARN);
    }

    buf.is_loaded = 0;
    buf.lo_write = -1;
    buf.hi_write = -1;

}

//=============================================================================

alpha_desc_t decode_eis_alphanum_desc(t_uint64 word, const eis_mf_t* mfp, int is_read, int is_fwd)
{
    alpha_desc_t d(*mfp, word, is_fwd);
    return d;
}

//-----------------------------------------------------------------------------

#if 0
num_desc_t decode_eis_num_desc(t_uint64 word, const eis_mf_t* mfp, int is_read, int is_fwd)
{
    num_desc_t d(*mfp, word, is_fwd);
    return d;
}
#endif

//-----------------------------------------------------------------------------

bit_desc_t decode_eis_bit_desc(t_uint64 word, const eis_mf_t* mfp, int is_read, int is_fwd)
{
    bit_desc_t d(*mfp, word, is_fwd);
    return d;
}

//=============================================================================

alpha_desc_t::alpha_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd)
{
    int y_addr = getbits36(word, 0, 18);    // we'll check for PR usage later
    int cn = getbits36(word, 18, 3);
    int bitno = 0;
    int ta = getbits36(word, 21, 2);    // data type
    int n_orig = getbits36(word, 24, 12);

    int width = (ta == 0) ? 9 : (ta == 1) ? 6 : (ta == 2) ? 4 : 0;
    if (width == 0) {
        log_msg(ERR_MSG, "APU::EIS", "Illegal ta value in MF\n");
        fault_gen(illproc_fault);
        cancel_run(STOP_BUG);
    }

    init(mf, y_addr, width, cn, bitno, n_orig, is_fwd);
}

//=============================================================================

num_desc_t::num_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd)
{
    int y_addr = getbits36(word, 0, 18);    // we'll check for PR usage later

    int cn = getbits36(word, 18, 3);
    int bitno = 0;
    int tn = getbits36(word, 21, 1);    // data type
    int stype = getbits36(word, 22, 2); 
    int sf = getbits36(word, 24, 6); 
    int n_orig = getbits36(word, 30, 6);
    int width = (tn == 0) ? 9 : 4;

    init(mf, y_addr, width, cn, bitno, n_orig, is_fwd);
    s = stype;
    scaling_factor = sf;
}

//=============================================================================

bit_desc_t::bit_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd)
{
    int y_addr = getbits36(word, 0, 18);    // we'll check for PR usage later
    int cn = getbits36(word, 18, 2);
    // We scale cn because various functions expect cn to be in units of nbits,
    // but bit descriptors use a cn measured in units 9-bit chars.
    cn *= 9;
    int bitno = getbits36(word, 20, 4);
    int n_orig = getbits36(word, 24, 12);
    if (bitno > 8) {
        log_msg(ERR_MSG, "EIS", "Bit String Operand String has illegal bitno.\n");
        cancel_run(STOP_BUG);
    }
    int width = 1;

    init(mf, y_addr, width, cn, bitno, n_orig, is_fwd);
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
                    if (opt_debug)
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
                    if (opt_debug)
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
 * ptr_t::set()
 *
 * Constructor.
 */

void ptr_t::set(bool ar, unsigned reg, int width, unsigned y)
{
    initial.ar = ar;
    initial.reg = reg;
    initial.width = width;
    initial.y = y;
}

//=============================================================================

/*
 * ptr_t::init()
 *
 * Decode the constructor args.
 * Uses struct "initial" to write struct "base".
 */

int ptr_t::init()
{
    int ret = base.init(initial.ar, initial.reg, initial.width, initial.y);
    _bitnum = base.bitno;
    return ret;
}

//=============================================================================

/*
 * ptr_t::pr_info_t::init()
 *
 *
 */

int ptr_t::pr_info_t::init(bool ar, unsigned reg, int width, unsigned y)
{
    const char* moi = "APU::EIS::addr::init";

    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "Calling get-address.\n");
    unsigned bitpos;
    if (decode_eis_address(y, ar, reg, width, &ringno, &segno, &offset, &bitno) != 0) {
        log_msg(ERR_MSG, moi, "Failed to decode eis address.\n");
        return 1;
    }
    if (opt_debug) {
        if (bitno != 0)
            log_msg(DEBUG_MSG, moi, "Initial bit offset is %d\n", bitno);
    }

    _init = 1;
    return 0;
}


//=============================================================================

/*
 * Get translation of absolute address and page boundries
 */

int ptr_t::get()
{
    const char* moi = "APU::EIS::addr::get";

    if (!base._init)
        init();             // shame on caller though

    // Unlike older version named get_eis_an_next_page(), we know our
    // offset without having to calculate it

    if (get_ptr_address(base.ringno, base.segno, base.offset + page._offset, &page.addr, &page.lo, &page.hi) != 0) {
        log_msg(ERR_MSG, moi, "Failed to translate eis address.\n");
        cancel_run(STOP_BUG);
        return 1;
    } else
        page._valid = 1;

    // Sanity check -- but only on pages other than the first page
    if (page._offset != 0) {
        if (page._offset < 0) {
            // reverse
            if (page.addr != page.hi) {
                log_msg(WARN_MSG, moi, "Next address is not at end of page: next = %06o, min = %06o, max = %06o\n", page.addr, page.lo, page.hi);
                cancel_run(STOP_WARN);
            }
        } else {
            // forward
            if (page.addr != page.lo) {
                log_msg(WARN_MSG, moi, "Next address is not at start of page: next = %06o, min = %06o, max = %06o\n", page.addr, page.lo, page.hi);
                cancel_run(STOP_WARN);
            }
        }
    }

    // BUG: useless msgs for compability
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "Page range is %06o .. %06o\n", page.lo, page.hi);

    return 0;
}

//=============================================================================

/*
 * Get translation of absolute address and page boundries
 */
#if 0
int ptr_t::get(unsigned* addrp, unsigned* bitnop, unsigned* minp, unsigned* maxp)
{
    int ret = get();
    if (ret != 0)
        return ret;

    if (addrp != NULL)
        *addrp = page.addr;
    if (bitnop != NULL)
        *bitnop = _bitnum;
    if (minp != NULL)
        *minp = page.lo;
    if (maxp != NULL)
        *maxp = page.hi;
    return 0;
}
#endif

//=============================================================================

void ptr_t::word_advance(int nwords)
{
    const char* moi = "ptr_t::word_advance";

    bool was_valid = page.valid();
    page._offset += nwords;
    page.addr += nwords;
    _bitnum = 0;
#if 0
    if (was_valid && ! page.valid())
        log_msg(INFO_MSG, moi, "Ptr %o now outside of page bounds %o .. %o.\n", page.addr, page.lo, page.hi);
#endif
}

//=============================================================================

void ptr_t::char_advance(int nchars)
{

    _bit_advance(nchars * initial.width, 1);
}

//=============================================================================

/*
 */

void ptr_t::_bit_advance(int nbits, bool quiet)
{
    const char* moi = "ptr_t::bit_advance";

    int nwords;
    unsigned bitno = _bitnum;
    if (nbits < 0) {
        unsigned absbits = - nbits;
        nwords = - (absbits / 36);
        nbits =  - (absbits % 36);
        if (bitno < (absbits % 36)) {
            bitno += 36 + nbits;
            -- nwords;
        } else {
            bitno += nbits;
        }
    } else {
        bitno += nbits;
        if (bitno < 36)
            nwords = 0;
        else {
            nwords = bitno/36;
            bitno %= 36;
        }
    }

    if (nwords == 0)
        _bitnum = bitno;
    else {
        if (!quiet)
            log_msg(INFO_MSG, moi, "Too many offset bits for a single word.  Offset was %#o with bit offset %d; now: %#o with bit offset %d.\n", page._offset, _bitnum, page._offset + nwords, bitno);
        _bitnum = bitno;
        bool was_valid = page.valid();
        page._offset += nwords;
        page.addr += nwords;
#if 0
        if (was_valid && ! page.valid())
            log_msg(INFO_MSG, moi, "Ptr %o now outside of page bounds %o .. %o after advancing %d bits.\n", page.addr, page.lo, page.hi, nbits);
#endif
    }
}

//=============================================================================

/*
 * desc_t::get()
 *
 * Return a single "character" via an EIS descriptor.  Fetches new words as
 * needed.  High order bits of returnd value are zero filled.  Advances the
 * internal pointer forwards or backwards as appropriate.
 */

int desc_t::_get(unsigned* valp, bool want_advance)
{

    const char* moi = "APU::EIS::get";

    if (n() == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    if (! ptr_init) {
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "Initializing\n");
        if (init_ptr() != 0) {
            log_msg(WARN_MSG, moi, "Cannot initialize\n");
            cancel_run(STOP_WARN);
            return 1;
        }
        if (opt_debug)
            log_msg(INFO_MSG, moi, "%s char is at addr %#o, bit offset %d\n",
                _is_fwd ? "First" : "Last", _curr.addr(), _curr.bitno());
        ptr_init = 1;
    }

    // Fetch a new src word if the buffer is empty
    if (! buf.is_loaded) {
        // First make sure our page or segment is paged in
        if (!valid()) {
            // We must have previously advanced past the beginning or end of a
            // page.
            if (buf.is_loaded) {
                log_msg(WARN_MSG, moi, "Internal error: buffer loaded for invalid address.\n");
                buf.is_loaded = 0;
                cancel_run(STOP_BUG);
            }
            if (opt_debug)
                log_msg(DEBUG_MSG, moi, "Address %o is %s segment or page range of %#o .. %#o\n",
                    _curr._addr(),
                    (_curr.addr() < _curr.min()) ? "below" : "past",
                    _curr._min(), _curr._max());
            if (_curr.get() != 0) {
                log_msg(WARN_MSG, moi, "Cannot advance to next page.\n");
                cancel_run(STOP_WARN);
                return 1;
            }
        }
        // Read the word
        if (fetch_abs_word(_curr.addr(), &buf.word) != 0) {
            log_msg(WARN_MSG, moi, "Failed: fetch word %#o\n", _curr.addr());
            cancel_run(STOP_WARN);
            return 1;
        }
        buf.is_loaded = 1;
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "Fetched word at %#o: %012llo\n",
                _curr.addr(), buf.word);
    }

    *valp = getbits36(buf.word, _curr.bitno(), _width);

    if (opt_debug > 1)
        log_msg(DEBUG_MSG, moi, "%dbit char at bit %2d of word %012llo, value is %#o\n", _width, _curr.bitno(), buf.word, *valp);

    if (want_advance) {
        -- _n;
        int addr = _curr.addr();
        _curr.char_advance(_is_fwd ? 1 : -1);
        buf.is_loaded = addr == _curr.addr();
    }

    return 0;
}
    
//=============================================================================

int decode_eis_alphanum_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd)
{
    alpha_desc_t d = decode_eis_alphanum_desc(word, mfp, is_read, is_fwd);
    descp->objp = new alpha_desc_t(d);
    descp->n = d.n();
    descp->nbits = d.width();
    descp->num.s = -1;
    descp->num.scaling_factor = -1;
    descp->dummyp = descp;
    return descp->objp == NULL;
}

//=============================================================================

int decode_eis_bit_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd)
{
    bit_desc_t d = decode_eis_bit_desc(word, mfp, is_read, is_fwd);
    descp->objp = new bit_desc_t(d);
    descp->n = d.n();
    descp->nbits = d.width();
    descp->num.s = -1;
    descp->num.scaling_factor = -1;
    descp->dummyp = descp;
    return descp->objp == NULL;
}

//=============================================================================

#if 0
int decode_eis_num_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd)
{
    num_desc_t nd = decode_eis_num_desc(word, mfp, is_read, is_fwd);
    descp->objp = new num_desc_t(nd);
    descp->n = nd.n();
    descp->nbits = nd.width();
    descp->num.s = nd.s;
    descp->num.scaling_factor = nd.scaling_factor;

    descp->dummyp = descp;
    return descp->objp == NULL;
}
#endif

//=============================================================================

void eis_desc_mod64(eis_desc_t* descp)
{
    desc_t* d = (desc_t*) descp->objp;
    d->mod64();
}

//=============================================================================

int eis_desc_get(eis_desc_t* descp, unsigned* valp)
{
    desc_t* d = (desc_t*) descp->objp;
    int ret = d->get(valp);
    descp->n = d->n();
    return ret;
}

//=============================================================================

int eis_desc_val(eis_desc_t* descp, unsigned* valp)
{
    desc_t* d = (desc_t*) descp->objp;
    int ret = d->val(valp);
    descp->n = d->n();
    return ret;
}

//=============================================================================

int eis_desc_put(eis_desc_t* descp, unsigned val)
{
    desc_t* d = (desc_t*) descp->objp;
    int ret = d->put(val);
    descp->n = d->n();
    return ret;
}

//=============================================================================

int eis_desc_flush(eis_desc_t* descp)
{
    desc_t* d = (desc_t*) descp->objp;
    return d->flush();
}

//=============================================================================

/*
 * desc_t::init_ptr()
 *
 * BUG: handle buf.is_loaded;
 */

int desc_t::init_ptr()
{
    const char* moi = "APU::EIS::addr";

    if (_curr.init() != 0) {
        log_msg(WARN_MSG, moi, "Failed to init ptr.\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    // Convert to 24-bit main memory address
    if (_curr.get() != 0) {
        log_msg(WARN_MSG, moi, "Failed to translate ptr.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    if (opt_debug) {
        if (_curr.bitno() == 0)
            log_msg(DEBUG_MSG, moi, "Base address is %#o (with no bit offset); range is %#o .. %#o\n", _curr.addr(), _curr.min(), _curr.max());
        else
            log_msg(DEBUG_MSG, moi, "Base address is %#o with bit offset %d; range is %#o .. %#o\n", _curr.addr(), _curr.bitno(), _curr.min(), _curr.max());
    }

    // BUG: why not do this in the constructor?  Or at least before
    // calling get() ?
    if (first_char.cn != 0 || first_char.bitno != 0) {
        // Descriptor initially points to a non-zero bit offset
        int curr_bit = _curr.bitno();   // save for debug msgs
        _curr.bit_advance(first_char.cn * _width + first_char.bitno);
        if (opt_debug>0) {
            if (_curr.bitno() == 0)
                log_msg(DEBUG_MSG, moi, "Cn is %d, addr is bit %d, and bitno is zero, so initial bitpos becomes %d\n", first_char.cn, curr_bit, _curr.bitno());
            else
                log_msg(DEBUG_MSG, moi, "Cn is %d, addr is bit %d, and bitno is %d, so initial bitpos becomes %d\n", first_char.cn, curr_bit, first_char.bitno, _curr.bitno());
        }
    }

    if (! _is_fwd) {
        log_msg(INFO_MSG, moi, "Reverse descriptor with %d chars; advancing %d chars.\n", _n, (_n > 0) ? _n - 1 : 0);
        if (_n > 0)
            _curr.char_advance(_n - 1);
    }

    return 0;
}

//=============================================================================

/*
 * desc_t::_put()
 *
 * Private function to save a single char Results are internally buffered
 * and stored when the buffer fills. Caller should call flush() before
 * exiting to flush the buffer and force a store.
 * Advances the internal pointer forwards or backwards as appropriate.
 *
 * BUG: we load the buffer if output starts in the middle of a word.  However,
 * it might probably make more sense to just track the position and do a
 * merge in flush().
 *
 * BUG: why is this so much more complex than get()?  Why isn't get() also
 * getting hung up on bitno positioning?
 */

int desc_t::_put(unsigned val, bool want_advance)
{
    const char* moi = "APU::EIS::put";

    if (n() == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    if (! ptr_init) {
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "Initializing\n");
        if (init_ptr() != 0) {
            log_msg(WARN_MSG, moi, "Cannot initialize\n");
            cancel_run(STOP_WARN);
            return 1;
        }
        if (opt_debug)
            log_msg(INFO_MSG, moi, "%s char is at addr %#o, bit offset %d\n",
                _is_fwd ? "First" : "Last", _curr.addr(), _curr.bitno());

        bool need_merge = _curr._bitno() != ((_is_fwd) ? 0 : 35);
        if (need_merge && buf.is_loaded) {
            log_msg(WARN_MSG, moi, "Impossible, uninitialized -- but buffer is loaded.\n");
            cancel_run(STOP_WARN);
        }
#if 1
        if (! need_merge) {
            buf.word = 0;       // makes debug output easier to read
            // BUG/TODO: set to -1 to validate flush
            if (opt_debug>0)
                log_msg(DEBUG_MSG, moi, "Initial output at addr %#o is at offset %s, so not loading initial word\n", _curr.addr(), (_curr.bitno() == 0) ? "zero" : "thirty-five");  // BUG
        } else {
            if (!_curr.valid()) {
                if (!_curr.get()) {
                    log_msg(WARN_MSG, moi, "Failed to page in.\n");
                    cancel_run(STOP_WARN);
                    return 1;
                }
            }
            // Read in contents of target word because we won't be overwriting
            // all of it
            // BUG: Don't merge.  Instead, use buf.lo_write and buf.hi_write
            // and handle in flush()
            if (fetch_abs_word(_curr.addr(), &buf.word) != 0) {
                log_msg(WARN_MSG, moi, "Failed.\n");
                return 1;
            }
            buf.is_loaded = 1;  // BUG: hack
            if (opt_debug>0)
                log_msg(DEBUG_MSG, moi, "Fetched target word %012llo from addr %#o because output starts at bit %d.\n", buf.word, _curr.addr(), _curr.bitno());
        }
#endif
        ptr_init = 1;
    }

    // BUG: bitno cannot be negative (after reverse put); this test is
    // copied from an earlier version and is obsolete...
    bool need_flush = _curr._bitno() > 35 || _curr._bitno() < 0;
    // bool need_flush = _curr.bitno() == (_is_fwd) ? 36 : 0;
    if (need_flush) {
        // BUG: this might be possible if page faulted on earlier write attempt?
        log_msg(WARN_MSG, moi, "Internal odditity, prior call did not flush buffer...\n");
        cancel_run(STOP_WARN);
        if (flush() != 0)
            return 1;
    }

    // this test is  copied from an earlier version and is obsolete...
    need_flush = _curr._bitno() > 35 || _curr._bitno() < 0;
    if (need_flush) {
        // We don't have a usable offset
        log_msg(WARN_MSG, moi, "Internal error, no usable bit offset.\n");
        cancel_run(STOP_BUG);
        return 1;
    }

    t_uint64 tmp = buf.word;
    buf.word = setbits36(buf.word, _curr._bitno(), _width, val);
    // NOTE: We need to know the bitno whether or not the page is paged-in.
    if (buf.lo_write == -1 || buf.lo_write > _curr._bitno())
        buf.lo_write = _curr._bitno();
    if (buf.hi_write == -1 || buf.hi_write < _curr._bitno())
        buf.hi_write = _curr._bitno();
    if (opt_debug > 1)
        log_msg(DEBUG_MSG, moi, "Setting %d-bit char at position %2d of %012llo to %#o: %012llo\n",
            _width, _curr._bitno(), tmp, val, buf.word);
    if ((buf.word >> 36) != 0) {
        log_msg(WARN_MSG, moi, "Internal error, bad 36-bit word: was %012llo; set %d bits at %d to %#o, now %012llo\n", tmp, _width, _curr._bitno(), val, buf.word);
        cancel_run(STOP_BUG);
    }

    // TODO: Should we check valid() here so that a future flush() will
    // be sure to have the necessary page loaded?   This would also
    // make sure bitno() isn't -1.

    // We won't flush() unless we're moving the ptr
    if (want_advance) {
        -- _n;
        bool is_buf_full = (_is_fwd) ? (_curr._bitno() + _width == 36) :
            (_curr._bitno() == 0);
        if (is_buf_full) {
            // Need to flush buffer
            if (flush(0) != 0) {
                log_msg(WARN_MSG, moi, "Failed.\n");
                cancel_run(STOP_WARN);
                return 1;
            }
        }
        if (is_buf_full) {
            // Advance a full word
            // BUG: advancing a single char would have the same effect and would auto-handle bitno...
            _curr.word_advance(_is_fwd ? 1 : -1);
            if (!_is_fwd)
                _curr._bitnum = 36 - _width;
            buf.lo_write = -1;
            buf.hi_write = -1;
            buf.is_loaded = 0;  // BUG: hack?
        } else
            _curr.char_advance(_is_fwd ? 1 : -1);
    }

    return 0;
}

//=============================================================================

// Save buffered output to memory via an EIS descriptor.  Call this at the
// end of loops in EIS opcodes.

int desc_t::flush(int verbose)
{

    const char* moi = "APU::eis-an-save";

    bool is_empty = buf.lo_write == -1 && buf.hi_write == -1;
    if (is_empty)
        return 0;

    if (! ptr_init) {
        log_msg(WARN_MSG, moi, "Odd, descriptor is not initialized!  Desc bitpos is %d.\n", _curr.bitno());
        cancel_run(STOP_BUG);
        return 1;
    }

    if (!valid()) {
        if (opt_debug)
            log_msg(DEBUG_MSG, moi, "Address %o is %s segment or page range of %#o .. %#o\n",
                _curr._addr(),
                (_curr.addr() < _curr.min()) ? "below" : "past",
                _curr._min(), _curr._max());
        if (_curr.get() != 0) {
            log_msg(ERR_MSG, moi, "Cannot access page\n");
            cancel_run(STOP_WARN);
            return 1;
        }
    }

    bool is_full = buf.lo_write == 0 && buf.hi_write == 36 - _width;
    if (buf.lo_write < 0 || buf.hi_write > 36 - _width) {
        log_msg(WARN_MSG, moi, "Odd, buffer of width %d holds bits %d .. %d\n", _width, buf.lo_write, buf.hi_write);
        cancel_run(STOP_WARN);
    }
    bool complain_full = 0; // old version complained for save() but not put()

#if 1
    if (buf.is_loaded && buf.hi_write + _width == 36) { // BUG: hack -- using old buffer on initial put scheme
        // don't do unnecessary merge
    } else
#endif
    if (is_full) {
        if (complain_full) {
            // BUG: Due to OPU::csl mixing retr, get, and put
            log_msg(WARN_MSG, moi, "Odd, dest is a full word.\n");  // why didn't we write it during loop?
            cancel_run(STOP_WARN);
        }
    } else {
        // Merge in parts of the destination with what was written to the
        // buffer.  Note that the bits in the buffer are contiguous and 
        // only have unwritten bits at either the front or the back, not
        // the middle.
        t_uint64 word;
        if (fetch_abs_word(_curr.addr(), &word) != 0) {
            log_msg(WARN_MSG, moi, "Failed.\n");
            return 1;
        }
        t_uint64 tmp = buf.word;
        if (buf.lo_write > 0)
            buf.word = setbits36(buf.word, 0, buf.lo_write, getbits36(word, 0, buf.lo_write));
        int tail = buf.hi_write + _width;
        if (tail < 36)
            buf.word = setbits36(buf.word, tail, 36 - tail, word);
        if (opt_debug>0)
            log_msg(DEBUG_MSG, moi, "Combined buffered dest %012llo with fetched %012llo: %012llo\n", tmp, word, buf.word);
    }

    //opt_debug = 1;
    if (opt_debug>0 && verbose)
        log_msg(DEBUG_MSG, moi, "Storing %012llo to addr=%#o.\n", buf.word, _curr.addr());
    if (store_abs_word(_curr.addr(), buf.word) != 0) {
        log_msg(WARN_MSG, moi, "Failed.\n");
        // opt_debug = saved_debug;
        return 1;
    }

    return 0;
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
    const char* moi = "APU::EIS::addr-reg";
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
            log_msg(ERR_MSG, moi, "Internal error: opcode %03o(%d) not implemented for EIS address register arithmetic\n", op, bit27);
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
    if (opt_debug)
        if (td != 0) {
            if (TPR.is_value)
                log_msg(DEBUG_MSG, moi, "Reg mod for EIS yields value %#llo\n", TPR.value);
            else
                log_msg(DEBUG_MSG, moi, "Reg mod for EIS yields %#o.\n", TPR.CA);
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
            // log_msg(NOTIFY_MSG, moi, "Auto breakpoint for s9bd\n");
            // cancel_run(STOP_IBKPT);
            /* fall through */
        case opcode1_a9bd:
        {
            int sign = (op == opcode1_a9bd) ? 1 : -1;
            log_msg(INFO_MSG, moi, "%s\n", (sign == 1) ? "a9bd" : "s9bd");
            int oops = sign18(TPR.CA) < 0 || sign < 0;
    oops = 1;
            if (oops) {
                ++ opt_debug;
                log_msg(DEBUG_MSG, moi, "Enabling debug for negative CA.\n");
            }
            if (opt_debug>0) {
                log_msg(DEBUG_MSG, moi, "CA = %#o => %#o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                log_msg(DEBUG_MSG, moi, "Initial AR[%d]:   wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
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
            if (opt_debug>0)
                log_msg(DEBUG_MSG, moi, "Addr mod: AR[%d]: wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
                    ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            if (oops) -- opt_debug;
            return 0;
        }
#if 1
        case opcode1_abd: {
            int oops = sign18(TPR.CA) < 0;
            if (oops) {
                ++ opt_debug;
                log_msg(DEBUG_MSG, moi, "Enabling debug for negative CA.\n");
            }
            if (opt_debug>0) {
                log_msg(DEBUG_MSG, moi, "CA = %#o => %#o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                log_msg(DEBUG_MSG, moi, "Initial AR[%d]:   wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
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
            if (opt_debug>0)
                log_msg(DEBUG_MSG, "APU", "Addr mod: AR[%d]: wordno = %#o, charno=%#o, bitno=%#o; PR bitno=%d\n",
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
