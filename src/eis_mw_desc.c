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

extern void reg_mod(uint td, int off);  // BUG
extern int get_address(uint y, flag_t ar, uint reg, uint *addrp, uint* bitnop, int nbits);

static int get_via_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int index, uint *nib);
static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int *nib);
static void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits);
static int get_mf_an_addr(uint y, const eis_mf_t* mfp, uint *addrp, uint* bitnop, int nbits);

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


static int fetch_mf_op(uint opnum, const eis_mf_t* mfp, t_uint64* wordp)
{
    // Fetch an EIS operand via a MF

    uint addr = PPR.IC;
    if (fetch_word(PPR.IC + opnum, wordp) != 0)
        return 1;
    if (mfp != NULL && mfp->id) {
        // don't implement until we see an example
        log_msg(ERR_MSG, "APU", "Fetch EIS multi-word op: MF with indir bit set is unsupported:\n");
        uint addr = *wordp >> 18;
        uint a = getbits36(*wordp, 29, 1);
        uint td = *wordp & MASKBITS(4);
        addr_modes_t addr_mode = get_addr_mode();
        if (a) {
            // indir via pointer register
            uint pr = addr >> 15;
            int32 offset = addr & MASKBITS(15);
            int32 soffset = sign15(offset);
            // PR[pr]
            // generalize: int get_mf_an_addr(uint y, const eis_mf_t* mfp, uint *addrp, uint* bitnop)
            log_msg(ERR_MSG, "APU", "Indir word %012Lo: pr=0%o, offset=0%o(%d); REG(Td)=0%o\n", *wordp, pr, offset, soffset, td); 
        } else {
            // use 18 bit addr in all words -- fetch_word will handle
            log_msg(ERR_MSG, "APU", "Indir word %012Lo: offset=0%o(%d); REG(Td)=0%o\n", *wordp, addr, sign18(addr), td); 
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
    sprintf(bufp, "{ar=%d, rl=%d, id=%d, reg=0%o}", mfp->ar, mfp->rl, mfp->id, mfp->reg);
    return bufp;
}

const char* eis_alpha_desc_to_text(const eis_alpha_desc_t* descp)
{
    static char bufs[2][100];
    static int which = 0;
    which = !which;
    char *bufp = bufs[which];

    sprintf(bufp, "{y=0%o, char-no=0%o, ta=%o, n=0%o(%d)}",
        descp->addr, descp->cn, descp->ta, descp->n, descp->n);
    return bufp;
}

//=============================================================================

void parse_eis_alpha_desc(t_uint64 word, const eis_mf_t* mfp, eis_alpha_desc_t* descp)
{
    // Return absolute address for EIS operand in Alphanumeric Data Descriptor Format
    descp->addr = getbits36(word, 0, 18);
    descp->base_addr = descp->addr;
    descp->cn = getbits36(word, 18, 3);
    descp->ta = getbits36(word, 21, 2); // data type
    //descp->n = sign12(getbits36(word, 24, 12));
    descp->n = getbits36(word, 24, 12);
int n = descp->n;
    descp->nbits = (descp->ta == 0) ? 9 : (descp->ta == 1) ? 6 : (descp->ta == 2) ? 4 : -1;
    if (descp->nbits == -1) {
        log_msg(ERR_MSG, "APU::EIS", "Illegal ta value in MF\n");
        fault_gen(illproc_fault);
        cancel_run(STOP_BUG);
    }
    fix_mf_len(&descp->n, mfp, descp->nbits);
if ((descp->n >> 12) != 0) {
    log_msg(WARN_MSG, "APU", "parse_eis_alpha_desc: desc: n orig 0%o; After mod 0%o => 0%o\n", n, descp->n, descp->n & MASKBITS(12));
    descp->n &= MASKBITS(12);
}
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
                else if (nbits == 4 || nbits == 6)
                    *np = reg_A & MASKBITS(21);
                else if (nbits == 9)
                    *np = reg_A & MASKBITS(20);
                else {
                    // impossible
                    *np = reg_A;
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d, reg modifier 05: A=0%Lo => 0%o\n", nbits, reg_A, *np);
                    cancel_run(STOP_IBKPT);
                }
                return;
            case 06: // q
                if (nbits == 1)
                    *np = reg_Q & MASKBITS(24);
                else if (nbits == 4 || nbits == 6)
                    *np = reg_Q & MASKBITS(21);
                else if (nbits == 9)
                    *np = reg_Q & MASKBITS(20);
                else {
                    // impossible
                    *np = reg_Q;
                    cancel_run(STOP_BUG);
                }
                if (nbits != 9) {
                    log_msg(WARN_MSG, "APU", "MF len: nbits=%d, reg modifier 06: A=0%Lo => 0%o\n", nbits, reg_A, *np);
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
                log_msg(ERR_MSG, "APU", "MF len: reg modifier 0%o not implemented.\n", *np);
                cancel_run(STOP_BUG);
        }
    }
}

//=============================================================================

// Regarding all of the eis_an routines
    // TODO:
    // Would be efficient for VM system to expose the bounds of the page we're
    // addressing.  That would avoid using the VM for every single word.
    // Could have page-in always set a global.   The fetch/store y-block
    // routines could also benefit.
    
    // BUG: We get a base addr and assume everything we want is paged in


int get_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 


    //log_msg(WARN_MSG, "APU::eis-an-get", "Starting at IC 0%o\n", PPR.IC);
    //return get_via_eis_an(mfp, descp, -1, nib);
    return get_eis_an_fwd(mfp, descp, nib);
}

static int get_eis_an_base(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{
    // Get base addr
    // BUG: need to get rel addr

    const char* moi = "APU::eis-an-addr";

    uint addr1;

    if (get_mf_an_addr(descp->base_addr, mfp, &addr1, &descp->base_bitpos, descp->nbits) != 0) {
        log_msg(WARN_MSG, moi, "Failed: get_mf_an_addr\n");
        return 1;
    }
    if (descp->base_bitpos == 0) {
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Base address is 0%o (with no bit offset)\n", addr1);
    } else {
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Base address is 0%o with bit offset %d\n", addr1, descp->base_bitpos);
    }
    descp->base_addr = addr1;

    if (descp->cn != 0) {
        descp->base_bitpos += descp->cn * descp->nbits;
        if (descp->base_bitpos >= 36) {
            log_msg(WARN_MSG, moi, "Too many offset bits for a single word.  Base address is 0%o with bit offset %d; CN offset is %d (%d bits).\n", descp->base_addr, descp->base_bitpos, descp->cn, descp->cn * descp->nbits);
            cancel_run(STOP_WARN);
            descp->base_addr += descp->base_bitpos % 36;
            descp->base_bitpos /= 36;
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Cn is %d, so base bitpos becomes %d\n", descp->cn, descp->base_bitpos);
    }

    return 0;
}


static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get-f";

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    uint need_fetch = 0;
    if (descp->first) {
        if (get_eis_an_base(mfp, descp) != 0)
            return 1;
        // forward sequential
        descp->addr = descp->base_addr;
        descp->bitpos = descp->base_bitpos;
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "First char is at addr 0%o, bit offset %d\n", descp->addr, descp->bitpos);
        need_fetch = 1;
    } else {
        need_fetch = descp->bitpos == 36;
        if (need_fetch)
            descp->bitpos = 0;
    }


    if (need_fetch) {
        // fetch a new src word

        // log_msg(DEBUG_MSG, moi, "Fetching src word.\n");
        if (fetch_abs_word(descp->addr, &descp->word) != 0) {
            log_msg(WARN_MSG, moi, "Failed: fetch word 0%o\n", descp->addr);
            return 1;
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at 0%o %012Lo\n", descp->addr, descp->word);

        if (descp->first)
            descp->first = 0;
        ++ descp->addr;
    }

    *nib = getbits36(descp->word, descp->bitpos, descp->nbits);
    //if (opt_debug)
    //  log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012Lo, value is 0%o\n", descp->nbits, descp->bitpos, descp->word, *nib);
    -- descp->n;
    descp->bitpos += descp->nbits;
    return 0;
}

int get_eis_an_rev(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get-r";

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n <= 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    int saved_debug = opt_debug;
//  if (descp->cn != 0) ++ opt_debug;

    uint need_fetch = 0;
    if (descp->first) {
        if (get_eis_an_base(mfp, descp) != 0) {
            cancel_run(STOP_WARN);
            opt_debug = saved_debug;
            return 1;
        }
        // reverse sequential
        int nparts = 36 / descp->nbits; // "chars" per word
        int ndx = descp->n + descp->base_bitpos / descp->nbits;
        -- ndx; // index, not length
        ndx -= descp->cn; // BUG: We assume CN doesn't mean string is shorter than N
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Base at 0%o.  Using offset of %d (len=%d - 1 + b=%dc - cn=%d) %dbit chars\n",
            descp->base_addr, ndx, descp->n, descp->base_bitpos / descp->nbits, descp->cn, descp->nbits);
        descp->n -= descp->cn;  // will hit zero at cn instead of base
        descp->addr = descp->base_addr + ndx / nparts;
        descp->bitpos = (ndx % nparts) * descp->nbits;
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Last char is at addr 0%o, bit offset %d\n", descp->addr, descp->bitpos);
        if (descp->cn != 0) {
            log_msg(WARN_MSG, moi, "Non zero CN for reverse string access.\n");
            cancel_run(STOP_WARN);
        }
        need_fetch = 1;
    } else {
        need_fetch = descp->bitpos < 0;
        if (need_fetch)
            descp->bitpos = 36 - descp->nbits;
    }


    if (need_fetch) {
        // fetch a new src word

        // log_msg(DEBUG_MSG, moi, "Fetching src word.\n");
        if (fetch_abs_word(descp->addr, &descp->word) != 0) {
            log_msg(WARN_MSG, moi, "Failed: fetch word 0%o\n", descp->addr);
            cancel_run(STOP_WARN);
            opt_debug = saved_debug;
            return 1;
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched word at 0%o %012Lo\n", descp->addr, descp->word);

        if (descp->first)
            descp->first = 0;
        -- descp->addr;
    }

    *nib = getbits36(descp->word, descp->bitpos, descp->nbits);
    if (opt_debug)
        log_msg(DEBUG_MSG, moi, "%dbit char at bit %d of word %012Lo, value is 0%o\n", descp->nbits, descp->bitpos, descp->word, *nib);
    -- descp->n;
    descp->bitpos -= descp->nbits;
    opt_debug = saved_debug;
    return 0;
}

//=============================================================================

int put_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib)
{
    // Save a single char via an EIS descriptor.  Stores words when needed.  Call
    // save_eis_an() to force a store.

    const char* moi = "APU::eis-an-put-f";

    if (descp->first)
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Starting\n");

    if (descp->n == 0) {
        log_msg(WARN_MSG, moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    int saved_debug = opt_debug;
    if (descp->first) {
        //++ opt_debug;
        if (get_eis_an_base(mfp, descp) != 0) {
            opt_debug = saved_debug;
            return 1;
        }
        opt_debug = saved_debug;
        // forward sequential
        descp->addr = descp->base_addr;
        descp->bitpos = descp->base_bitpos;
        if (descp->bitpos == 0) {
            descp->word = 0;    // makes debug output easier to read
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Initial output at addr 0%o is at offset zero, so not loading initial word\n", descp->addr);
        } else {
            // Read in contents of target word because we won't be overwriting all of it
            //log_msg(DEBUG_MSG, moi, "First put, bit pos is %d -- loading in partial word\n", descp->bitpos);
            if (fetch_abs_word(descp->addr, &descp->word) != 0) {
                log_msg(WARN_MSG, moi, "Failed.\n");
                return 1;
            }
            //opt_debug = 1;
            if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Fetched target word %012Lo from addr 0%o because output starts at bit %d.\n", descp->word, descp->addr, descp->bitpos);
            opt_debug = saved_debug;
        }
        descp->first = 0;
    }

    if (descp->bitpos == 36) {
        // BUG: this might be possible if page faulted on earlier write attempt?
        log_msg(WARN_MSG, moi, "Internal error, prior call did not flush buffer\n");
        cancel_run(STOP_WARN);
        if (save_eis_an(mfp, descp) != 0) {
            opt_debug = saved_debug;
            return 1;
        }
    }

    t_uint64 tmp = descp->word;
    descp->word = setbits36(descp->word, descp->bitpos, descp->nbits, nib);
    if (opt_debug > 1)
        log_msg(DEBUG_MSG, moi, "Setting %d-bit char at position %2d of %012Lo to 0%o: %012Lo\n",
            descp->nbits, descp->bitpos, tmp, nib, descp->word);

    -- descp->n;
    descp->bitpos += descp->nbits;

    if (descp->bitpos == 36) {
        // Word is full, so output it
        // Note: Bitno is irrelevent -- if necessary, we folded in the first word
        if(opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012Lo to addr=0%o\n", descp->word, descp->addr);
        if (store_abs_word(descp->addr, descp->word) != 0) {
            log_msg(WARN_MSG, moi, "Failed.\n");
            opt_debug = saved_debug;
            return 1;
        }
        ++ descp->addr;
        descp->bitpos = 0;
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

    if (descp->bitpos == -1)
        return 0;
    if (descp->bitpos == 0)
        return 0;

    int saved_debug = opt_debug;

    if (descp->bitpos == 36) {
        log_msg(WARN_MSG, moi, "Odd, dest is a full word.\n");  // why didn't we write it during loop?
        cancel_run(STOP_WARN);
    } else {
        //log_msg(DEBUG_MSG, moi, "Dest word isn't full.  Loading dest from memory 0%o and folding.\n", addr2);
        t_uint64 word;
        if (fetch_abs_word(descp->addr, &word) != 0) {
            log_msg(WARN_MSG, moi, "Failed.\n");
            opt_debug = saved_debug;
            return 1;
        }
        t_uint64 tmp = descp->word;
        descp->word = setbits36(descp->word, descp->bitpos, 36 - descp->bitpos, word);
        if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Combined buffered dest %012Lo with fetched %012Lo: %012Lo\n", tmp, word, descp->word);
    }

    //opt_debug = 1;
    if (opt_debug>0) log_msg(DEBUG_MSG, moi, "Storing %012Lo to addr=0%o.\n", descp->word, descp->addr);
    opt_debug = 0;
    if (store_abs_word(descp->addr, descp->word) != 0) {
        log_msg(WARN_MSG, moi, "Failed.\n");
        opt_debug = saved_debug;
        return 1;
    }
    opt_debug = saved_debug;
    ++ descp->addr;
    descp->bitpos = 0;
    return 0;
}


//=============================================================================

static int get_mf_an_addr(uint y, const eis_mf_t* mfp, uint *addrp, uint* bitnop, int nbits)
{
    // Return absolute address for EIS operand in Alphanumeric Data Descriptor Format
    // BUG: some callers may keep results.  This isn't valid for multi-page segments.

    return get_address(y, mfp->ar, mfp->reg, addrp, bitnop, nbits);

}


void fnord()
{
#if 0
    // example
    32626:  000140100540 mlr (pr,rl),(pr,rl)
    32626:  variable=000140 opcode10=100 I1=0 MF1=(A=1,B=1,C=0,reg=0) (b1100000)
    32626:  variable=000140 opcode10=100 I1=0 MF1=(A=1,B=1,C=0,reg=0) (b1100000)
    32626:  fill=0, t=0, mf2=140(b1100000) variable=000140 opcode10=100 I1=0 MF1=(A=1,B=1,C=0,reg=0) (b1100000)
    32626:  fill=0, t=0, mf2={A=1,B=1,C=0,reg=0} opcode10=100 I1=0 MF2=(A=1,B=1,C=0,reg=0) (b1100000)
        a=1 ==> register mode
        b=1 ==> rl ==> check operands for length values
        c=0 ==>
        reg=0 ==> pr[0] for segment number
    
#endif

#if 0
    'a' flag is for addr field of operand -- not an indicator of how we fetch operand
    if (mfp->a == 0) {
        ; // do nothing
    } else {
        // Current instruction should have been fetched via APPEND mode if
        // ops are to be fetched in that mode...
        if (get_addr_mode() != APPEND_mode) {
            log_msg(ERR_MSG, "APU", "Fetch MF op: Cannot use append mode for operand when not in append mode.\n");
            cancel_run(STOP_BUG);
            return 1;
        }
    }
#endif
}

//=============================================================================

int get_eis_indir_addr(t_uint64 word, uint* addrp)
{
    const char* moi = "APU::eis-indir";

    uint addr = word >> 18;
    uint a = getbits36(word, 29, 1);
    uint td = word & MASKBITS(4);

    // the following is just for debugging
    if (a) {
        // indir via pointer register
        uint pr = addr >> 15;
        uint ar = addr >> 15;
        int32 offset = addr & MASKBITS(15);
        int32 soffset;
        soffset = sign15(offset);
        log_msg(NOTIFY_MSG, moi, "Indir word %012Lo: pr=0%o, offset=0%o(%d); REG(Td)=0%o\n", word, pr, offset, soffset, td); 
    } else {
        // use 18 bit addr in all words -- fetch_word will handle
        log_msg(NOTIFY_MSG, moi, "Indir word %012Lo: offset=0%o(%d); REG(Td)=0%o\n", word, addr, sign18(addr), td); 
    }

    uint bitno;
    int ret = get_address(addr, a, td, addrp, &bitno, 36);
    if (bitno != 0) {
        log_msg(WARN_MSG, moi, "Non zero bitno %d cannot be returned\n", bitno);
        cancel_run(STOP_BUG);
    } else {
        // log_msg(WARN_MSG, moi, "Auto breakpoint\n");
        // cancel_run(STOP_IBKPT);
    }
    //log_msg(WARN_MSG, moi, "Resulting addr is 0%o\n", *addrp);
    return ret;
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
    enum atag_tm tm = atag_r;
    uint td = ip->mods.single.tag & MASKBITS(4);
    TPR.CA = 0;         // BUG: what does reg mod mean for these instr?
    reg_mod(td, 0);     // BUG: what does reg mod mean for these instr?


    switch (op) {
        case opcode1_a9bd: {
            int oops = sign18(TPR.CA) < 0;
            // if (oops) {
            {
                if (oops) ++ opt_debug;
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "CA = 0%o => 0%o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                if (opt_debug>0) log_msg(DEBUG_MSG, "APU::eis-addr-reg", "Initial AR[%d]:   wordno = 0%o, charno=0%o, bitno=0%o; PR bitno=%d\n",
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
            if (opt_debug>0) log_msg(DEBUG_MSG, "APU", "Addr mod: AR[%d]: wordno = 0%o, charno=0%o, bitno=0%o; PR bitno=%d\n",
                ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
if (AR_PR[ar].wordno == 010000005642) {
                log_msg(WARN_MSG, "APU", "a=%d; CA is 0%o\n", a, TPR.CA);
                log_msg(WARN_MSG, "APU", "soffset is %d\n", soffset);
                cancel_run(STOP_BUG);
}
            if (oops) -- opt_debug;
            return 0;
        }
        case opcode1_a4bd:
        case opcode1_a6bd:
        default:
            log_msg(ERR_MSG, "APU", "internal error: opcode 0%0o(%d) not valid for EIS address register arithmetic\n", op, bit27);
            cancel_run(STOP_BUG);
    }
    return 1;
}

//=============================================================================
