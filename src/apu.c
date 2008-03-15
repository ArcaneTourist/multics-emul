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

static const int page_size = 1024;      // CPU allows [2^6 .. 2^12]; multics uses 2^10

static int compute_addr(const instr_t *ip, ca_temp_t *ca_tempp);
static int addr_append(t_uint64 *wordp);
static int do_esn_segmentation(instr_t *ip, ca_temp_t *ca_tempp);
static int do_its_itp(const instr_t* ip, ca_temp_t *ca_tempp, t_uint64 word01);
static int page_in(uint offset, uint perm_mode, uint *addrp);
static void decode_PTW(t_uint64 word, PTW_t *ptwp);
static int set_PTW_used(uint addr);
static void decode_SDW(t_uint64 word0, t_uint64 word1, SDW_t *sdwp);
// static SDW_t* get_sdw(void);
static SDWAM_t* page_in_sdw(void);
static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp);
static void reg_mod(uint td, int off);
static int get_address(uint y, flag_t ar, uint reg, uint *addrp, uint* bitnop);
static int get_via_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int index, uint *nib);
static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int *nib);

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
        debug_msg("APU", "Setting absolute mode.\n");
    } else if (mode == APPEND_mode) {       // BUG: is this correct?
        IR.abs_mode = 0;
        IR.not_bar_mode = 1;
        debug_msg("APU", "Setting append mode.\n");
    } else if (mode == BAR_mode) {
        IR.abs_mode = 0;    // BUG: is this correct?
        IR.not_bar_mode = 0;
        debug_msg("APU", "Setting bar mode.\n");
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
        warn_msg("APU", "BAR mode is untested\n");
        cancel_run(STOP_WARN);
        return BAR_mode;
    }

    return APPEND_mode;
}

//=============================================================================


#if 0
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
#endif

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
        complain_msg("APU", "Fetch EIS multi-word op: MF with indir bit set is unsupported:\n");
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
            complain_msg("APU", "Indir word %012Lo: pr=0%o, offset=0%o(%d); REG(Td)=0%o\n", *wordp, pr, offset, soffset, td); 
        } else {
            // use 18 bit addr in all words -- fetch_word will handle
            complain_msg("APU", "Indir word %012Lo: offset=0%o(%d); REG(Td)=0%o\n", *wordp, addr, sign18(addr), td); 
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
#if 0
    // old
    if (mf2p != NULL)
        if (fetch_mf_op(2, mf2p, word2p) != 0)
            return 1;
    if (mf2p != NULL)
        if (fetch_mf_op(3, mf3p, word3p) != 0)
            return 1;
#else
    // new
    if (word2p != NULL)
        if (fetch_mf_op(2, mf2p, word2p) != 0)
            return 1;
    if (word3p != NULL)
        if (fetch_mf_op(3, mf3p, word3p) != 0)
            return 1;
#endif
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
    descp->n = getbits36(word, 24, 12);
    descp->nbits = (descp->ta == 0) ? 9 : (descp->ta == 1) ? 6 : (descp->ta == 2) ? 4 : -1;
    if (descp->nbits == -1) {
        complain_msg("APU::EIS", "Illegal ta value in MF\n");
        fault_gen(illproc_fault);
        cancel_run(STOP_BUG);
    }
    fix_mf_len(&descp->n, mfp, descp->nbits);
    if (descp->nbits == 9) {
        if ((descp->cn & 1) != 0) {
            complain_msg("APU::EIS", "TA of 0 (9bit) not legal with cn of %d\n", descp->cn);
            fault_gen(illproc_fault);
        } else
            descp->cn /= 2;
    }
    if (descp->nbits * descp->cn >= 36) {
        complain_msg("APU::EIS", "Data type TA of %d (%d bits) does not allow char pos of %d\n", descp->ta, descp->nbits, descp->cn);
        fault_gen(illproc_fault);
    }
    descp->first = 1;
}

//=============================================================================

void fix_mf_len(uint *np, const eis_mf_t* mfp, int nbits)
{
    if (mfp->rl) {
        switch(*np) {
            // case 0: // illegal
            // case 1:  // au
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
                    warn_msg("APU", "MF len: nbits=%d, reg modifier 05: A=0%Lo => 0%o\n", nbits, reg_A, *np);
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
                    warn_msg("APU", "MF len: nbits=%d, reg modifier 06: A=0%Lo => 0%o\n", nbits, reg_A, *np);
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
                complain_msg("APU", "MF len: reg modifier 0%o not implemented.\n", *np);
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


    //warn_msg("APU::eis-an-get", "Starting at IC 0%o\n", PPR.IC);
    //return get_via_eis_an(mfp, descp, -1, nib);
    return get_eis_an_fwd(mfp, descp, nib);
}

static int get_eis_an_base(const eis_mf_t* mfp, eis_alpha_desc_t *descp)
{
    // Get base addr
    // BUG: need to get rel addr

    const char* moi = "APU::eis-an-addr";

    uint addr1;

    if (get_mf_an_addr(descp->base_addr, mfp, &addr1, &descp->base_bitpos) != 0) {
        warn_msg(moi, "Failed: get_mf_an_addr\n");
        return 1;
    }
    if (descp->base_bitpos == 0) {
        debug_msg(moi, "Base address is 0%o (with no bit offset)\n", addr1);
    } else {
        debug_msg(moi, "Base address is 0%o with bit offset %d\n", addr1, descp->base_bitpos);
    }
    descp->base_addr = addr1;

    if (descp->cn != 0) {
        descp->base_bitpos += descp->cn * descp->nbits;
        if (descp->base_bitpos >= 36) {
            warn_msg(moi, "Too many offset bits for a single word.  Base address is 0%o with bit offset %d; CN offset is %d (%d bits).\n", descp->base_addr, descp->base_bitpos, descp->cn, descp->cn * descp->nbits);
            cancel_run(STOP_WARN);
            descp->base_addr += descp->base_bitpos % 36;
            descp->base_bitpos /= 36;
        }
        debug_msg(moi, "Cn is %d, so base bitpos becomes %d\n", descp->cn, descp->base_bitpos);
    }

    return 0;
}


static int get_eis_an_fwd(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get";

    if (descp->n == 0) {
        warn_msg(moi, "Descriptor exhausted\n");
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
        debug_msg(moi, "First char is at addr 0%o, bit offset %d\n", descp->addr, descp->bitpos);
        need_fetch = 1;
    } else {
        need_fetch = descp->bitpos == 36;
        if (need_fetch)
            descp->bitpos = 0;
    }


    if (need_fetch) {
        // fetch a new src word

        // debug_msg(moi, "Fetching src word.\n");
        if (fetch_abs_word(descp->addr, &descp->word) != 0) {
            warn_msg(moi, "Failed: fetch word 0%o\n", descp->addr);
            return 1;
        }
        debug_msg(moi, "Fetched word at 0%o %012Lo\n", descp->addr, descp->word);

        if (descp->first)
            descp->first = 0;
        ++ descp->addr;
    }

    *nib = getbits36(descp->word, descp->bitpos, descp->nbits);
    //if (opt_debug)
    //  debug_msg(moi, "%dbit char at bit %d of word %012Lo, value is 0%o\n", descp->nbits, descp->bitpos, descp->word, *nib);
    -- descp->n;
    descp->bitpos += descp->nbits;
    return 0;
}


static int get_via_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, int index, uint *nib)
{
    // Return a single char via an EIS descriptor.  Fetches new words as needed.
    // High order bits of nib are zero filled 

    const char* moi = "APU::eis-an-get";

    int saved_debug = opt_debug;
    uint need_fetch = 0;
    if (descp->first) {
        // Get base addr
        uint addr1;
        // BUG: need to get rel addr
        if (get_mf_an_addr(descp->base_addr, mfp, &addr1, &descp->base_bitpos) != 0) {
            warn_msg(moi, "Failed: get_mf_an_addr\n");
            opt_debug = saved_debug;
            return 1;
        }
        if (descp->base_bitpos == 0) {
            debug_msg(moi, "Base address is 0%o (with no bit offset)\n", addr1);
        } else {
            debug_msg(moi, "Base address is 0%o with bit offset %d\n", addr1, descp->base_bitpos);
        }
        descp->base_addr = addr1;
        if (descp->cn != 0) {
            descp->base_bitpos += descp->cn * descp->nbits;
            if (descp->base_bitpos >= 36) {
                warn_msg(moi, "Too many offset bits for a single word.  Base address is 0%o with bit offset %d; CN offset is %d (%d bits).\n", descp->base_addr, descp->base_bitpos, descp->cn, descp->cn * descp->nbits);
                cancel_run(STOP_WARN);
                descp->base_addr += descp->base_bitpos % 36;
                descp->base_bitpos /= 36;
            }
            debug_msg(moi, "Cn is %d, so base bitpos becomes %d\n", descp->cn, descp->base_bitpos);
        }
        if (index == -1) {
            // forward sequential
            descp->addr = descp->base_addr;
            descp->bitpos = descp->base_bitpos;
            debug_msg(moi, "First char is at addr 0%o, bit offset %d\n", descp->addr, descp->bitpos);
        } else if (index < 0) {
            // reverse sequential
            int nparts = 36 / descp->nbits; // "chars" per word
            descp->addr = descp->base_addr;
            int ndx = descp->n + descp->base_bitpos * nparts;
            descp->addr = descp->base_addr + ndx / nparts;
            descp->bitpos = (ndx % nparts) * descp->nbits;
            debug_msg(moi, "Last char is at addr 0%o, bit offset %d\n", descp->addr, descp->bitpos);
        } else {
            // Random access -- handle below
        }
        need_fetch = 1;
    } else {
        if (index == -1) {
            // forward sequential
            need_fetch = descp->bitpos == 36;
        } else if (index < 0) {
            // reverse sequential
            need_fetch = descp->bitpos < 0;
        }
    }

    if (index >= 0) {
        // Random access
        // See if current word has the bits we need
        int orig_index = index;
        index += descp->base_bitpos / descp->nbits;
        int nparts = 36 / descp->nbits; // "chars" per word
        uint a = descp->base_addr + index / nparts;
        if (descp->addr != a) {
            descp->addr = a;
            need_fetch = 1;
        }
        descp->bitpos = (index % nparts) * descp->nbits;
        debug_msg(moi, "Random access char %d is at addr 0%o, bit offset %d\n", orig_index, descp->addr, descp->bitpos);
    }

    // TODO:
    // Would be efficient for VM system to expose the bounds of the page we're
    // addressing.  That would avoid using the VM for every single word.
    // Could have page-in always set a global.   The fetch/store y-block
    // routines could also benefit.
    
    if (index < 0) {
        // Sequential access (forward or reverse)
        if (descp->n == 0) {
            warn_msg(moi, "Descriptor exhausted\n");
            opt_debug = saved_debug;
            cancel_run(STOP_WARN);
            return 1;
        }
    }

    // BUG: We get a base addr and assume everything we want is paged in

    // BUG: FIXED: This stepping logic probably isn't right for strings that are offset
    // from the initial addr via a PR.  PR defines starting location of the
    // string, so it's always a factor.   However, CN is an offset into the
    // string.  It's always a factor too.  Using CN may affect effective 
    // string length?  Probably not...
    // Example
    //   if PR points to last char, then char #2 is at the beginning of second word
    //   if PR points to first char, but CN says first char is at 3rd positon,
    //   the 2nd char must still be at the beginning of the second word


    if (need_fetch) {
        // fetch a new src word

        // debug_msg(moi, "Fetching src word.\n");
        if (fetch_abs_word(descp->addr, &descp->word) != 0) {
            warn_msg(moi, "Failed: fetch word 0%o\n", descp->addr);
            opt_debug = saved_debug;
            return 1;
        }
        opt_debug = saved_debug;
        debug_msg(moi, "Fetched word at 0%o %012Lo\n", descp->addr, descp->word);

        // figure out bitpos

        if (descp->first) {
            // Bitpos set above
            descp->first = 0;
        } else {
            if (index == -1) {
                // forward sequential
                descp->bitpos = 0;
            } else if (index < 0) {
                // reverse sequential
                //int nparts = 36 / descp->nbits;   // "chars" per word
                descp->bitpos =  36 - descp->nbits;
            } else  {
                // Random access -- bitpos set above
            }
        }
        if (index == -1) {
            // forward sequential
            ++ descp->addr;
        } else if (index < 0) {
            // reverse sequential
            -- descp->addr;
        }
        opt_debug = saved_debug;
    }
opt_debug = saved_debug;

    *nib = getbits36(descp->word, descp->bitpos, descp->nbits);
    //if (opt_debug)
    //  debug_msg(moi, "%dbit char at bit %d of word %012Lo, value is 0%o\n", descp->nbits, descp->bitpos, descp->word, *nib);
    -- descp->n;
    descp->bitpos += descp->nbits;
    return 0;
}

//=============================================================================


int put_eis_an(const eis_mf_t* mfp, eis_alpha_desc_t *descp, uint nib)
{
    // Save a single char via an EIS descriptor.  Stores words when needed.  Call
    // save_eis_an() to force a store.

    const char* moi = "APU::eis-an-put";

    if (descp->n == 0) {
        warn_msg(moi, "Descriptor exhausted\n");
        cancel_run(STOP_WARN);
        return 1;
    }

    int saved_debug = opt_debug;
    if (descp->first) {
        opt_debug = 0;
        if (get_eis_an_base(mfp, descp) != 0)
            return 1;
        opt_debug = saved_debug;
        // forward sequential
        descp->addr = descp->base_addr;
        descp->bitpos = descp->base_bitpos;
        if (descp->bitpos == 0) {
            descp->word = 0;    // makes debug output easier to read
        } else {
            // Read in contents of target word because we won't be overwriting all of it
            //debug_msg(moi, "First put, bit pos is %d -- loading in partial word\n", descp->bitpos);
            if (fetch_abs_word(descp->addr, &descp->word) != 0) {
                warn_msg(moi, "Failed.\n");
                return 1;
            }
            opt_debug = 1;
            debug_msg(moi, "Fetched target word %012Lo from addr 0%o because output starts at bit %d.\n", descp->word, descp->addr, descp->bitpos);
            opt_debug = saved_debug;
        }
        descp->first = 0;
    }

    if (descp->bitpos == 36) {
        // BUG: this might be possible if page faulted on earlier write attempt?
        warn_msg(moi, "Internal error, prior call did not flush buffer\n");
        cancel_run(STOP_WARN);
        if (save_eis_an(mfp, descp) != 0)
            return 1;
    }

    t_uint64 tmp = descp->word;
    descp->word = setbits36(descp->word, descp->bitpos, descp->nbits, nib);
    if (opt_debug > 1)
        debug_msg(moi, "Setting %d-bit char at position %2d of %012Lo to 0%o: %012Lo\n",
            descp->nbits, descp->bitpos, tmp, nib, descp->word);

    -- descp->n;
    descp->bitpos += descp->nbits;

    if (descp->bitpos == 36) {
        // Word is full, so output it
        // Note: Bitno is irrelevent -- if necessary, we folded in the first word
        debug_msg(moi, "Storing %012Lo to addr=0%o\n", descp->word, descp->addr);
        if (store_abs_word(descp->addr, descp->word) != 0) {
            warn_msg(moi, "Failed.\n");
            return 1;
        }
        ++ descp->addr;
        descp->bitpos = 0;
    }

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
        warn_msg(moi, "Odd, dest is a full word.\n");   // why didn't we write it during loop?
        cancel_run(STOP_WARN);
    } else {
        //debug_msg(moi, "Dest word isn't full.  Loading dest from memory 0%o and folding.\n", addr2);
        t_uint64 word;
        if (fetch_abs_word(descp->addr, &word) != 0) {
            warn_msg(moi, "Failed.\n");
            return 1;
        }
        t_uint64 tmp = descp->word;
        descp->word = setbits36(descp->word, descp->bitpos, 36 - descp->bitpos, word);
        debug_msg(moi, "Combined buffered dest %012Lo with fetched %012Lo: %012Lo\n", tmp, word, descp->word);
    }

    opt_debug = 1;
    debug_msg(moi, "Storing %012Lo to addr=0%o.\n", descp->word, descp->addr);
    opt_debug = 0;
    if (store_abs_word(descp->addr, descp->word) != 0) {
        warn_msg(moi, "Failed.\n");
        opt_debug = saved_debug;
        return 1;
    }
    opt_debug = saved_debug;
    ++ descp->addr;
    descp->bitpos = 0;
    return 0;
}


//=============================================================================

int get_mf_an_addr(uint y, const eis_mf_t* mfp, uint *addrp, uint* bitnop)
{
    // Return absolute address for EIS operand in Alphanumeric Data Descriptor Format
    // BUG: some callers may keep results.  This isn't valid for multi-page segments.

    return get_address(y, mfp->ar, mfp->reg, addrp, bitnop);
}


static int get_address(uint y, flag_t ar, uint reg, uint *addrp, uint* bitnop)
{
    // Return absolute address given an address, 'ar' flag, and 'reg' modifier
    // Arg ar should be negative to use current TPR or non-negative to use a pointer/address register.
    // BUG: some callers may keep results.  This isn't valid for multi-page segments.
    
    // BUG: handle BAR mode and abs mode as described in EIS indir doc
    addr_modes_t addr_mode = get_addr_mode();

    uint offset;
    uint saved_PSR, saved_PRR, saved_CA, saved_bitno;
    if (ar) {
        saved_PSR = TPR.TSR;
        saved_PRR = TPR.TRR ;
        saved_CA = TPR.CA;
        saved_bitno = TPR.bitno;
        //
        uint pr = y >> 15;
        int32 soffset = sign15(y & MASKBITS(15));
        TPR.TSR = AR_PR[pr].PR.snr;
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR);
        offset = AR_PR[pr].wordno + soffset;
        TPR.bitno = AR_PR[pr].PR.bitno;
        debug_msg("APU::get-addr", "Using PR[%d]: TSR=0%o, TRR=0%o, offset=0%o, bitno=0%o\n",
            pr, TPR.TSR, TPR.TRR, offset, TPR.bitno);
        *bitnop = TPR.bitno;
    } else {
        offset = y;
        *bitnop = 0;
    }

    if (reg != 0) {
        // BUG: ERROR: Apply EIS reg mod
        complain_msg("APU::get-addr", "Register mod 0%o unimplemented\n", reg);
        cancel_run(STOP_BUG);
        return 1;
    }

    uint perm = 0;  // BUG: need to have caller specify
    page_in(offset, perm, addrp);

    if (ar) {
        TPR.TSR = saved_PSR;
        TPR.TRR = saved_PRR;
        TPR.CA = saved_CA;
        TPR.bitno = saved_bitno;
    }

    return 0;
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
            complain_msg("APU", "Fetch MF op: Cannot use append mode for operand when not in append mode.\n");
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
        warn_msg(moi, "Indir word %012Lo: pr=0%o, offset=0%o(%d); REG(Td)=0%o\n", word, pr, offset, soffset, td); 
    } else {
        // use 18 bit addr in all words -- fetch_word will handle
        warn_msg(moi, "Indir word %012Lo: offset=0%o(%d); REG(Td)=0%o\n", word, addr, sign18(addr), td); 
    }

    uint bitno;
    int ret = get_address(addr, a, td, addrp, &bitno);
    if (bitno != 0) {
        warn_msg(moi, "Non zero bitno %d cannot be returned\n", bitno);
        cancel_run(STOP_BUG);
    } else
        warn_msg(moi, "Auto breakpoint\n");
    warn_msg(moi, "Resulting addr is 0%o\n", *addrp);
    cancel_run(STOP_IBKPT);
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
            if (oops) {
                ++ opt_debug;
                warn_msg("APU::eis-addr-reg", "CA = 0%o => 0%o =>%d\n", TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
                warn_msg("APU::eis-addr-reg", "Initial AR[%d]: wordno = 0%o, charno=0%o, bitno=0%o; PR bitno=%d\n",
                    ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            }
            if (a == 0) {
                AR_PR[ar].wordno = soffset + sign18(TPR.CA) / 4;
                AR_PR[ar].AR.charno = TPR.CA % 4;
                // handle anomaly (AL39 AR description)
                AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
            } else {
                AR_PR[ar].wordno += soffset + (sign18(TPR.CA) + AR_PR[ar].AR.charno) / 4;
                AR_PR[ar].AR.charno = (TPR.CA + AR_PR[ar].AR.charno) % 4;
                AR_PR[ar].AR.bitno = 0;
                // handle anomaly (AL39 AR description)
                AR_PR[ar].PR.bitno = AR_PR[ar].AR.charno * 9;   // 0, 9, 18, 27
            }
            debug_msg("APU", "Addr mod: AR[%d]: wordno = 0%o, charno=0%o, bitno=0%o; PR bitno=%d\n",
                ar, AR_PR[ar].wordno, AR_PR[ar].AR.charno, AR_PR[ar].AR.bitno, AR_PR[ar].PR.bitno);
            if (AR_PR[ar].wordno == 010000005642) {
                warn_msg("APU", "a=%d; CA is 0%o\n", a, TPR.CA);
                warn_msg("APU", "soffset is %d\n", soffset);
                cancel_run(STOP_BUG);
            }
            if (oops) -- opt_debug;
            return 0;
        }
        case opcode1_a4bd:
        case opcode1_a6bd:
        default:
            debug_msg("APU", "internal error: opcode 0%0o(%d) not valid for EIS address register arithmetic\n", op, bit27);
            cancel_run(STOP_BUG);
    }
    return 1;
}

//=============================================================================

int addr_mod(const instr_t *ip)
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
    addr_modes_t addr_mode = get_addr_mode();
    int ptr_reg_flag = ip->mods.single.pr_bit;
    ca_temp_t ca_temp;  // BUG: hack

    // BUG: The following check should only be done after a sequential
    // instr fetch, not after a transfer!  We're only called by do_op(),
    // so this criteria is *almost* met.   Need to detect transfers.

    if (ptr_reg_flag == 0) {
        ca_temp.soffset = sign18(ip->addr);
        // TPR.TSR = PPR.PSR;   -- done prior to fetch_instr()
        // TPR.TRR = PPR.PRR;   -- done prior to fetch_instr()
        TPR.CA = ip->addr;
        TPR.bitno = 0;
    } else {
        set_addr_mode(addr_mode = APPEND_mode);
        // AL39: Page 341, Figure 6-7
        int32 offset = ip->addr & MASKBITS(15);
        ca_temp.soffset = sign15(offset);
        uint pr = ip->addr >> 15;
        TPR.TSR = AR_PR[pr].PR.snr;
        TPR.TRR = max3(AR_PR[pr].PR.rnr, TPR.TRR, PPR.PRR);
        TPR.CA = (AR_PR[pr].wordno + ca_temp.soffset) & MASK18;
        TPR.bitno = AR_PR[pr].PR.bitno;
        debug_msg("APU", "Using PR[%d]: TSR=0%o, TRR=0%o, CA=0%o(0%o+0%o<=>%d+%d), bitno=0%o\n",
            pr, TPR.TSR, TPR.TRR, TPR.CA, AR_PR[pr].wordno, ca_temp.soffset, AR_PR[pr].wordno, ca_temp.soffset, TPR.bitno);

        // BUG: Enter append mode & stay if execute a transfer
    }

    ca_temp.tag = ip->mods.single.tag;


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
            debug_msg("APU", "Final (incomplete) CA: 0%0o\n", TPR.CA);
            return 1;
        }
        if (ca_temp.more)
            mult = 1;
        ca_temp.soffset = sign18(TPR.CA);
        if (ca_temp.more)
            debug_msg("APU", "Post CA: Continuing indirect fetches\n");
#if 0
        if (ca_temp.more)
            debug_msg("APU", "Pre Seg: Continuing indirect fetches\n");
        if (do_esn_segmentation(ip, &ca_temp) != 0) {
            debug_msg("APU", "Final (incomplete) CA: 0%0o\n", TPR.CA);
            return 1;
        }
        if (ca_temp.more)
            debug_msg("APU", "Post Seg: Continuing indirect fetches\n");
#endif
        if (ca_temp.more)
            mult = 1;
    }
    if (mult) {
            debug_msg("APU", "Final CA: 0%0o\n", TPR.CA);
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

    // uint tm = (ca_tempp->mods.single.tag >> 4) & 03; // the and is a hint to the compiler for the following switch...
    enum atag_tm tm = (ca_tempp->tag >> 4) & 03;    // the and is a hint to the compiler for the following switch...

    uint td = ca_tempp->tag & 017;

    ca_tempp->special = tm;

    switch(tm) {
        case atag_r: {  // Tm=0 -- register (r)
            reg_mod(td, ca_tempp->soffset);
            return 0;
        }
        case atag_ri: {     // Tm=1 -- register then indirect (ri)
            if (td == 3 || td == 7) {
                // ",du" or ",dl"
                warn_msg("APU", "RI with td==0%o is illegal.\n", td);
                fault_gen(illproc_fault);   // need illmod sub-category
                return 1;
            }
            int off = ca_tempp->soffset;
            uint ca = TPR.CA;
            reg_mod(td, off);
            debug_msg("APU", "RI: pre-fetch:  TPR.CA=0%o <==  TPR.CA=%o + 0%o\n",
                TPR.CA, ca, TPR.CA - ca);
            t_uint64 word;
            if (addr_append(&word) != 0)
                return 1;
            debug_msg("APU", "RI: fetch:  word at TPR.CA=0%o is 0%Lo\n",
                TPR.CA, word);
            ca_tempp->tag = word & MASKBITS(6);
            if (TPR.CA % 2 == 0 && (ca_tempp->tag == 041 || ca_tempp->tag == 043)) {
                    do_its_itp(ip, ca_tempp, word);
                    debug_msg("APU", "RI: post its/itp: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
            } else {
                TPR.CA = word >> 18;
                debug_msg("APU", "RI: post-fetch: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
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
                        if (opt_debug) {
                            // give context for appending msgs
                            debug_msg("APU", "IT(di): addr now 0%o, tally 0%o\n", addr, tally);
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
                            debug_msg("APU", "IT(id): addr now 0%o, tally 0%o\n", addr, tally);
                        }
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
#if 0
            complain_msg("APU", "IR addr mod not implemented.\n");
            cancel_run(STOP_BUG);
            return 1;
#else
            while(tm == atag_ir || tm == atag_ri) {
                if (tm == atag_ir)
                    cu.CT_HOLD = td;
                // BUG: Maybe handle special tag (41 itp, 43 its).  Or post handle?
                debug_msg("APU::IR", "pre-fetch: Td=0%o, TPR.CA=0%o\n", td, TPR.CA);
                t_uint64 word;
                if (addr_append(&word) != 0)
                    return 1;
                debug_msg("APU::IR", "fetched:  word at TPR.CA=0%o is 0%Lo:\n",
                    TPR.CA, word);
                ca_tempp->tag = word & MASKBITS(6);
                // BUG: is ITS and ITP valid for IR
                if (TPR.CA % 2 == 0 && (ca_tempp->tag == 041 || ca_tempp->tag == 043)) {
                        warn_msg("APU::IR", "found ITS/ITP\n");
                        cancel_run(STOP_WARN);
                        do_its_itp(ip, ca_tempp, word);
                        debug_msg("APU::IR", "post its/itp: TPR.CA=0%o, tag=0%o\n", TPR.CA, ca_tempp->tag);
                } else {
                    TPR.CA = word >> 18;
                    tm = (ca_tempp->tag >> 4) & 03;
                    debug_msg("APU::IR", "post-fetch: TPR.CA=0%o, tag=0%o, new tm=0%o\n", TPR.CA, ca_tempp->tag, tm);
                    if (td == 0) {
                        // BUG: Disallow a reg_mod() with td equal to NULL (AL39)
                        // Disallow always or maybe ok for ir?
                        complain_msg("APU::IR", "Found td==0 (for tm=0%o)\n", tm);
                        cancel_run(STOP_WARN);
                    }
                    switch(tm) {
                        case atag_ri:
                            reg_mod(td, ca_tempp->soffset);
                            // cotinue looping
                            break;
                        case atag_r:
                            reg_mod(td, ca_tempp->soffset);
                            return 0;
                        case atag_it:
                            //reg_mod(td, ca_tempp->soffset);
                            complain_msg("APU::IR", "Need to run normal IT algorithm, ignoring fault 1.\n"); // actually cannot have fault 1 if disallow td=0 above
                            cancel_run(STOP_BUG);
                            return 0;
                        case atag_ir:
                            // do nothing -- keep looping
                            break;
                    }
                }
                warn_msg("APU::RI", "Finished, but unverified.\n");
                cancel_run(STOP_WARN);
                return 0;
            }
#endif
        }
    }

    // BUG: Need to do ESN special handling if loop is continued

    return 0;
}


//=============================================================================

static void reg_mod(uint td, int off)
{
    switch(td) {
        case 0:
            break;  // no mod
        case 1: // ,au
            TPR.CA = off + sign18(getbits36(reg_A, 0, 18));
            TPR.CA &= MASK18;
            break;
        case 2: // ,qu
            TPR.CA = off + sign18(getbits36(reg_Q, 0, 18));
            TPR.CA &= MASK18;
            break;
        case 3: // ,du
            TPR.is_value = 1;   // BUG: Use "direct operand flag" instead
            TPR.value = ((t_uint64) TPR.CA) << 18;
            debug_msg("APU", "Mod du: Value from offset 0%o is 0%Lo\n", TPR.CA, TPR.value);
            break;
        case 4: // PPR.IC
            TPR.CA = off + PPR.IC;  // BUG: IC assumed to be unsigned
            TPR.CA &= MASK18;
            break;
        case 5:
            TPR.CA = off + sign18(getbits36(reg_A, 18, 18));
            TPR.CA &= MASK18;
            uint a = getbits36(reg_A, 18, 18);
            if (opt_debug)
                debug_msg("APU", "Tm=REG,Td=%02o: offset 0%o(%d) + A=0%Lo=>0%o(%+d decimal) ==> 0%o=>0%o(%+d)\n",
                    td, off, off, reg_A, a, sign18(a), TPR.CA, sign18(TPR.CA), sign18(TPR.CA));
            break;
        case 6:
            TPR.CA = off + sign18(getbits36(reg_Q, 18, 18));
            TPR.CA &= MASK18;
            break;
        case 7: // ,dl
            TPR.is_value = 1;   // BUG: Use "direct operand flag" instead
            TPR.value = TPR.CA; // BUG: Should we sign?
            debug_msg("APU", "Mod dl: Value from offset 0%o is 0%Lo\n", TPR.CA, TPR.value);
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
            TPR.CA &= MASK18;
            if (opt_debug)
                debug_msg("APU", "Tm=REG,Td=%02o: offset 0%o + X[%d]=0%o(%+d decimal)==>0%o(+%d) yields 0%o (%+d decimal)\n",
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
        set_addr_mode(APPEND_mode);
        uint n = getbits36(word1, 0, 3);
        TPR.TSR = AR_PR[n].PR.snr;
        SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
        uint sdw_r1 = SDWp->r1;
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
        TPR.CA &= MASK18;
        ca_tempp->more = 1;
        complain_msg("APU", "ITP not tested\n");
        cancel_run(STOP_WARN);
        return 0;
    } else if (ca_tempp->tag == 043) {
        // its
        t_uint64 word1, word2;
        // BUG: are we supposed to fetch?
        debug_msg("APU", "ITS: CA initially 0%o\n", TPR.CA);
        int ret = fetch_pair(TPR.CA, &word1, &word2);   // bug: refetching word1
        if (ret != 0)
            return ret;
        set_addr_mode(APPEND_mode);
        TPR.TSR =  getbits36(word1, 3, 15);
        uint its_rn = getbits36(word1, 18, 3);
        SDW_t *SDWp = get_sdw();    // Get SDW for TPR.TSR
        uint sdw_r1 = SDWp->r1;
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
        TPR.CA &= MASK18;
        debug_msg("APU", "ITS: CA = wordno=0%o + r=0%o => 0%o\n", i_wordno, r, TPR.CA);
        ca_tempp->more = 1;
        return 0;
    }

#if 0
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
        complain_msg("APU::append", "BAR mode not implemented.\n");
        cancel_run(STOP_BUG);
        return fetch_abs_word(offset, wordp);
    }
    if (addr_mode != APPEND_mode) {
        // impossible
        complain_msg("APU::append", "Unknown mode\n");
        cancel_run(STOP_BUG);
        return fetch_abs_word(offset, wordp);
    }

    uint addr;
    int ret = page_in(offset, 0, &addr);
    if (ret == 0) {
        debug_msg("APU::fetch_append", "Using addr 0%o\n", addr);
        ret = fetch_abs_word(addr, wordp);
    }
    return ret;
}


int store_appended(uint offset, t_uint64 word)
{
    addr_modes_t addr_mode = get_addr_mode();
    if (addr_mode != APPEND_mode) {
        // impossible
        complain_msg("APU::store-append", "Not APPEND mode\n");
        cancel_run(STOP_BUG);
    }

    uint addr;
    int ret = page_in(offset, 0, &addr);
    if (ret == 0) {
        debug_msg("APU::store-append", "Using addr 0%o\n", addr);
        ret = store_abs_word(addr, word);
    }
    return ret;
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
    return page_in(offset, perm_mode, addrp);
}

static int page_in(uint offset, uint perm_mode, uint *addrp)
{
    // Implements AL39, figure 5-4
    // Returns non-zero if a fault in groups 1-6 detected
    // Note that we allow an arbitrary offset not just TPR.CA.   This is to support
    // instruction fetches.
    // Resulting 24bit physical memory addr stored in addrp.

    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...
    debug_msg("APU::append", "Starting for Segno=0%o, offset=0%o.  (PPR.PSR is 0%o)\n", segno, offset, PPR.PSR);

    // ERROR: Validate that all PTWAM & SDWAM entries are always "full" and that use fields are always sane
    SDWAM_t* SDWp = page_in_sdw();
    if (SDWp == NULL) {
        warn_msg("APU::append", "SDW not loaded\n");
        return 1;
    }
    return page_in_page(SDWp, offset, perm_mode, addrp);
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
                // debug_msg("APU::append", "Found SDW for segno 0%o in SDWAM[%d]\n", segno, i);
                break;
            }
            //if (! SDWAM[i].assoc.is_full) {
            //  debug_msg("APU::append", "Found SDWAM[%d] is unused\n", i);
            //}
            if (SDWAM[i].assoc.use == 0)
                oldest_sdwam = i;
        }
    } else {
        // debug_msg("APU::append", "SDW for segno 0%o is the MRU -- in SDWAM[%d]\n", segno, SDWp-SDWAM);
    }

    if (SDWp != NULL) {
        // SDW is in SDWAM; it becomes the LRU
        // debug_msg("APU::append", "SDW is in SDWAM[%d].\n", SDWp - SDWAM);
        if (SDWp->assoc.use != 15) {
            for (int i = 0; i < ARRAY_SIZE(SDWAM); ++i) {
                if (SDWAM[i].assoc.use > SDWp->assoc.use)
                    -- SDWAM[i].assoc.use;
            }
            SDWp->assoc.use = 15;
        }
    } else {
        // Fetch SDW and place into SDWAM
        debug_msg("APU::append", "SDW for segno 0%o is not in SDWAM.  DSBR addr is 0%o\n", segno, DSBR.addr);
        t_uint64 sdw_word0, sdw_word1;
        if (DSBR.u) {
            // Descriptor table is unpaged
            // Do a NDSW cycle
            if (segno * 2 >= 16 * (DSBR.bound + 1)) {
                cu.word1flags.oosb = 1;         // ERROR: nothing clears
                warn_msg("APU::append", "Initial check: Segno outside DSBR bound of 0%o(%u) -- OOSB fault\n", DSBR.bound, DSBR.bound);
                fault_gen(acc_viol_fault);
                return NULL;
            }
            // debug_msg("APU::append", "Fetching SDW for unpaged descriptor table from 0%o\n", DSBR.addr + 2 * segno);
            if (fetch_abs_pair(DSBR.addr + 2 * segno, &sdw_word0, &sdw_word1) != 0)
                return NULL;
        } else {
            // Descriptor table is paged

            // First, the DSPTW fetch cycle (PTWAM doesn't cache the DS?)
            uint y1 = (2 * segno) % page_size;          // offset within page table
            uint x1 = (2 * segno - y1) / page_size;     // offset within DS page
            PTW_t DSPTW;
            t_uint64 word;
            // debug_msg("APU::append", "Fetching DS-PTW for paged descriptor table from 0%o\n", DSBR.addr + x1);
            if (fetch_abs_word(DSBR.addr + x1, &word) != 0) // We assume DS is 1024 words (bound=077->16*64=1024)
                return NULL;
            decode_PTW(word, &DSPTW);   // TODO: cache this
            if (DSPTW.f == 0) {
                debug_msg("APU::append", "DSPTW directed fault\n");
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
            // debug_msg("APU::append", "Fetching SDW from 0%o<<6+0%o => 0%o\n", DSPTW.addr, y1, (DSPTW.addr<<6) + y1);
            if (fetch_abs_pair((DSPTW.addr<<6) + y1, &sdw_word0, &sdw_word1) != 0)
                return NULL;
        }
        // Allocate a SDWAM entry
        if (oldest_sdwam == -1) {
            complain_msg("APU::append", "SDWAM had no oldest entry\n");
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
    }

    //debug_msg("APU::append", "SDW: addr - 0%o, bound = 0%o(%d), f=%d\n",
    //  SDWp->sdw.addr, SDWp->sdw.bound, SDWp->sdw.bound, SDWp->sdw.f);

    return SDWp;
}


static int page_in_page(SDWAM_t* SDWp, uint offset, uint perm_mode, uint *addrp)
{
    // Second part of page_in()
    uint segno = TPR.TSR;   // Should be been loaded with PPR.PSR if this is an instr fetch...

    // Following done for either paged or unpaged segments
    if (SDWp->sdw.f == 0) {
        debug_msg("APU::append", "SDW directed fault\n");
        fault_gen(dir_flt0_fault + SDWp->sdw.fc);   // Directed Faults 0..4 use sequential fault numbers
        return 1;
    }
    if (offset >= 16 * (SDWp->sdw.bound + 1)) {
        cu.word1flags.oosb = 1;         // ERROR: nothing clears
        debug_msg("APU::append", "SDW: Offset=0%o(%u), bound = 0%o(%u) -- OOSB fault\n", offset, offset, SDWp->sdw.bound, SDWp->sdw.bound);
        fault_gen(acc_viol_fault);
        return 1;
    }

    // ERROR: check access bits of SDW.{r,e,etc} versus reference (perm_mode arg)
    if (perm_mode != 0) {
        warn_msg("APU::append", "Segment permission checking not implemented\n");
        cancel_run(STOP_WARN);
    }

    if (SDWp->sdw.u) {
        // Segment is unpaged (it is contiguous) -- FANP cycle
        *addrp = SDWp->sdw.addr + offset;
        // debug_msg("APU::append", "Resulting addr is 0%o (0%o+0%o)\n", *addrp,  SDWp->sdw.addr, offset);
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
                    // debug_msg("APU::append", "Found PTW for (segno 0%o, page 0%o) in PTWAM[%d]\n", segno, x2, i);
                    break;
                }
                if (PTWAM[i].assoc.use == 0)
                    oldest_ptwam = i;
                //if (! PTWAM[i].assoc.is_full) {
                //  debug_msg("APU::append", "PTW[%d] is not full\n", i);
                //}
            }
        } else {
            // debug_msg("APU::append", "PTW for (segno 0%o, page 0%o) is the MRU -- in PTWAM[%d]\n", segno, x2, PTWp-PTWAM);
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
                complain_msg("APU::append", "PTWAM had no oldest entry\n");
                cancel_run(STOP_BUG);
                return 1;
            }
            t_uint64 word;
            // debug_msg("APU::append", "Fetching PTW for (segno 0%o, page 0%o) from 0%o(0%o+page)\n", segno, x2, SDWp->sdw.addr + x2, SDWp->sdw.addr);
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
                debug_msg("APU::append", "PTW directed fault\n");
                fault_gen(dir_flt0_fault + PTWp->ptw.fc);   // Directed Faults 0..4 use sequential fault numbers
                return 1;
            }
        }
        *addrp = (PTWp->ptw.addr << 6) + y2;
        // debug_msg("APU::append", "Resulting addr is 0%o (0%o<<6+0%o)\n", *addrp,  PTWp->ptw.addr, y2);
    }

    debug_msg("APU::append", "Resulting addr is 0%o\n", *addrp);

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
    sdwp->f = getbits36(word0, 33, 1);
    sdwp->fc = getbits36(word0, 34, 2);

    sdwp->bound = getbits36(word1, 1, 14);
    sdwp->r = getbits36(word1, 15, 1);
    sdwp->e = getbits36(word1, 16, 1);
    sdwp->w = getbits36(word1, 17, 1);
    sdwp->p = getbits36(word1, 18, 1);
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
        sdw.bound, sdw.bound, sdw.r, sdw.e, sdw.w, sdw.p, sdw.u, sdw.g, sdw.c, sdw.cl);
    return buf;
}

//=============================================================================
