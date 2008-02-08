/*
    apu.c -- appending unit (APU) -- "address appending"
*/

// Supposedly, appending unit HW controls fault recognition?

#include "hw6180.h"

//=============================================================================

static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        debug_msg("APU::sign18", "0%Lo => 0%o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}

//=============================================================================

#if 0
    NOTES

#endif


void set_addr_mode(addr_modes_t mode)
{
    // BUG: set_addr_mode() probably belongs in CPU
    if (mode == ABSOLUTE_mode) {
        IR.abs_mode = 1;
        IR.not_bar_mode = 1;
    } else if (mode == APPEND_mode) {       // BUG: is this correct?
        IR.abs_mode = 0;
        IR.not_bar_mode = 1;
    } else if (mode == BAR_mode) {
        IR.abs_mode = 0;    // BUG: is this correct?
        IR.not_bar_mode = 0;
    } else {
        complain_msg("APU", "Unable to determine address mode.\n");
        cancel_run(STOP_BUG);
    }
}


addr_modes_t get_addr_mode()
{
    // BUG: get_addr_mode() probably belongs in CPU

    if (IR.abs_mode)
        return ABSOLUTE_mode;

    // BUG: addr mode depends upon instr's operand

    if (IR.not_bar_mode == 0) {
        complain_msg("APU", "Unsure of mode -- seems to be BAR\n");
        return BAR_mode;
    }

    complain_msg("APU", "Unsure of mode -- seems to be APPEND\n");
    return APPEND_mode;
}

//=============================================================================

#if 0
int decode_addr(instr_t* ip, t_uint64* addrp)
{
    // returns non-zero if fault in groups 1-6 detected
    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        complain_msg("APU", "Only ABS mode implemented.\n");
        cancel_run(STOP_BUG);
    }
    *addrp = ip->offset;
    return 0;
}
#endif

//-----------------------------------------------------------------------------

int decode_ypair_addr(instr_t* ip, t_uint64* addrp)
{
    // returns non-zero if fault in groups 1-6 detected
    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        complain_msg("APU", "Only ABS mode implemented.\n");
        cancel_run(STOP_BUG);
    }

    t_uint64 addr = ip->offset;
    if (addr % 2 == 1)
        -- addr;
    *addrp = addr;

    return 0;
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
    preparation cycles.  If a transfer of control is make with any of the
    above constructs, the proc remains in append mode after the xfer.
*/

    // BUG: do reg and indir word stuff first?


    // Addr appending below

    addr_modes_t addr_mode = get_addr_mode();
    int ptr_reg_flag = ip->pr_bit;

    // BUG: following might only be valid for basic and single word EIS instructions

    // see AL39, 4-7

    // BUG: if (appropriate instr)
    //  18bit offset rel to current val of instr counter
    uint tm = (ip->tag >> 4) & 03;  // the and is a hint to the compiler for the following switch...
    uint td = ip->tag & 017;

    switch(tm) {
        case 0: {   // register (r)
            if (td == 0)
                break;
            int off = sign18(ip->offset);
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
                    ip->is_value = 1;
                    //ip->value = offset << 18;
                    TPR.CA = (t_uint64) ip->offset << 18;
                    debug_msg("APU", "Mod du: Value from offset 0%o; TPR.CA now 0%Lo\n", ip->offset, TPR.CA);
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
                    ip->is_value = 1;
                    // ip->value = offset;
                    TPR.CA = ip->offset;
                    debug_msg("APU", "Mod dl: Value from offset 0%o; TPR.CA now 0%o\n", ip->offset, TPR.CA);
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
            break;
        }
        case 1: {   // register then indirect (ri)
            complain_msg("APU", "RI addr mod not implemented.\n");
            cancel_run(STOP_BUG);
            return 1;
        }
        case 2: {   // indirect then tally (it)
            switch(td) {
                case 0:
                    debug_msg("APU", "IT with Td zero not valid in instr word.\n");
                    fault_gen(f1_fault);    // This mode not ok in instr word
                    break;
                case 014: {
                    t_uint64 iword;
                    int ret;
                    int iloc = TPR.CA;
                    if ((ret = fetch_word(TPR.CA, &iword)) == 0) {
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
                case 016: {
                    t_uint64 iword;
                    int ret;
                    int iloc = TPR.CA;
                    if ((ret = fetch_word(TPR.CA, &iword)) == 0) {
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
        case 3: {   // indirect then register (ir)
            complain_msg("APU", "IR addr mod not implemented.\n");
            cancel_run(STOP_BUG);
            return 1;
        }
    }

    if (addr_mode == ABSOLUTE_mode && ptr_reg_flag == 0) {
        // TPR.CA is the 18-bit absolute main memory addr
        return 0;
    }

    complain_msg("APU", "Only ABS mode implemented.\n");
    cancel_run(STOP_BUG);
#if 0
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
