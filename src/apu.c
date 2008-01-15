/*
    apu.c -- appending unit (APU) -- "address appending"
*/

// Supposedly, appending unit HW controls fault recognition?

#include "hw6180.h"

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
        complain_msg("CPU", "Unable to determine address mode.\n");
        abort();
    }
}


addr_modes_t get_addr_mode()
{
    // BUG: get_addr_mode() probably belongs in CPU

    if (IR.abs_mode)
        return ABSOLUTE_mode;

    // BUG: addr mode depends upon instr's operand

    if (IR.not_bar_mode == 0) {
        complain_msg("CPU", "Unsure of mode -- seems to be BAR\n");
        return BAR_mode;
    }

    complain_msg("CPU", "Unsure of mode -- seems to be APPEND\n");
    return APPEND_mode;
}

//=============================================================================

#if 0
int decode_addr(instr_t* ip, t_uint64* addrp)
{
    // returns non-zero if fault in groups 1-6 detected
    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        abort();
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
        abort();
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
    [Various constucts] in abs mode places the processor in append
    mode for orne or more addr preparation cycles.  If a transfer of
    control is make with any of the above constructs, the proc remains
    in append mode after the xfer.
*/

    // BUG: do reg and indir word stuff first?


    // Addr appending below

    addr_modes_t addr_mode = get_addr_mode();
    int ptr_reg_flag = ip->pr_bit;

    // BUG: following might only be valid for basic and single word EIS instructions

    // see AL39, 4-7

    if (addr_mode == ABSOLUTE_mode && ptr_reg_flag == 0) {
        // TPR.CA is the 18-bit absolute main memory addr
        return 0;
    }

    abort();
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
