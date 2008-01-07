typedef enum { ABSOLUTE_mode, APPEND_mode, BAR_mode } addr_modes_t;
typedef enum { NORMAL_mode, PRIV_mode } instr_modes_t;

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
    
    
