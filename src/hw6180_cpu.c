#include "hw_6180.h"

//-----------------------------------------------------------------------------
//*** SIMH specific externs

extern t_uint64 M[];    /* memory */


//-----------------------------------------------------------------------------
//*** Registers

static t_uint64 PC; /* probably wrong size */
IR_t IR;    // indicator register

// SIMH gets a copy of all registers
// todo: modify simh to take a PV_LSBLEFT flag (leftmost bit is bit 0)
REG cpu_reg[] = {
    // name="PC", loc=PC, radix=<8>, width=36, offset=<0>, depth=<1>, flags=, qptr=
    { ORDATA (PC, PC, 36) },
    { FLDATA (IR.Z, IR.zero, 0) },
#define FLDATA(nm,loc,pos) #nm, &(loc), 2, 1, (pos), 1

    { NULL }
};


//-----------------------------------------------------------------------------
//*** Other Globals holding state values
//      If SIMH uses known devices to switch between CPUs, we'll have to 
//      add these to cpu_reg as read only (perhaps even hidden) registers.
//      In order to suppport SIMH's save/restore commands, we'll at least
//      have to register a dummy register and examine/deposit routine.

static cycles_t cycle;


//-----------------------------------------------------------------------------
//***  Other Externs
extern int bootimage_loaded;    // only relevent for the boot CPU ?


//=============================================================================

t_stat cpu_reset (DEVICE *dptr)
{
    // todo: reset *all* structures to zero
    // Note that SIMH doesn't have much difference between reset and power-on

    // Real hardware had to wait for a connect signal from the SCU before
    // doing anything -- multicians.org glossary
    // Tempting to load "idle until interrupt" into IR.
    // However, we'll use a flag to simulate watching for a control signal
    // being seeing after boot load.   Actually, we could just require
    // the user to load a boot tape image before starting the CPU...

    bootimage_loaded = 0;

    generate Startup Fault
}


//=============================================================================


t_stat sim_instr(void)
{
    // This function is called by SIMH to execute one or more instructions (or
    // at least perform one or more processor cycles).

    // principal elements of the 6180 processor:
    //  appending unit (addressing)
    //  assoc mem assembly (VM registers)
    //  control unit (addr mod, instr_mode, interrupt recognition, 
    //      decode instr & indir words, timer registers
    //  operation unit (binary arithmetic, boolean)
    //  decimal unit (decimal arithmetic, char string, bit string)
    //
    
    int reason = 0;
    while (reason == 0) {   /* loop until halted */
        if (sim_interval <= 0) { /* check clock queue */
            // process any SIMH timed events including keyboard halt
            if (reason = sim_process_event()) break;
        }
        control_unit();
        /* -- this section obsoleted by following?
            // interrupts mostly only at end of any even/odd instr *pair*
            if (int_req > 0) { /* interrupt? */
                // todo: handle interrupt
            }
            fetch if needed -- two words mostly
        */
        sim_interval--; // todo: maybe only per instr or by brkpoint type?
    }
    return reason;
}


//=============================================================================

typedef enum {
    ABORT_cycle, FAULT_cycle, EXEC_cycle, INTERRUPT_CYCLE,
    FETCH_cycle,
} cycles_t;
# CA FETCH OPSTORE, DIVIDE_EXEC

control_unit(void)
{
    // ------------------------------------------------------------------------

    // Emulation of the control unit -- fetch cycle, execute cycle, 
    // interrupt handling, fault handling, etc.
    //
    // We allow SIMH to regain control between any of the cycles of
    // the control unit.   This includes returning to SIMH on fault
    // detection and between the even and odd words of a fetched
    // two word instruction pair.

    // ------------------------------------------------------------------------

    // See the following portions of AL39:
    //    Various registers in Section 3: "CONTROL UNIT DATA", IR, Fault, etc
    //    Note word 5 etc in control unit data
    //    All of Section 7

    // SEE ALSO
    //  AN87 -- CPU history registers (interrupt is present, etc)

    // ------------------------------------------------------------------------

    // internal notes -- order of events

    // CU detects fault conditions and determines
    // when the fault will be handled.
    // Multics does not inspect the fault register (AN87, 1-41)

    // TODO -- what to do with int_req variable ?

    // interrupts only recognized during instruction pair fetches,
    // not in mid-instruction -- multicians.org glossary

    // interrupt vector at addr 0; fault vector at arbitrary addr
    // Interrupt or fault signals saved if inhibited and reset only
    // when the trap occurs -- AL39 1-3

    // ------------------------------------------------------------------------

    if (! bootimage_loaded) {
        complain_msg("CPU::CU", "Memory is empty, no bootimage loaded yet\n");
        return;
    }

    if (cycle == FETCH_CYCLE) {
        // If execution of the current pair is complete, we should check
        // two? internal flags for group 7 faults and/or interrupts.
        // Group 7 -- See tally runout in IR, connect fields of Fault register
        // DC power off must come via an interrupt?
        if check for group 7 faults
            cycle = FAULT_cycle;
        } else {
            if interrupts
                cycle = INTERRUPT_cycle;
        }
    }

    // problem: need to recognize faults during long instructions
    // Are interrupts really note recognized during those?  Seems like
    // we might need them for operand fetches

    at various points in the code:
        sample XIP interrupt present lines
        if any on, set internal flag or register

    // Check for fault  --- NOTES
    //
    // How?  If one of the 32 interrupt cells is set in one of the SCs,
    // our processor will have the interrupt present (XIP) line active.
    // Perhaps faults are flagged in the same way via the SXC system
    // controller command.
    // 
    // Processor always fetches instructions in pairs.   At an appropriate
    // point (as early as possible) in the execution of a pair, the next
    // seq pair if fetched and held in a special instr buff reg.  Exact
    // point depends on instr seq and other conditions.
    // Group 7 interupts sampled at point of next pairs addr formation;
    // if any found, a flag is set.   CPU next samples interrupt lines
    // from all eight memory interface ports and loads a register with
    // bits corresponding to states of the lines.  If any bit in the
    // register is set ON, an internal flag is set to reflect the presence
    // of the bit(s) in the register.
    // [ exceptions to sampling group 7 -- result of xfer, repeat instrs ]
    // See also AL39, 3-30 re group 7 opcode traps etc.
    //
    // During exec of current pair, processor test internal flags for
    // group 7 faults and interrupts.   If either flag is set, it does 
    // not fetch the next pair.
    //
    // At completion of current pair, once again check internal flags.  If
    // neither one is set, exec of next instr pair proceeds.  If the internal
    // flag for group 7 faults is set, the processor enters a FAULT cycle
    // of rht highest prior group 7 fault present.   If the internal flag
    // for interrups is set, the processor enters an INTERRUPT cycle.

    switch(cycle) {
        case ABORT_cycle:
            debug_msg("CPU::CU", "Cycle = ABORT\n");
            // Invoked when control unit decides to handle fault
            // Bring all overlapped functions to an orderly halt -- however,
            // the simulator has no overlapped functions?
            // Also bring asynchronous functions within the processor
            // to an orderly halt -- todo -- do we have any?
            cycle = FAULT_cycle;
            break;
        case FAULT_cycle:
            debug_msg("CPU::CU", "Cycle = FAULT\n");

            // TODO: Safe store control unit data into invisible registers
            // in prep for a store control unit (scu) instr

            Enter temp absolute mode
            force current ring of execution C(PPR.PRR) to zero
            gen computed addr for fault trap pair
                Setting of FAULT BASE switches and 2x the fault #
            Force computed addr and xed opcode into the instruction
            register and execute (during FAULT CYCLE not EXECUTE CYCLE).
            // todo: for SIMH breakpoint on execution for that addr or
            // maybe in the code for the xed opcode
    
            If fetch/execute results in another fault, abort current
            FAULT CYCLE and initiate new FAULT CYCLE for the trouble
            fault (31).  But do not safe-stroe the CU data.
    
            If either of the two instr in the fault trap pair
            result in transfer of control to ... set absolute 
            indicator to ON.
    
            If either .. results in transfer ..., the transfer is
            made in append mode
    
            If not transfer of control takes place, ... resume at
            instr following the faulting instr ... note ring zero
    
            Most fault pairs are a scu instr followed by a transfer
            instr to a fault analysis routine
    
            If a fault is to be ignored, use a scu/rcu pair with
            a unique Y-block8
            ...
            break;
        case FETCH_cycle:
            debug_msg("CPU::CU", "Cycle = FETCH\n");
            fetch pair of words -- error if pc is odd?
            even to cu_data.IR, odd to cu_data.IRODD;
            cycle = EXEC_cycle;
            CU.hist.IC = 0; // execute odd instr of current pair
            break;
        case EXEC_cycle:
            // todo: Assumption: PC will be at curr instr
            /* check for SIMH breakpoint on execution */
            if (CU.hist.IC == 0) {
                debug_msg("CPU::CU", "Cycle = EXEC, even instr\n");
                if (sim_brk_summ) {
                    if (sim_brk_test (PC, SWMASK ('E'))) {
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                execute_ir();
                if not xfered
                    CU.hist.IC = 1;
            } else {
                debug_msg("CPU::CU", "Cycle = EXEC, odd instr\n");
                // todo: advance PC, maybe move IRODD to IR
                if (sim_brk_summ) {
                    if (sim_brk_test (PC, SWMASK ('E'))) {
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                execute_ir_odd();
                do interrupt/fault checks?
                cycle = FETCH_CYCLE;
            }
            break;
    }
}


//=============================================================================

execute_ir(void)
{
    // execute whatever instruction is in the IR (not PC)
    // IR is word 6 of CU data
}

//=============================================================================

control unit notes
{
    fault detection
        Control unit determines the proper time to initial
        the fault sequence!  At that time, the control unit
        interrupts normal sequential processing with an
        ABORT CYCLE.   The contol unit next initiates a
        FAULT CYCLE.
    abort cycle
        // invoked when control unit decides to handle fault
        bring all overlapped and asynch functions within
        process to an orderly halt
    fault cycle
        Safe store control unit data into invisible registers
        in prep for a store control unit (scu) instr
        Enter temp absolute mode
        force current ring of executionto C(PPR.PRR) to zero
        gen computed addr for fault trap pair
            Setting of FAULT BASE switches and 2x the fault #
        Force computed addr and xed opcode into the instruction
        register and execute (during FAULT CYCLE not EXECUTE CYCLE).

        If fetch/execute results in another fault, abort current
        FAULT CYCLE and initiate new FAULT CYCLE for the trouble
        fault (31).  But do not safe-stroe the CU data.

        If either of the two instr in the fault trap pair
        result in transfer of control to ... set absolute 
        indicator to ON.

        If either .. results in transfer ..., the transfer is
        made in append mode

        If not transfer of control takes place, ... resume at
        instr following the faulting instr ... note ring zero

        Most fault pairs are a scu instr followed by a transfer
        instr to a fault analysis routine

        If a fault is to be ignored, use a scu/rcu pair with
        a unique Y-block8
}
