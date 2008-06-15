/*
    hw6180_cpu.c

    Provides the topmost level of instruction processing, the sim_instr()
    function which fetches instructions and updates the program counter.
    Provides fault and interrupt handling.
    See opu.c for implemention of individual instructions.
    See apu.c for implemention of addressing.
    Provides routines for fetching from memory and storing to memory.
    Also provides the cpu_boot() routine which simulates the way an IOM or
    other hardware would load the first record from a boot tape into memory.
    Provides definitions of most of the data structures related to
    a CPU.
*/


#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "hw6180.h"
#include "sim_tape.h"
#include "bits.h"

// BUG: Make sure all data types referenced in sim_devices[] are sized as the min size for number of bits

//-----------------------------------------------------------------------------
//*** SIMH specific externs

#define MAXMEMSIZE (16*1024*1024)
t_uint64 M[MAXMEMSIZE]; /* memory */
extern int32 sim_interval;
extern uint32 sim_brk_summ;
extern int32 sim_switches;

//-----------------------------------------------------------------------------
//*** Registers

// We don't have a simple PC -- we have HW virtual memory
// PPR pseudo-register is the PC
//  AU:PSR -- segment number (15 bits); not used in absolute mode
//  CU:IC -- offset (18 bits)
//  See also
//      AU:P -- privileged
//      AU:PRR: 3 bits -- ring #
// we should probably use our own addr rtn and provide for handling
// absolute, bar, and append modes


/* OBSOLETE PC COMMENTS */
// todo: Figure this out after fetch and addr code is finished
//
// We don't have a simple PC -- we have a segment, page, and offset
// we should probably use our own addr rtn and provide for handling
// absolute, bar, and append modes
//
// would SIMH accept a reg array?  probably not as good as custom addr rtns
//
//  PPR.IC offset
//  Maybe dump fetched instr to PC?
//  PPR.PSR Procedure segment register [15 bits]
//  Probably the following from addr.c:
/*
       append unit processing
       see section 6 -- formation of a virt mem addr
           we have effective seg num segno and a computed
           addr (offset) in TPR.SNR and TPR.CA
       check segment boundries
       see fig 5-4 flowchart
*/
//  OU history has an copy of the CU ICT (offset from PSR)
//  FYI, APU history  has an ESN effective seg # 
    


t_uint64 reg_A; // Accumulator, 36 bits
t_uint64 reg_Q; // Quotient, 36 bits
// Note: AQ register is just a combination of the A and Q registers
t_uint64 reg_E; // Exponent
// Note: EAQ register is just a combination of the E, A, and Q registers
//t_uint64 reg_X[8];    // Index Registers, 18 bits
uint32 reg_X[8];    // Index Registers, 18 bits; SIMH expects data type to be no larger than needed
IR_t IR;        // Indicator register
static t_uint64 saved_IR;
// static int32 IC; // APU (appending unit) Instruction Counter, 18 bits -- see PPR.IC !!!
static t_uint64 saved_IC;
BAR_reg_t BAR;      // Base Addr Register; 18 bits
//static uint32 saved_BAR;
uint32 reg_TR;      // Timer Reg, 27 bits -- only valid after calls to SIMH clock routines
uint8 reg_RALR; // Ring Alarm Reg, 3 bits
AR_PR_t AR_PR[8];   // Combined Pointer Registers (42 bits) and Address Registers (24 bits)
PPR_t PPR;      // Procedure Pointer Reg, 37 bits, internal only
TPR_t TPR;      // Temporary Pointer Reg, 42 bits, internal only
DSBR_t DSBR;    // Descriptor Segment Base Register, 51 bits
SDWAM_t SDWAM[16];  // Segment Descriptor Word Associative Memory, 88 bits
PTWAM_t PTWAM[16];  // Page Table Word Associative Memory, 51 bits
// static fault_reg_t FR;   // Fault Register, 35 bits
// CMR;     // Cache Mode Register, 28 bits
// CU_hist[16];     // 72 bits
// OU_hist[16];     // 72 bits
// DU_hist[16];     // 72 bits
// APU_hist[16];    // 72 bits
// ConfigSwitch[5];
// CU data  // Control Unit data, 288 bits, internal only
// DU data  // Decimal Unit data, 288 bits, internal only


// SIMH gets a copy of all (or maybe most) registers
// todo: modify simh to take a PV_ZLEFT flag (leftmost bit is bit 0) or
// perhaps PV_ZMSB (most significant bit is numbered zero)
REG cpu_reg[] = {
    // name="PC", loc=PC, radix=<8>, width=36, offset=<0>, depth=<1>, flags=, qptr=
    { ORDATA (IC, saved_IC, 18) },
    { GRDATA (IR, saved_IR, 2, 18, 0), REG_RO },
    { ORDATA (A, reg_A, 36) },
    { ORDATA (Q, reg_Q, 36) },
    { ORDATA (E, reg_E, 36) },
    { BRDATA (X, reg_X, 8, 18, 8) },
    { ORDATA (TR, reg_TR, 27), REG_RO },
    { NULL }
};

//-----------------------------------------------------------------------------
//*** CPU

/* CPU data structures for SIMH

    cpu_dev     CPU device descriptor
    cpu_unit    CPU unit
    cpu_reg     CPU register list
    cpu_mod     CPU modifier list
*/

//static UNIT cpu_unit = {
UNIT cpu_unit = {
    // TODO: idle svc
    UDATA (NULL, UNIT_FIX|UNIT_BINK|UNIT_IDLE, MAXMEMSIZE)
};

static MTAB cpu_mod[] = {
    // for SIMH "show" and "set" commands; Todo: fill in MTAB
    { 0 }
};

t_stat cpu_boot (int32 unit_num, DEVICE *dptr);;
t_stat cpu_reset (DEVICE *dptr);

// SIMH directs requests to examine/change memory to the CPU DEVICE?
static t_stat cpu_ex (t_value *eval_array, t_addr addr, UNIT *uptr, int32 switches);
static t_stat cpu_dep (t_value v, t_addr addr, UNIT *uptr, int32 switches);
DEVICE cpu_dev = {
    // todo: revisit cpu_dev: add debug, examine, deposit, etc
    "CPU", &cpu_unit, cpu_reg, cpu_mod,
    1, 8, 24, 1, 8, 36,                     // should we say 18 for awidth?
    &cpu_ex, &cpu_dep, &cpu_reset,
    &cpu_boot, NULL, NULL,
    NULL, DEV_DEBUG
};

extern t_stat clk_svc(UNIT *up);

UNIT TR_clk_unit = { UDATA(&clk_svc, UNIT_IDLE, 0) };

extern t_stat mt_svc(UNIT *up);
UNIT mt_unit = {
    // one drive
    // NOTE: other SIMH tape sims don't set UNIT_SEQ
    UDATA (&mt_svc, UNIT_ATTABLE | UNIT_SEQ | UNIT_ROABLE | UNIT_DISABLE | UNIT_IDLE, 0)
};

DEVICE tape_dev = {
    "tape", &mt_unit, NULL, NULL,
    // 1, 10, 31, 1, 8, 8,
    1, 10, 31, 1, 8, 9,
    NULL, NULL, NULL,
    NULL, &sim_tape_attach, &sim_tape_detach,
    NULL, DEV_DEBUG
};

DEVICE opcon_dev = {
    "Operator's console", NULL, NULL, NULL,
    0, 10, 8, 1, 8, 8,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, DEV_DEBUG
};

//-----------------------------------------------------------------------------
//*** Other Globals holding state values
//      If SIMH uses known devices to switch between CPUs, we'll have to 
//      add these to cpu_reg as read only (perhaps even hidden) registers.
//      In order to suppport SIMH's save/restore commands, we'll at least
//      have to register a dummy register and examine/deposit routine.

static cycles_t cycle;
static events_t events;
switches_t switches;
cpu_ports_t cpu_ports;
// the following two should probably be combined
cpu_state_t cpu;
ctl_unit_data_t cu; 

scu_t scu;  // only one for now
iom_t iom;  // only one for now
t_bool fault_gen_no_fault;

//-----------------------------------------------------------------------------
//***  Other Externs
int opt_debug;
extern int bootimage_loaded;    // only relevent for the boot CPU ?
extern uint32 sim_emax;


//-----------------------------------------------------------------------------
//***  Constants, unchanging lookup tables, etc

static int fault2group[32] = {
    // from AL39, page 7-3
    7, 4, 5, 5, 7, 4, 5, 4,
    7, 4, 5, 2, 1, 3, 3, 1,
    6, 6, 6, 6, 6, 5, 5, 5,
    5, 5, 0, 0, 0, 0, 0, 2
};
static int fault2prio[32] = {
    // from AL39, page 7-3
    27, 10, 11, 17, 26,  9, 15,  5,
    25,  8, 16,  4,  1,  7,  6,  2,
    20, 21, 22, 23, 24, 12, 13, 14,
    18, 19,  0,  0,  0,  0,  0,  3
};

static int is_eis[1024];    // hack

//-----------------------------------------------------------------------------
//***  Function prototypes

static t_stat control_unit(void);
int fetch_instr(uint IC, instr_t *ip);
void execute_ir(void);
void fault_gen(enum faults f);
void decode_instr(instr_t *ip, t_uint64 word);
static void init_ops();

void tape_block(unsigned char *p, uint32 len, uint32 addr);


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
        debug_msg("sign15", "0%Lo => 0%o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}

//=============================================================================

t_stat cpu_boot (int32 unit_num, DEVICE *dptr)
{
    // Boot -- Copy bootstrap loader into memory & set PC (actually send startup fault)
    // todo: Issue: have to specify boot tape or file out-of-band

    char *fname = "boot.tape";
    int ret;
    uint32 old_switches = sim_switches;
    sim_switches |= SWMASK ('R');   // Read-only -- don't create an empty boot tape
    debug_msg("CPU::boot", "Attaching file %s to tape drive\n", fname);
    ret = sim_tape_attach(&mt_unit, fname);
    sim_switches = old_switches;
    if (ret != 0) {
        complain_msg("CPU::boot", "Cannot attach file '%s' to tape unit.\n", fname);
        return ret;
    }

    const size_t bufsz = 4096 * 1024;
    uint8 buf[bufsz];
    t_mtrlnt tbc;
    if ((ret = sim_tape_rdrecf(&mt_unit, buf, &tbc, bufsz)) != 0) {
        complain_msg("CPU::boot", "Cannot read tape\n");
        return ret;
    }
    warn_msg("CPU::boot", "Read %d bytes from simulated tape %s\n", (int) tbc, fname);
    tape_block(buf, tbc, 030);
    bootimage_loaded = 1;

    return 0;
}

//=============================================================================

t_stat cpu_reset (DEVICE *dptr)
{
    // Reset -- Reset to initial state -- clear all device flags and cancel
    // any outstanding timing operations
    // Used by SIMH's RESET, RUN, and BOOT commands

    // BUG: reset *all* structures to zero
    // Note that SIMH doesn't have much difference between reset and power-on

    // Real hardware had to wait for a connect signal from the SCU before
    // doing anything -- multicians.org glossary
    // Tempting to load "idle until interrupt" into IR.
    // However, we'll use a flag to simulate watching for a control signal
    // being seeing after boot load.   Actually, we could just require
    // the user to load a boot tape image before starting the CPU...

    debug_msg("CPU", "Reset\n");

    init_ops();

    bootimage_loaded = 0;
    memset(&events, 0, sizeof(events));
    memset(&cpu, 0, sizeof(cpu));
    memset(&cu, 0, sizeof(cu));
    memset(&PPR, 0, sizeof(PPR));
    cpu.ic_odd = 0;

    // BUG: reset *all* other structures to zero

    cycle = FETCH_cycle;
    set_addr_mode(ABSOLUTE_mode);
    fault_gen(startup_fault);   // pressing POWER ON button causes this fault

    return 0;
}


//=============================================================================

static int cancel;

void cancel_run(enum sim_stops reason)
{
    // Maybe we should generate an OOB fault?

    (void) sim_cancel_step();
    if (cancel == 0 || reason < cancel)
        cancel = reason;
    debug_msg("CU", "Cancel requested: %d\n", reason);
}

uint32 ninstr;

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
    
    if (! bootimage_loaded) {
        // We probably should not do this
        // See AL70, section 8
        complain_msg("MAIN", "Memory is empty, no bootimage loaded yet\n");
        // return STOP_MEMCLEAR;
    }

    opt_debug = (cpu_dev.dctrl != 0);   // todo: should CPU control all debug settings?

    // BUG: todo: load registers that SIMH user might have modified

    // Setup clocks
    (void) sim_rtcn_init(CLK_TR_HZ, TR_CLK);
    
    int reason = 0;
    cancel = 0;

uint32 ncycles = 0;
ninstr = 0;
    uint32 start = sim_os_msec();
    static t_symtab_ent *source = 0;        // WARNING: re-init this is (re)init() is ever implemented for symtab pkg

    while (reason == 0) {   /* loop until halted */
        if (sim_interval <= 0) { /* check clock queue */
            // process any SIMH timed events including keyboard halt
            if ((reason = sim_process_event()) != 0) break;
        }
#if 0
        uint32 t;
        {
            if ((t = sim_is_active(&TR_clk_unit)) == 0)
                ; // debug_msg("MAIN::clock", "TR is not running\n", t);
            else
                debug_msg("MAIN::clock", "TR is running with %d time units left.\n", t);
        }
#endif
if (opt_debug) {
    // fflush(stdout); printf("\n\r"); fflush(stdout);
    // fflush(stdout); printf("\n"); fflush(stdout);
    printf("\n");
    // debug_msg("MAIN", "IC: %o\n", PPR.IC);
    if (source) {
        if (source->addr_lo <= PPR.IC && PPR.IC <= source->addr_hi)
            debug_msg("MAIN", "IC: %o\n", PPR.IC);  // source unchanged
        else {
            char *old = source->name;
            source = symtab_find(PPR.IC);
            if (source)
                debug_msg("MAIN", "IC: %o\tSource: %s\n", PPR.IC, source->name);
            else
                debug_msg("MAIN", "IC: %o\tSource: Unknown (leaving %s)\n", PPR.IC, old);
        } 
    } else {
        if ((source = symtab_find(PPR.IC)) != NULL)
            debug_msg("MAIN", "IC: %o\tSource: %s\n", PPR.IC, source->name);
        else
            debug_msg("MAIN", "IC: %o\n", PPR.IC);  // still unknown
    }
}
        reason = control_unit();
        ++ ncycles;
        sim_interval--; // todo: maybe only per instr or by brkpoint type?
        if (cancel) {
            if (reason == 0)
                reason = cancel;
        }
    }

    uint32 delta = sim_os_msec() - start;
    //if (delta > 2000)
    if (delta > 200)
        warn_msg("CU", "Step: %.1f seconds: %d cycles at %d cycles/sec, %d instructions at %d instr/sec\n",
            (float) delta / 1000, ncycles, ncycles*1000/delta, ninstr, ninstr*1000/delta);

    // BUG: pack private variables into SIMH's world
    saved_IC = PPR.IC;
    save_IR(&saved_IR);
        
    return reason;
}

void load_IR(t_uint64 word)
{
    IR.zero = getbits36(word, 18, 1);
    IR.neg = getbits36(word, 19, 1);
    IR.carry = getbits36(word, 20, 1);
    IR.overflow = getbits36(word, 21, 1);
    // IR.exp_overflow = getbits36(word, 22, 1);
    // IR.exp_underflow = getbits36(word, 23, 1);
    IR.overflow_mask = getbits36(word, 24, 1);
    IR.tally_runout = getbits36(word, 25, 1);
    // IR.parity_error = getbits36(word, 25, 1);
    // IR.parity_mask = getbits36(word, 27, 1);
    IR.not_bar_mode = getbits36(word, 28, 1);
    IR.truncation = getbits36(word, 29, 1);
    IR.mid_instr_intr_fault = getbits36(word, 30, 1);
    IR.abs_mode = getbits36(word, 31, 1);
    IR.hex_mode = getbits36(word, 32, 1);
}

void save_IR(t_uint64* wordp)
{
    // Saves 14 or 15 IR bits to the lower half of *wordp.
    // Upper half of *wordp zeroed and unused lower bits are zeroed.

    // Note that most of AN87 and AL39 ignores bit 32.  Only section
    // 3 of AL38 describes this bit which is only available on DPS8M.
    // It's also part of the saved control unit data, but not listed
    // in the description of CU data nor the rcu or scu instructions.

    *wordp =
        (IR.zero << (35-18)) |
        (IR.neg << (35-19)) |
        (IR.carry << (35-20)) |
        (IR.overflow << (35-21)) |
        // (IR.exp_overflow << (35-22)) |
        // (IR.exp_underflow << (35-23)) |
        (IR.overflow_mask << (35-24)) |
        (IR.tally_runout << (35-25)) |
        (IR.not_bar_mode << (35-28)) |
        (IR.truncation << (35-29)) |
        (IR.mid_instr_intr_fault << (35-30)) |
        (IR.abs_mode << (35-31)) |
        (IR.hex_mode << (35-32));
}
        

//=============================================================================


static t_stat control_unit(void)
{
    // ------------------------------------------------------------------------
    //
    // TODO: Can we get rid of the simulation of two word fetch of an even/odd
    //  instruction pair?

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
    //  AN87 -- Mode register's overlap inhibit settings

    // ------------------------------------------------------------------------

    // BUG: Check non group 7 faults?  No, expect cycle to have been reset to FAULT_cycle

    int reason = 0;
    int break_on_fault = 1;

    switch(cycle) {
        case FETCH_cycle:
            if (opt_debug) debug_msg("CU", "Cycle = FETCH; IC = %0o (%dd)\n", PPR.IC, PPR.IC);
            // If execution of the current pair is complete, the processor
            // checks two? internal flags for group 7 faults and/or interrupts.
            if (events.any) {
                if (break_on_fault) {
                    warn_msg("CU", "Fault: auto breakpoint\n");
                    (void) cancel_run(STOP_IBKPT);
                }
                if (events.low_group != 0) {
                    // BUG: don't need test below now that we detect 1-6 here
                    debug_msg("CU", "Fault detected prior to FETCH\n");
                    cycle = FAULT_cycle;
                    break;
                }
                if (events.group7 != 0) {
                    // Group 7 -- See tally runout in IR, connect fields of the
                    // fault register.  DC power off must come via an interrupt?
                    debug_msg("CU", "Fault detected prior to FETCH\n");
                    cycle = FAULT_cycle;
                    break;
                }
                if (events.int_pending) {
                    debug_msg("CU", "Interrupt detected prior to FETCH\n");
                    cycle = INTERRUPT_cycle;
                    break;
                }
            }
            // fetch a pair of words
            // todo:  Is cpu.ic_odd unnecessary?  Can we just look at
            // the IC to determine which to execute?
            // AL39, 1-13: for fetches, procedure pointer reg (PPR) is ignored. [PPR IC is a dup of IC]
            cpu.ic_odd = PPR.IC % 2;    // won't exec even if fetch from odd
            // BUG: Fetch both words because if we fault on fetch of even word, we'll
            // handle the fault at the even word and then an EXEC cycle will want to
            // later execute junk from cu.IRODD.   Actually, perhaps fault should always lead to
            // a fetch rather than an EXEC.  On the other hand, our fetch_instr and fetch_word
            // can never fault anyway.  BUG: Update -- Not true -- appending mode can fault
            cycle = EXEC_cycle;
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            if (fetch_instr(PPR.IC - PPR.IC % 2, &cu.IR) != 0)
                cycle = FAULT_cycle;
            if (fetch_word(PPR.IC - PPR.IC % 2 + 1, &cu.IRODD) != 0)
                cycle = FAULT_cycle;
            if (opt_debug && get_addr_mode() != ABSOLUTE_mode)
                debug_msg("CU", "Fetched odd half of instructio pair\n");
            break;

#if 0
    we don't use an ABORT cycle
        case ABORT_cycle:
            debug_msg("CU", "Cycle = ABORT\n");
            // Invoked when control unit decides to handle fault
            // Bring all overlapped functions to an orderly halt -- however,
            // the simulator has no overlapped functions?
            // Also bring asynchronous functions within the processor
            // to an orderly halt -- todo -- do we have any?
            cycle = FAULT_cycle;
            break;
#endif

        case FAULT_cycle:
            // BUG: low_group not used/maintained
            {
            debug_msg("CU", "Cycle = FAULT\n");
            addr_modes_t saved_addr_mode = get_addr_mode();
            set_addr_mode(ABSOLUTE_mode); // until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3

            // find highest fault
            int fault = 0;
            int group;
            for (group = 0; group <= 6; ++ group) {
                if ((fault = events.fault[group]) != 0)
                    break;
            }
            if (fault == 0) {
                if (events.group7 == 0) {
                    // bogus fault
                    complain_msg("CU", "Fault cycle with no faults set\n");
                    reason = STOP_BUG;
                    break;
                } else {
                    // find highest priority group 7 fault
                    int hi = -1;
                    int i;
                    for (i = 0; i < 31; ++i) {
                        if (fault2group[i] == 7 && (events.group7 & (1<<fault)))
                            if (hi == -1 || fault2prio[i] < fault2prio[hi])
                                hi = i;
                    }
                    if (hi == -1) {
                        // no group 7 fault
                        complain_msg("CU", "Fault cycle with missing group-7 fault\n");
                        reason = STOP_BUG;
                        break;
                    } else
                        fault = hi;
                }
            }
            debug_msg("CU", "fault = %d (group %d)\n", fault, group);
            if (fault != trouble_fault) {
                // TODO: Safe store control unit data into invisible registers
                // in prep for a store control unit (scu) instr
            }

            // BUG: clear fault?  Or does scr instr do that?
            int next_fault = 0;
            if (fault == 7) {
                // BUG: clear group 7 fault
                complain_msg("CU", "BUG: Fault group-7\n");
            } else {
                events.fault[group] = 0;
                // Find next remaining fault (and its group)
                for (group = 0; group <= 6; ++ group) {
                    if ((next_fault = events.fault[group]) != 0)
                        break;
                }
            }
            events.any = 0;     // BUG: What about interrupts, other faults, etc?

            PPR.PRR = 0;    // set ring zero
            uint addr = switches.FLT_BASE + 2 * fault; // ABSOLUTE mode
            // Force computed addr and xed opcode into the instruction
            // register and execute (during FAULT CYCLE not EXECUTE CYCLE).
            cu.IR.addr = addr;
            cu.IR.opcode = (opcode0_xed << 1);
            cu.IR.inhibit = 1;
            cu.IR.mods.single.pr_bit = 0;
            cu.IR.mods.single.tag = 0;

            // Maybe just set a flag and run the EXEC case?
            // Maybe the following increments and tests are handled by EXEC and/or the XED opcode?

            // todo: Check for SIMH breakpoint on execution for that addr or
            // maybe in the code for the xed opcode.
            debug_msg("CU", "calling execute_ir() for xed\n");
            uint IC_temp = PPR.IC;
            execute_ir();   // executing in FAULT CYCLE, not EXECUTE CYCLE
            if (break_on_fault) {
                warn_msg("CU", "Fault: auto breakpoint\n");
                (void) cancel_run(STOP_IBKPT);
            }
            if (events.any && events.fault[fault2group[trouble_fault]] == trouble_fault) {
                // Fault occured during execution, so fault_gen() flagged
                // a trouble fault
                // Stay in FAULT CYCLE
                debug_msg("CU", "re-faulted, remaining in fault cycle\n");
            } else {
                // BUG: track events.any
                if (next_fault == 0 && events.group7 == 0) {
                    events.any = events.int_pending;
                    cycle = FETCH_cycle;    // BUG: is this right?
                } else
                    events.any = 1;
                // BUG: kill cpu.xfr
                if (!cpu.xfr && PPR.IC == IC_temp) {
                    warn_msg("CU", "no re-fault, no transfer -- incrementing IC\n");
                    // BUG: Faulted instr doesn't get re-invoked?
                            (void) cancel_run(STOP_IBKPT);
                    ++ PPR.IC;
                    if (cpu.ic_odd) {
                        if (PPR.IC % 2 != 0) { complain_msg("CU", "Fault on odd half of instr pair results in next instr being at an odd IC\n"); }
                        cycle = FETCH_cycle;
                    } else {
                        if (PPR.IC % 2 != 1) { complain_msg("CU", "Fault on even half of instr pair results in odd half pending with an even IC\n"); }
                        cycle = EXEC_cycle;
                    }
                    // Note that we don't automatically return to the original
                    // ring, but hopefully we're executed an rcu instruction
                    // to set the correct ring
                    // set_addr_mode(saved_addr_mode);  // FIXED BUG: this wasn't appropriate
                } else {
                    debug_msg("CU", "no re-fault, but a transfer done -- not incrementing IC\n");
                }
            }
            } // end case FAULT_cycle
            break;

        case INTERRUPT_cycle:
            set_addr_mode(ABSOLUTE_mode); // until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3
            debug_msg("CU", "Interrupts unimplemented\n");
            reason = STOP_BUG;
            break;

        case EXEC_cycle:
            // todo: Assumption: IC will be at curr instr, even
            // when we're ready to execute the odd half of the pair
            // todo: check for IC matching curr instr or at least even/odd sanity
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            if (! cpu.ic_odd) {
                if (opt_debug) debug_msg("CU", "Cycle = EXEC, even instr\n");
                if (sim_brk_summ) {
                    if (sim_brk_test (PPR.IC, SWMASK ('E'))) {
                        warn_msg("CU", "Execution Breakpoint\n");
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                uint IC_temp = PPR.IC;
                (void) execute_ir();
#if 0
                // delay any fault handling for faults by even instr
                if (cycle == EXEC_cycle && !cpu.xfr && PPR.IC == IC_temp) {
                    cpu.ic_odd = 1; // execute odd instr of current pair
                    ++ PPR.IC;
                } else {
                    cycle = FETCH_cycle;
                }
#else
                // Check for fault from even instr
                // todo: simplify --- cycle won't be EXEC anymore
                if (events.any && events.low_group && events.low_group < 7) {
                    // faulted
                    warn_msg("CU", "Probable fault after EXEC even instr\n");
                } else {
                    if (cycle == EXEC_cycle) {
                            if (!cpu.xfr && PPR.IC == IC_temp) {
                                cpu.ic_odd = 1; // execute odd instr of current pair
                                ++ PPR.IC;
                            } else {
                                cycle = FETCH_cycle;
                            }
                    } else {
                        warn_msg("CU", "Changed from EXEC cycle to %d, not updating IC\n", cycle);
                    }
                }
#endif
            } else {
                if (opt_debug) debug_msg("CU", "Cycle = EXEC, odd instr\n");
                // We assume IC was advanced after even instr
                decode_instr(&cu.IR, cu.IRODD);
                if (sim_brk_summ) {
                    if (sim_brk_test (PPR.IC, SWMASK ('E'))) {
                        warn_msg("CU", "Execution Breakpoint\n");
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                uint IC_temp = PPR.IC;
                execute_ir();
                // todo: simplify --- cycle won't be EXEC anymore
                if (events.any && events.low_group && events.low_group < 7) {
                    // faulted
                    warn_msg("CU", "Probable fault after EXEC odd instr\n");
                } else {
                    if (cycle == EXEC_cycle && !cpu.xfr && PPR.IC == IC_temp) {
                        cpu.ic_odd = 0; // finished with odd half; BUG: restart issues?
                        ++ PPR.IC;
                    }
                    cycle = FETCH_cycle;
                }
            }
            break;
        default:
            complain_msg("CU", "Unknown cycle # %d\n", cycle);
            reason = STOP_BUG;
    }

    return reason;
}


//=============================================================================

void execute_ir(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)

    cpu.xfr = 0;
    execute_instr();    // located in opu.c
    ++ ninstr;  // BUG: exec instructions should increase this
}

//=============================================================================

void fault_gen(enum faults f)
{
    int group;
    if (!fault_gen_no_fault)
        events.any = 1;

    if (f == oob_fault) {
        complain_msg("CU::fault", "Faulting for internal bug\n");
        f = trouble_fault;
        (void) cancel_run(STOP_BUG);
    }

    if (f < 1 || f > 32) {
        complain_msg("CU::fault", "Bad fault # %d\n", f);
        abort();
    }
    group = fault2group[f];
    if (fault_gen_no_fault) {
        debug_msg("CU::fault", "Ignoring fault # %d in group %d\n", f, group);
        return;
    }
    debug_msg("CU::fault", "Recording fault # %d in group %d\n", f, group);
    if (group < 1 || group > 7)
        abort();
    // if (f == shutdown_fault || f == timer_fault || f == connect_fault)
    if (group == 7) {
        // Recognition of group 7 faults is delayed and we can have
        // multiple group 7 faults pending.
        events.group7 |= (1 << f);
    } else {
        // Groups 1-6 are handled more immediately and there can only be
        // one fault pending within each group
        if (cycle == FAULT_cycle) {
            f = trouble_fault;
            group = fault2group[f];
            debug_msg("CU::fault", "Double fault:  Recording current fault as a trouble fault (fault # %d in group %d).\n", f, group);
        } else {
            if (events.fault[group]) {
                // todo: error, unhandled fault
                debug_msg("CU::fault", "Found unhandled prior fault #%d in group %d.\n", events.fault[group], group);
            }
            if (cycle == EXEC_cycle) {
                // don't execute any pending odd half of an instruction pair
                cycle = FAULT_cycle;
            }
        }
        events.fault[group] = f;
    }
    if (events.low_group == 0 || group < events.low_group)
        events.low_group = group;   // new highest priority fault group
}

//=============================================================================

int fetch_instr(uint IC, instr_t *ip)
{
    // Returns non-zero if a fault in groups 1-6 is detected
    // todo: check for read breakpoints

    if (get_addr_mode() == ABSOLUTE_mode) {
        if (IC >= ARRAY_SIZE(M)) {
            complain_msg("CU::fetch-instr", "Addr 0%o (%d decimal) is too large\n", IC, IC);
            (void) cancel_run(STOP_BUG);
        }
        decode_instr(ip, M[IC]);    // WARNING: skips fetch_word (but we're in absolute mode)
        return 0;
    }

    t_uint64 word;
    int ret = fetch_word(IC, &word);
    decode_instr(ip, word);
    if (opt_debug)
        debug_msg("CU::fetch-instr", "Fetched word %012Lo => %s\n", word, instr2text(ip));
    return ret;
}

//=============================================================================

int fetch_word(uint addr, t_uint64 *wordp)
{
    // todo: Allow SIMH to use segmented addressing to specify breakpoints. Next, check for such.
    // Returns non-zero if a fault in groups 1-6 is detected

    addr_modes_t mode = get_addr_mode();

    if (mode == APPEND_mode) {
        int ret = fetch_appended(addr, wordp);
        return ret;
    } else if (mode == ABSOLUTE_mode) {
        return fetch_abs_word(addr, wordp);
    } else if (mode == BAR_mode) {
        complain_msg("CU::fetch", "Addr=0%o:  BAR mode unimplemented.\n", addr);
        cancel_run(STOP_BUG);
        return fetch_abs_word(addr, wordp);
    } else {
        complain_msg("CU::fetch", "Addr=0%o:  Unknown addr mode %d.\n", addr, mode);
        cancel_run(STOP_BUG);
        return fetch_abs_word(addr, wordp);
    }

}


int fetch_abs_word(uint addr, t_uint64 *wordp)
{
    // Fetch word at 24-bit absolute address addr.
    // Returns non-zero if a fault in groups 1-6 is detected

    // todo: check for read breakpoints

    // todo: efficiency: combine into a single min/max with sub-tests
    if (addr >= IOM_MBX_LOW && addr < IOM_MBX_LOW + IOM_MBX_LEN) {
        debug_msg("CU::fetch", "Fetch from IOM mailbox area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        debug_msg("CU::fetch", "Fetch from DN355 mailbox area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
#define CONFIG_DECK_LOW 012000
#define CONFIG_DECK_LEN 010000
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        debug_msg("CU::fetch", "Fetch from CONFIG DECK area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr <= 030) {
        debug_msg("CU::fetch", "Fetch from 0..030 for addr 0%o\n", addr);
    }

    if (addr >= ARRAY_SIZE(M)) {
            complain_msg("CU::fetch", "Addr 0%o (%d decimal) is too large\n", addr, addr);
            (void) cancel_run(STOP_BUG);
            return 0;
    }

    if (sim_brk_summ)
        if (sim_brk_test (addr, SWMASK ('M'))) {
            warn_msg("CU::fetch", "Memory Breakpoint, address 0%o\n", addr);
            (void) cancel_run(STOP_IBKPT);
        }

    *wordp = M[addr];   // absolute memory reference
    return 0;
}


int store_word(uint addr, t_uint64 word)
{
    // todo: Allow SIMH to use segmented addressing to specify breakpoints. Next, check for such.
    // Returns non-zero if a fault in groups 1-6 is detected

    addr_modes_t mode = get_addr_mode();

    if (mode == APPEND_mode) {
        int ret = store_appended(addr, word);
        return ret;
    } else if (mode == ABSOLUTE_mode) {
        return store_abs_word(addr, word);
    } else if (mode == BAR_mode) {
        complain_msg("CU::store", "Addr=0%o:  BAR mode unimplemented.\n", addr);
        cancel_run(STOP_BUG);
        return store_abs_word(addr, word);
    } else {
        complain_msg("CU::store", "Addr=0%o:  Unknown addr mode %d.\n", addr, mode);
        cancel_run(STOP_BUG);
        return 1;   // BUG: gen fault
    }
}


int store_abs_word(uint addr, t_uint64 word)
{
    // Store word to location given by 24bit abs memory addr

    // todo: efficiency: combine into a single min/max with sub-tests
    if (addr >= IOM_MBX_LOW && addr < IOM_MBX_LOW + IOM_MBX_LEN) {
        debug_msg("CU::store", "Store to IOM mailbox area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        debug_msg("CU::store", "Store to DN355 mailbox area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        debug_msg("CU::store", "Store to CONFIG DECK area for addr 0%o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr <= 030) {
        //debug_msg("CU::store", "Fetch from 0..030 for addr 0%o\n", addr);
    }
    if (addr >= ARRAY_SIZE(M)) {
            complain_msg("CU::store", "Addr 0%o (%d decimal) is too large\n");
            (void) cancel_run(STOP_BUG);
            return 0;
    }
    if (sim_brk_summ)
        if (sim_brk_test (addr, SWMASK ('M'))) {
            warn_msg("CU::store", "Memory Breakpoint, address 0%o\n", addr);
            (void) cancel_run(STOP_IBKPT);
        }

    M[addr] = word; // absolute memory reference
    return 0;
}

int store_abs_pair(uint addr, t_uint64 word0, t_uint64 word1)
{
    // Store to even and odd words at Y-pair given by 24-bit absolute address addr.
    // Returns non-zero if fault in groups 1-6 detected

    int ret;
    uint Y = (addr % 2 == 0) ? addr : addr - 1;

    if ((ret = store_abs_word(Y, word0)) != 0) {
        return ret;
    }
    if ((ret = store_abs_word(Y+1, word1)) != 0) {
        return ret;
    }
    return 0;
}

int store_pair(uint addr, t_uint64 word0, t_uint64 word1)
{
    // Store to even and odd words at Y-pair given by address addr.
    // Returns non-zero if fault in groups 1-6 detected
    // BUG: Is it the offset that must be even or the final addr that must be even?  (Or both?)

    int ret;
    uint Y = (addr % 2 == 0) ? addr : addr - 1;

    if ((ret = store_word(Y, word0)) != 0) {
        return ret;
    }
    if ((ret = store_word(Y+1, word1)) != 0) {
        return ret;
    }
    return 0;
}


int fetch_abs_pair(uint addr, t_uint64* word0p, t_uint64* word1p)
{
    // Fetch even and odd words at Y-pair given by 24-bit absolute address addr.
    // Returns non-zero if fault in groups 1-6 detected

    int ret;
    uint Y = (addr % 2 == 0) ? addr : addr - 1;

    if ((ret = fetch_abs_word(Y, word0p)) != 0) {
        return ret;
    }
    if ((ret = fetch_abs_word(Y+1, word1p)) != 0) {
        return ret;
    }
    return 0;
}

int fetch_pair(uint addr, t_uint64* word0p, t_uint64* word1p)
{
    // BUG: Is it the offset that must be even or the final addr that must be even?  (Or both?)

    int ret;
    uint Y = (addr % 2 == 0) ? addr : addr - 1;

    if ((ret = fetch_word(Y, word0p)) != 0) {
        return ret;
    }
    if ((ret = fetch_word(Y+1, word1p)) != 0) {
        return ret;
    }
    return 0;
}


int fetch_yblock(uint addr, int aligned, uint n, t_uint64 *wordsp)
{
    // Fetch words from Y-block addr.
    // Returns non-zero if fault in groups 1-6 detected
    // BUG: Is it the offset that must be zero mod n or the final addr that must be zero mod n?  (Or both?)

    int ret;
    uint Y = (aligned) ? (addr / n) * n : Y;

    for (int i = 0; i < n; ++i)
        if ((ret = fetch_word(Y++, wordsp++)) != 0)
            return ret;
    return 0;
}


int fetch_yblock8(uint addr, t_uint64 *wordsp)
{
    return fetch_yblock(addr, 1, 8, wordsp);
}


static int store_yblock(uint addr, int aligned, int n, const t_uint64 *wordsp)
{
    // Store words of to Y-block addr.
    // Returns non-zero if fault in groups 1-6 detected
    // BUG: Is it the offset that must be zero mod n or the final addr that must be zero mod n?  (Or both?)

    int ret;
    uint Y = (aligned) ? (addr / n) * n : Y;

    for (int i = 0; i < n; ++i)
        if ((ret = store_word(Y++, *wordsp++)) != 0)
            return ret;
    return 0;
}

int store_yblock8(uint addr, const t_uint64 *wordsp)
{
    return store_yblock(addr, 1, 8, wordsp);
}

int store_yblock16(uint addr, const t_uint64 *wordsp)
{
    return store_yblock(addr, 1, 16, wordsp);
}



//=============================================================================

void decode_instr(instr_t *ip, t_uint64 word)
{
    ip->addr = getbits36(word, 0, 18);
    ip->opcode = getbits36(word, 18, 10);
    ip->inhibit = getbits36(word, 28, 1);
    if (! (ip->is_eis_multiword = is_eis[ip->opcode])) {
        ip->mods.single.pr_bit = getbits36(word, 29, 1);
        ip->mods.single.tag = getbits36(word, 30, 6);
    } else {
        ip->mods.mf1.ar = getbits36(word, 29, 1);
        ip->mods.mf1.rl = getbits36(word, 30, 1);
        ip->mods.mf1.id = getbits36(word, 31, 1);
        ip->mods.mf1.reg = getbits36(word, 32, 4);
    }
}

//=============================================================================

void anal36 (const char* tag, t_uint64 word);
#include <ctype.h>

void tape_block(unsigned char *p, uint32 len, uint32 addr)
{
    debug_msg("CPU::boot", "Tape block: %u bytes, %u 36-bit words\n", len, len*8/36);
    if ((len * 8) % 36 != 0) {
        complain_msg("CPU::boot", "Length %u bytes is not a multiple of 36 bits.\n");
    }
    bitstream_t *bp = bitstm_new(p, len);
    uint32 nbits = len * 8;
    while (nbits >= 36) {
        bitstm_get(bp, 36, &M[addr++]);     // absolute addresses
        nbits -= 36;
    }
    if (nbits != 0) {
        complain_msg("CPU::boot", "Internal error.   Some bits left over while reading tape\n");
    }
}

//=============================================================================

void anal36 (const char* tag, t_uint64 word)
{
    unsigned char nines[4];
    nines[3] = word & 0777;
    nines[2] = (word >> 9) & 0777;
    nines[1] = (word >> 18) & 0777;
    nines[0] = (word >> 27) & 0777;
    printf("%s: %012Lo octal, %Lu decimal\n", tag, word, word);
    printf("bin64: %s\n", bin2text(word, 64));
    printf("bin36: %s\n", bin2text(word, 36));
    printf("9bits(oct): %03o %03o %03o %03o\n", nines[0], nines[1], nines[2], nines[3]);
    printf("9bits(ascii):");
    int i;
    for (i = 0; i < 4; ++ i) {
        if (isprint(nines[i])) {
            printf(" '%c'", nines[i]);
        } else {
            printf(" \\%03o", nines[i]);
        }
    }
    printf("\n");
}

char *bin2text(t_uint64 word, int n)
{
    // WARNING: static buffer
    static char str1[65];
    static char str2[65];
    static char *str = NULL;
    if (str == NULL)
        str = str1;
    else if (str == str1)
        str = str2;
    else
        str = str1;
    str[n] = 0;
    int i;
    for (i = 0; i < n; ++ i) {
        str[n-i-1] = ((word % 2) == 1) ? '1' : '0';
        word >>= 1;
    }
    return str;
}

//=============================================================================

static t_stat cpu_ex (t_value *eval_array, t_addr addr, UNIT *uptr, int32 switches)
{
    // BUG: sanity check args
    // NOTE: We ignore UNIT, because all CPUS see the same memory
    // SIMH Documention is incorrect; SIMH code expects examine() to only
    // write a single value (often a member of an array of size sim_emax)
    memcpy(eval_array, &M[addr], sizeof(M[0])); // SIMH absolute reference
    return 0;
}

static t_stat cpu_dep (t_value v, t_addr addr, UNIT *uptr, int32 switches)
{
    // BUG: sanity check args
    // NOTE: We ignore UNIT, because all CPUS see the same memory
    M[addr] = v;        // SIMH absolute reference
    return 0;
}

//=============================================================================

static void init_ops()
{
    // hack -- todo: cleanup

    memset(is_eis, 0, sizeof(is_eis));

    is_eis[(opcode1_cmpc<<1)|1] = 1;
    is_eis[(opcode1_scd<<1)|1] = 1;
    is_eis[(opcode1_scdr<<1)|1] = 1;
    is_eis[(opcode1_scm<<1)|1] = 1;
    is_eis[(opcode1_scmr<<1)|1] = 1;
    is_eis[(opcode1_tct<<1)|1] = 1;
    is_eis[(opcode1_tctr<<1)|1] = 1;
    is_eis[(opcode1_mlr<<1)|1] = 1;
    is_eis[(opcode1_mrl<<1)|1] = 1;
    is_eis[(opcode1_mve<<1)|1] = 1;
    is_eis[(opcode1_mvt<<1)|1] = 1;
    is_eis[(opcode1_cmpn<<1)|1] = 1;
    is_eis[(opcode1_mvn<<1)|1] = 1;
    is_eis[(opcode1_mvne<<1)|1] = 1;
    is_eis[(opcode1_csl<<1)|1] = 1;
    is_eis[(opcode1_csr<<1)|1] = 1;
    is_eis[(opcode1_cmpb<<1)|1] = 1;
    is_eis[(opcode1_sztl<<1)|1] = 1;
    is_eis[(opcode1_sztr<<1)|1] = 1;
}
