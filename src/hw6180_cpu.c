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

#define getbit18(x,n)  ((((x) >> (17-n)) & 1) != 0) // return nth bit of an 18bit half word

// BUG: Make sure all data types referenced in sim_devices[] are sized as the min size for number of bits

//-----------------------------------------------------------------------------
// *** SIMH specific externs

#define MAXMEMSIZE (16*1024*1024)
t_uint64 M[MAXMEMSIZE]; /* memory */
extern int32 sim_interval;
extern uint32 sim_brk_summ;
extern int32 sim_switches;

//-----------------------------------------------------------------------------
// *** Registers

// We don't have a simple program counter -- we have HW virtual memory
// PPR pseudo-register is the program counter.   The PPR is composed of
// registers from the appending unit and the control unit:
//  AU:PSR -- segment number (15 bits); not used in absolute mode
//  CU:IC -- offset (18 bits)
//  See also
//      AU:P -- privileged
//      AU:PRR: 3 bits -- ring #
// We should probably provide SIMH with our own addr rtn and provide for
// handling absolute, bar, and append modes 


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
int8 reg_E; // Exponent
// Note: EAQ register is just a combination of the E, A, and Q registers
uint32 reg_X[8];    // Index Registers, 18 bits; SIMH expects data type to be no larger than needed
IR_t IR;        // Indicator register
static t_uint64 saved_IR;   // Only for sending to/from SIMH
static t_uint64 saved_IC;   // Only for sending to/from SIMH; saved_IC address also stored in sim_PC external
BAR_reg_t BAR;      // Base Addr Register; 18 bits
static uint16 saved_BAR[2];
uint32 reg_TR;      // Timer Reg, 27 bits -- only valid after calls to SIMH clock routines
uint8 reg_RALR; // Ring Alarm Reg, 3 bits
AR_PR_t AR_PR[8];   // Combined Pointer Registers (42 bits) and Address Registers (24 bits)
    // Note that the eight PR registers are also known by the names: ap, ab, bp, bb, lp, lb, sp, sb
t_uint64 saved_ar_pr[8];    // packed versions of the AR/PR registers; format specific to this simulator
PPR_t PPR;      // Procedure Pointer Reg, 37 bits, internal only
static t_uint64 saved_PPR;
TPR_t TPR;      // Temporary Pointer Reg, 42 bits, internal only
static t_uint64 saved_DSBR;     // Descriptor Segment Base Register, 51 bits
// SDWAM_t SDWAM[16];   // Segment Descriptor Word Associative Memory, 88 bits
// PTWAM_t PTWAM[16];   // Page Table Word Associative Memory, 51 bits
// static fault_reg_t FR;   // Fault Register, 35 bits
// CMR;     // Cache Mode Register, 28 bits
// CU_hist[16];     // 72 bits
// OU_hist[16];     // 72 bits
// DU_hist[16];     // 72 bits
// APU_hist[16];    // 72 bits
// ConfigSwitch[5];
// CU data  // Control Unit data, 288 bits, internal only
// DU data  // Decimal Unit data, 288 bits, internal only

// This is a hack
static cpu_t cpu_info;
cpu_t *cpup = &cpu_info;

// SIMH gets a copy of all (or maybe most) registers
// todo: modify simh to take a PV_ZLEFT flag (leftmost bit is bit 0) or
// perhaps PV_ZMSB (most significant bit is numbered zero)
REG cpu_reg[] = {
    // structure members: name="PC", loc=PC, radix=<8>, width=36, offset=<0>, depth=<1>, flags=, qptr=
    { ORDATA (IC, saved_IC, 18) },      // saved_IC address also stored in sim_PC external
    { GRDATA (IR, saved_IR, 2, 18, 0), REG_RO | REG_VMIO | REG_USER1},
    { ORDATA (A, reg_A, 36) },
    { ORDATA (Q, reg_Q, 36) },
    { ORDATA (E, reg_E, 8) },
    { BRDATA (X, reg_X, 8, 18, 8) },
    { ORDATA (TR, reg_TR, 27), REG_RO },
    { BRDATA (PR, saved_ar_pr, 8, 42, 8), REG_VMIO | REG_USER2 },
    // stuff needed to yield a save/restore sufficent for examining memory dumps
    { BRDATA (BAR, saved_BAR, 8, 9, 2) },
    { ORDATA (PPR, saved_PPR, 37) },
    { ORDATA (DSBR, saved_DSBR, 51) },
    // The following is a hack, but works as long as you don't save/restore across
    // different architectures.
        { BRDATA (CPUINFO, (&cpu_info), 8, 8, sizeof(cpu_info)) },
    { NULL }
};

//-----------------------------------------------------------------------------
// *** CPU

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

t_stat cpu_boot (int32 unit_num, DEVICE *dptr);
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
    "TAPE", &mt_unit, NULL, NULL,
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
// *** Other Globals holding state values
//      If SIMH uses known devices to switch between CPUs, we'll have to 
//      add these to cpu_reg as read only (perhaps even hidden) registers.
//      In order to suppport SIMH's save/restore commands, we'll at least
//      have to register a dummy register and examine/deposit routine.

events_t events;
switches_t switches;
cpu_ports_t cpu_ports;
// the following two should probably be combined
cpu_state_t cpu;
ctl_unit_data_t cu; 

scu_t scu;  // only one for now
iom_t iom;  // only one for now
flag_t fault_gen_no_fault;

// *** Other variables -- These do not need to be part of save/restore
// static int seg_debug[n_segments];

//-----------------------------------------------------------------------------
// ***  Other Externs
int opt_debug;
extern int bootimage_loaded;    // only relevent for the boot CPU ?
extern uint32 sim_emax;
t_uint64 calendar_a;
t_uint64 calendar_q;

//-----------------------------------------------------------------------------
// ***  Constants, unchanging lookup tables, etc

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
// ***  Function prototypes

static t_stat control_unit(void);
int fetch_instr(uint IC, instr_t *ip);
void execute_ir(void);
void fault_gen(enum faults f);
void decode_instr(instr_t *ip, t_uint64 word);
static void init_ops(void);
static void check_events(void);
static void save_to_simh(void);
static void save_PR_registers(void);
static void restore_PR_registers(void);

void tape_block(unsigned char *p, uint32 len, uint32 addr);

//=============================================================================

#if 0
static int32 sign18(t_uint64 x)
{
    if (bit18_is_neg(x)) {
        int32 r = - ((1<<18) - (x&MASK18));
        return r;
    }
    else
        return x;
}
#endif

#if 0
static int32 sign15(uint x)
{
    if (bit_is_neg(x,15)) {
        int32 r = - ((1<<15) - (x&MASKBITS(15)));
        // log_msg(DEBUG_MSG, "sign15", "%#%#llo => %#o (%+d decimal)\n", x, r, r);
        return r;
    }
    else
        return x;
}
#endif

//=============================================================================

t_stat cpu_boot (int32 unit_num, DEVICE *dptr)
{
    // Boot -- Copy bootstrap loader into memory & set PC (actually send startup fault)
    // todo: Issue: have to specify boot tape or file out-of-band


#if 0
    char *fname = "boot.tape";
    int ret;
    uint32 old_switches = sim_switches;
    sim_switches |= SWMASK ('R');   // Read-only -- don't create an empty boot tape
    log_msg(DEBUG_MSG, "CPU::boot", "Attaching file %s to tape drive\n", fname);
    ret = sim_tape_attach(&mt_unit, fname);
    sim_switches = old_switches;
    if (ret != 0) {
        log_msg(ERR_MSG, "CPU::boot", "Cannot attach file '%s' to tape unit.\n", fname);
        return ret;
    }
#endif

#if 0
    // When using an IOM, we can get away with reading the tape with no involvement
    // from the IOM.   We just need to leave the virtual tape positioned past the
    // first block.
    const size_t bufsz = 4096 * 1024;
    uint8 buf[bufsz];
    t_mtrlnt tbc;
    if ((ret = sim_tape_rdrecf(&mt_unit, buf, &tbc, bufsz)) != 0) {
        log_msg(ERR_MSG, "CPU::boot", "Cannot read tape\n");
        return ret;
    }
    log_msg(NOTIFY_MSG, "CPU::boot", "Read %d bytes from simulated tape %s\n", (int) tbc, fname);
    tape_block(buf, tbc, 030);
    bootimage_loaded = 1;
#else
    // When using an IOX, the bootload_tape_label.alm code will check the result in the
    // status mailbox.   So, we might as well run the IOX to do the I/O.
    // ++ opt_debug; ++ cpu_dev.dctrl;
    if (cpu_dev.dctrl != 0) opt_debug = 1;  // todo: should CPU control all debug settings?
    iom_interrupt();
    // -- opt_debug; -- cpu_dev.dctrl;
    bootimage_loaded = 1;
#endif

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

    log_msg(DEBUG_MSG, "CPU", "Reset\n");
if(0) {
    // BUG -- temp debug hack
    out_msg("DEBUG: CPU Registers:\n");
    for (int i = 0; i < ARRAY_SIZE(cpu_reg); ++i) {
        REG *r = &cpu_reg[i];
        out_msg("\tRegister %d at %p: '%s': radix %d, width %d, depth %d, offset bit %d, flags %0o, loc %p, qptr %d\n",
            i, r, r->name, r->radix, r->width, r->depth, r->offset, r->flags, r->loc, r->qptr);
    }
    out_msg("\n");
}

    init_ops();
    ic_history_init();

    bootimage_loaded = 0;
    memset(&events, 0, sizeof(events));
    memset(&cpu, 0, sizeof(cpu));
    memset(&cu, 0, sizeof(cu));
    memset(&PPR, 0, sizeof(PPR));
    cu.SD_ON = 1;
    cu.PT_ON = 1;
    cpu.ic_odd = 0;

    // BUG: reset *all* other structures to zero

    set_addr_mode(ABSOLUTE_mode);

    // We need to execute the pair of instructions at 030 -- a "lda" instruction
    // and a transfer to the bootload code at 0330.   Location 030 is either part
    // of the interrupt vector or part of a combined interrupt/fault vector.
    if (0 && switches.FLT_BASE == 0) {
        // Most documents indicate that the boot process uses the startup fault
        cpu.cycle = FETCH_cycle;
        fault_gen(startup_fault);   // pressing POWER ON button causes this fault
    } else {
        // Simulate an interrupt as described in bootload_tape_label.alm
        cpu.cycle = INTERRUPT_cycle;
        events.int_pending = 1;
        events.interrupts[12] = 1;
    }

    calendar_a = 0xdeadbeef;
    calendar_q = 0xdeadbeef;

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
    log_msg(DEBUG_MSG, "CU", "Cancel requested: %d\n", reason);
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
    
    restore_from_simh();

    if (! bootimage_loaded) {
        // We probably should not do this
        // See AL70, section 8
        log_msg(WARN_MSG, "MAIN", "Memory is empty, no bootimage loaded yet\n");
        // return STOP_MEMCLEAR;
    }

    // opt_debug = (cpu_dev.dctrl != 0);    // todo: should CPU control all debug settings?

    // BUG: todo: load registers that SIMH user might have modified

    // Setup clocks
    (void) sim_rtcn_init(CLK_TR_HZ, TR_CLK);
    
    int reason = 0;
    cancel = 0;

uint32 ncycles = 0;
ninstr = 0;
    uint32 start = sim_os_msec();

    if (opt_debug && sim_interval > 32) {
        // debug mode is slow, so be more responsive to keyboard interrupt
        sim_interval = 32;
    }
    int prev_seg = PPR.PSR;
    while (reason == 0) {   /* loop until halted */
        if (PPR.PSR != prev_seg) {
            check_seg_debug();
            prev_seg = PPR.PSR;
        }
        if (sim_interval <= 0) { /* check clock queue */
            // process any SIMH timed events including keyboard halt
            if ((reason = sim_process_event()) != 0) break;
        }
#if 0
        uint32 t;
        {
            if ((t = sim_is_active(&TR_clk_unit)) == 0)
                ; // log_msg(DEBUG_MSG, "MAIN::clock", "TR is not running\n", t);
            else
                log_msg(DEBUG_MSG, "MAIN::clock", "TR is running with %d time units left.\n", t);
        }
#endif

        //
        // Log a message about where we're at -- if debugging, or if we want procedure call tracing,
        // or if we're displaying source code
        //

        const int show_source_lines = 1;
        if (opt_debug || show_source_lines)
            show_location(show_source_lines);

        //
        // Execute instructions (or fault or whatever)
        //

        reason = control_unit();

        if (opt_debug || show_source_lines) {
            log_ignore_ic_change();
            show_variables();
            log_notice_ic_change();
        }

        //
        // And record history, etc
        //

        ++ ncycles;
        sim_interval--; // todo: maybe only per instr or by brkpoint type?
        if (opt_debug) {
            log_ignore_ic_change();
            state_dump_changes();
            log_notice_ic_change();
            state_save();   // no stack or queue, just a single backup but all regs, but not just ic history
        }
        if (cancel) {
            if (reason == 0)
                reason = cancel;
        }
#if 0
        if (reason == 0 && PPR.IC == 014256 && reg_A == (t_uint64) 0777771000000) {
            log_msg(NOTIFY_MSG, "MAIN", "AutoBreakpoint for debug tape\n");
            reason = STOP_IBKPT;
        }
#endif
    }

    uint32 delta = sim_os_msec() - start;
    //if (delta > 2000)
    if (delta > 200)
        log_msg(NOTIFY_MSG, "CU", "Step: %.1f seconds: %d cycles at %d cycles/sec, %d instructions at %d instr/sec\n",
            (float) delta / 1000, ncycles, ncycles*1000/delta, ninstr, ninstr*1000/delta);

    // BUG: pack private variables into SIMH's world
    save_to_simh();
        
    return reason;
}


static void save_to_simh(void)
{
    saved_IC = PPR.IC;
    save_IR(&saved_IR);
    save_PR_registers();
if (0)
{
    AR_PR_t sv[8];
    memcpy(sv, AR_PR, sizeof(sv));
    restore_PR_registers();
    if (memcmp(sv, AR_PR, sizeof(sv)) != 0) {
        out_msg("CPU: WARNING: PR[] save/restore broken.  Fixing...\n");
        (void) cancel_run(STOP_WARN);
        memcpy(AR_PR, sv, sizeof(AR_PR));
    }
}

    saved_BAR[0] = BAR.base;
    saved_BAR[1] = BAR.bound;
    saved_PPR = (PPR.IC) | // 18 bits
        (PPR.P << 18) | // 1 bits
        (PPR.PRR << 19) | // 3 bits
        ((t_uint64) PPR.PSR << 22); // 15 bits
    saved_DSBR = 
        cpup->DSBR.stack | // 12 bits
        (cpup->DSBR.u << 12) | // 1 bit
        ((t_uint64) cpup->DSBR.bound << 13) | // 14 bits
        ((t_uint64) cpup->DSBR.addr << 27); // 24 bits
}

void restore_from_simh(void)
{

    PPR.IC = saved_IC;
    load_IR(&IR, saved_IR);

    restore_PR_registers();
    BAR.base = saved_BAR[0];
    BAR.bound = saved_BAR[1];
    // PPR.IC = saved_PPR & MASK18;
    PPR.P = (saved_PPR >> 18) & 1;
    PPR.PRR = (saved_PPR >> 19) & 7;
    PPR.PSR = (saved_PPR >> 22);
    cpup->DSBR.stack = saved_DSBR & MASKBITS(12);
    cpup->DSBR.u = (saved_DSBR >> 12) & 1;
    cpup->DSBR.bound = (saved_DSBR >> 13) & MASKBITS(14);
    cpup->DSBR.addr = (saved_DSBR >> 27) & MASKBITS(24);

    // Set default debug and check for a per-segment debug override
    check_seg_debug();
}

void load_IR(IR_t *irp, t_uint64 word)
{
    memset(irp, 0, sizeof(*irp));
    irp->zero = getbits36(word, 18, 1);
    irp->neg = getbits36(word, 19, 1);
    irp->carry = getbits36(word, 20, 1);
    irp->overflow = getbits36(word, 21, 1);
    irp->exp_overflow = getbits36(word, 22, 1);
    irp->exp_underflow = getbits36(word, 23, 1);
    irp->overflow_mask = getbits36(word, 24, 1);
    irp->tally_runout = getbits36(word, 25, 1);
    irp->parity_error = getbits36(word, 26, 1);
    irp->parity_mask = getbits36(word, 27, 1);
    irp->not_bar_mode = getbits36(word, 28, 1);
    irp->truncation = getbits36(word, 29, 1);
    irp->mid_instr_intr_fault = getbits36(word, 30, 1);
    irp->abs_mode = getbits36(word, 31, 1);
    irp->hex_mode = getbits36(word, 32, 1);
    // Bits 33..35 not used
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
        (IR.exp_overflow << (35-22)) |
        (IR.exp_underflow << (35-23)) |
        (IR.overflow_mask << (35-24)) |
        (IR.tally_runout << (35-25)) |
        (IR.parity_error << (35-26)) |
        (IR.parity_mask << (35-27)) |
        (IR.not_bar_mode << (35-28)) |
        (IR.truncation << (35-29)) |
        (IR.mid_instr_intr_fault << (35-30)) |
        (IR.abs_mode << (35-31)) |
        (IR.hex_mode << (35-32));
}
        
//=============================================================================

static void save_PR_registers()
{
    for (int i = 0; i < ARRAY_SIZE(saved_ar_pr); ++ i) {
        saved_ar_pr[i] =
            (AR_PR[i].PR.snr & 077777) |            // 15 bits
            ((AR_PR[i].PR.rnr & 07) << 15) |        //  3 bits
            ((AR_PR[i].PR.bitno & 077) << 18) |     //  6 bits
            ((t_uint64)(AR_PR[i].wordno & 0777777) << 24);  // 18 bits
    }
}

static void restore_PR_registers(void)
{
    for (int i = 0; i < ARRAY_SIZE(AR_PR); ++ i) {
        AR_PR[i].PR.snr = saved_ar_pr[i] & MASKBITS(15);
        AR_PR[i].PR.rnr = (saved_ar_pr[i] >> 15) & MASKBITS(3);
        AR_PR[i].PR.bitno = (saved_ar_pr[i] >> 18) & MASKBITS(6);
        AR_PR[i].AR.charno =  AR_PR[i].PR.bitno / 9;
        AR_PR[i].AR.bitno =  AR_PR[i].PR.bitno % 9;
        AR_PR[i].wordno = (saved_ar_pr[i] >> 24);
    }
}

//=============================================================================

#define XED_NEW 1

static t_stat control_unit(void)
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
    //  AN87 -- Mode register's overlap inhibit settings

    // ------------------------------------------------------------------------

    // BUG: Check non group 7 faults?  No, expect cycle to have been reset to FAULT_cycle

    int reason = 0;
    int break_on_fault = switches.FLT_BASE == 2;    // on for multics, off for t&d tape

    switch(cpu.cycle) {
        case FETCH_cycle:
            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Cycle = FETCH; IC = %0o (%dd)\n", PPR.IC, PPR.IC);
            // If execution of the current pair is complete, the processor
            // checks two? internal flags for group 7 faults and/or interrupts.
            if (events.any) {
                if (break_on_fault) {
                    log_msg(WARN_MSG, "CU", "Fault: auto breakpoint\n");
                    (void) cancel_run(STOP_IBKPT);
                }
                if (events.low_group != 0) {
                    // BUG: don't need test below now that we detect 1-6 here
                    if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Fault detected prior to FETCH\n");
                    cpu.cycle = FAULT_cycle;
                    break;
                }
                if (events.group7 != 0) {
                    // Group 7 -- See tally runout in IR, connect fields of the
                    // fault register.  DC power off must come via an interrupt?
                    if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Fault detected prior to FETCH\n");
                    cpu.cycle = FAULT_cycle;
                    break;
                }
                if (events.int_pending) {
                    if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Interrupt detected prior to FETCH\n");
                    cpu.cycle = INTERRUPT_cycle;
                    break;
                }
            }
            // fetch a pair of words
            // AL39, 1-13: for fetches, procedure pointer reg (PPR) is ignored. [PPR IC is a dup of IC]
            cpu.ic_odd = PPR.IC % 2;    // won't exec even if fetch from odd
            cpu.cycle = EXEC_cycle;
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            cu.instr_fetch = 1;
            if (fetch_instr(PPR.IC - PPR.IC % 2, &cu.IR) != 0) {
                cpu.cycle = FAULT_cycle;
                cpu.irodd_invalid = 1;
            } else {
                if (sim_brk_summ && sim_brk_test (cpu.read_addr, SWMASK ('E'))) {
                    log_msg(WARN_MSG, "CU", "Execution Breakpoint\n");
                    reason = STOP_IBKPT;    /* stop simulation */
                }
                if (fetch_word(PPR.IC - PPR.IC % 2 + 1, &cu.IRODD) != 0) {
                    cpu.cycle = FAULT_cycle;
                    cpu.irodd_invalid = 1;
                } else {
                    if (sim_brk_summ && sim_brk_test (cpu.read_addr, SWMASK ('E'))) {
                        log_msg(WARN_MSG, "CU", "Execution Breakpoint\n");
                        reason = STOP_IBKPT;    /* stop simulation */
                    }
                    cpu.irodd_invalid = 0;
                    if (opt_debug && get_addr_mode() != ABSOLUTE_mode)
                        log_msg(DEBUG_MSG, "CU", "Fetched odd half of instruction pair from %06o\n", PPR.IC - PPR.IC % 2 + 1);
                }
            }
            cpu.IC_abs = cpu.read_addr;
            cu.instr_fetch = 0;
            break;

#if 0
    // we don't use an ABORT cycle
        case ABORT_cycle:
            log_msg(DEBUG_MSG, "CU", "Cycle = ABORT\n");
            // Invoked when control unit decides to handle fault
            // Bring all overlapped functions to an orderly halt -- however,
            // the simulator has no overlapped functions?
            // Also bring asynchronous functions within the processor
            // to an orderly halt -- todo -- do we have any?
            cpu.cycle = FAULT_cycle;
            break;
#endif

        case FAULT_cycle:
            {
            log_msg(DEBUG_MSG, "CU", "Cycle = FAULT\n");

            // find highest fault
            int fault = 0;
            int group = 0;
#if 0
            // BUG: low_group not used/maintained
            for (group = 0; group <= 6; ++ group) {
                if ((fault = events.fault[group]) != 0)
                    break;
            }
#else
            if (events.low_group != 0 && events.low_group <= 6) {
                group = events.low_group;
                fault = events.fault[group];
                if (fault == 0) {
                    log_msg(ERR_MSG, "CU", "Lost fault\n");
                    cancel_run(STOP_BUG);
                }
            }
#endif
            if (fault == 0) {
                if (events.group7 == 0) {
                    // bogus fault
                    log_msg(ERR_MSG, "CU", "Fault cycle with no faults set\n");
                    reason = STOP_BUG;
                    events.any = events.int_pending;    // recover
                    cpu.cycle = FETCH_cycle;
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
                        log_msg(ERR_MSG, "CU", "Fault cycle with missing group-7 fault\n");
                        reason = STOP_BUG;
                        events.any = events.int_pending;    // recover
                        cpu.cycle = FETCH_cycle;
                        break;
                    } else {
                        fault = hi;
                        group = 7;
                    }
                }
            }
            log_msg(DEBUG_MSG, "CU", "fault = %d (group %d)\n", fault, group);
            if (fault != trouble_fault)
                cu_safe_store();

            set_addr_mode(ABSOLUTE_mode); // this mode will remain in effect until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3

            // BUG: clear fault?  Or does scr instr do that?
            int next_fault = 0;
            if (group == 7) {
                // BUG: clear group 7 fault
                log_msg(ERR_MSG, "CU", "BUG: Fault group-7\n");
            } else {
                events.fault[group] = 0;
                // Find next remaining fault (and its group)
                events.low_group = 0;
                for (group = 0; group <= 6; ++ group) {
                    if ((next_fault = events.fault[group]) != 0) {
                        events.low_group = group;
                        break;
                    }
                }
                if (! events.low_group)
                    if (events.group7 != 0)
                        events.low_group = 7;
            }
            events.any = events.int_pending || events.low_group != 0;

            PPR.PRR = 0;    // set ring zero
            uint addr = (switches.FLT_BASE << 5) + 2 * fault; // ABSOLUTE mode
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
            log_msg(DEBUG_MSG, "CU::fault", "calling execute_ir() for xed\n");
            uint IC_temp = PPR.IC;
            PPR.IC = addr;
            ic_history_add();       // xed will be listed against the the addr of the first instruction
            PPR.IC = IC_temp;
            // BUG: don't we need another ic_history_add() call here?
            execute_ir();   // executing in FAULT CYCLE, not EXECUTE CYCLE
            if (break_on_fault) {
                log_msg(WARN_MSG, "CU", "Fault: auto breakpoint\n");
                (void) cancel_run(STOP_IBKPT);
            }

            if (events.any && events.fault[fault2group[trouble_fault]] == trouble_fault) {
                // Fault occured during execution, so fault_gen() flagged
                // a trouble fault
                // Stay in FAULT CYCLE
                log_msg(WARN_MSG, "CU", "re-faulted, remaining in fault cycle\n");
            } else {
#if XED_NEW
                // cycle = FAULT_EXEC_cycle;
                cpu.cycle = EXEC_cycle;     // NOTE: scu will be in EXEC not FAULT cycle
                events.xed = 1;     // BUG: is this a hack?
#else
                // BUG: track events.any
                if (next_fault == 0 && events.group7 == 0) {
                    events.any = events.int_pending;
                    cpu.cycle = FETCH_cycle;    // BUG: is this right?
                } else
                    events.any = 1;
                // BUG: kill cpu.trgo or IC_temp
                if (!cpu.trgo && PPR.IC == IC_temp) {
                    log_msg(WARN_MSG, "CU", "no re-fault, no transfer -- incrementing IC\n");
                    // BUG: Faulted instr doesn't get re-invoked?
                            (void) cancel_run(STOP_IBKPT);
                    ++ PPR.IC;
                    if (cpu.ic_odd) {
                        if (PPR.IC % 2 != 0) { log_msg(ERR_MSG, "CU", "Fault on odd half of instr pair results in next instr being at an odd IC\n"); }
                        cpu.cycle = FETCH_cycle;
                    } else {
                        if (PPR.IC % 2 != 1) { log_msg(ERR_MSG, "CU", "Fault on even half of instr pair results in odd half pending with an even IC\n"); }
                        cpu.cycle = EXEC_cycle;
                    }
                    // Note that we don't automatically return to the original
                    // ring, but hopefully we're executed an rcu instruction
                    // to set the correct ring
                    // set_addr_mode(saved_addr_mode);  // FIXED BUG: this wasn't appropriate
                } else {
                    if (opt_debug) log_msg(DEBUG_MSG, "CU", "no re-fault, but a transfer done -- not incrementing IC\n");
                }
#endif
            }
            } // end case FAULT_cycle
            break;

        case INTERRUPT_cycle: {
            // This code is just a quick-n-dirty sketch to test booting via an interrupt instead of via a fault
            set_addr_mode(ABSOLUTE_mode); // this mode will remain in effect until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3
            log_msg(WARN_MSG, "CU", "Interrupts only partially implemented\n");
            PPR.PRR = 0;    // set ring zero
            int intr;
            for (intr = 0; intr < 32; ++intr)
                if (events.interrupts[intr])
                    break;
            if (intr == 32) {
                log_msg(ERR_MSG, "CU", "Interrupt cycle with no pending interrupt.\n");
                // BUG: Need error handling
            }
            log_msg(WARN_MSG, "CU", "Interrupt %#o (%d) found.\n", intr, intr);
            events.interrupts[intr] = 0;
            uint addr = 0 + 2 * intr; // ABSOLUTE mode
            // Force computed addr and xed opcode into the instruction
            // register and execute (during FAULT CYCLE not EXECUTE CYCLE).
            cu.IR.addr = addr;
            cu.IR.opcode = (opcode0_xed << 1);
            cu.IR.inhibit = 1;
            cu.IR.mods.single.pr_bit = 0;
            cu.IR.mods.single.tag = 0;
            reason = STOP_BUG;

            log_msg(DEBUG_MSG, "CU::interrupt", "calling execute_ir() for xed\n");
            uint IC_temp = PPR.IC;
            PPR.IC = addr;
            ic_history_add();
            PPR.IC = IC_temp;
            // BUG: don't we need another ic_history_add() call here?
            execute_ir();   // executing in INTERRUPT CYCLE, not EXECUTE CYCLE

#if XED_NEW
            // cpu.cycle = FAULT_EXEC_cycle;
            cpu.cycle = EXEC_cycle;
            events.xed = 1;     // BUG: is this a hack?
            events.int_pending = 0;     // BUG: make this a counter
            for (intr = 0; intr < 32; ++intr)
                if (events.interrupts[intr]) {
                    events.int_pending = 1;
                    break;
                }
#else
            cpu.cycle = FETCH_cycle;    // BUG: is this right?
            log_msg(ERR_MSG, "CU", "Interrupts not well tested\n");
#endif
            break;
        }

        case EXEC_cycle:
            // Assumption: IC will be at curr instr, even
            // when we're ready to execute the odd half of the pair.
            // Note that the fetch cycle sets the cpu.ic_odd flag.
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            
            // Possibly obsolete notes:
            // handle rpt and other repeat instructions
            // -----
            // don't switch to fetch cycle or update ic_odd except as noted here
            // pre-exec: nothing special needed?  Check ic_odd and exec appropriate instr?
            // post-exec, first: set fetch if needed
                // rpt@odd: fetch pair (if first -- note this matches non-rpt behavior)
                // rpt-db@odd: always fetch pair (if first -- note this matches non-rpt behavior)
                // rpt-dbl@even: always fetch pair (if first)
                // rpt@even: expect to execute odd ( note this matches non-rpt behavior )
            // post-exec, not first: rpt, IC unchanged; rpt-dbl: IC swaps even/odd
            // post-exec: check termination
            // faults and interrupts ???

        // Fall through -- FAULT_EXEC_cycle is a subset of EXEC_cycle
    
#if XED_NEW
        case FAULT_EXEC_cycle: ;
            // FAULT-EXEC is a pseudo cycle not present in the actual hardware.   The hardware
            // does execute intructions (e.g. an fault's xed) in a FAULT cycle.   We use the
            // FAULT-EXEC cycle for this to gain code re-use.
#endif

            flag_t do_odd = 0;

            int doing_xde = cu.xde;
            int doing_xdo = cu.xdo;
            if (doing_xde)
                log_msg(NOTIFY_MSG, "CU", "XDE-EXEC even\n");
            else if (doing_xdo) {
                log_msg(NOTIFY_MSG, "CU", "XDE-EXEC odd\n");
                do_odd = 1;
            } else if (! cpu.ic_odd) {
                if (opt_debug) log_msg(DEBUG_MSG, "CU", "Cycle = EXEC, even instr\n");
            } else {
                if (opt_debug) log_msg(DEBUG_MSG, "CU", "Cycle = EXEC, odd instr\n");
                do_odd = 1;
            }
            if (do_odd) {
                if (cpu.irodd_invalid) {
                    cpu.irodd_invalid = 0;
                    if (cpu.cycle != FETCH_cycle) {
                        if (switches.FLT_BASE == 2) {   // on for multics, off for t&d tape
                            reason = STOP_IBKPT;    /* stop simulation */
                            log_msg(NOTIFY_MSG, "CU", "Invalidating cached odd instruction; auto breakpoint\n");
                        } else
                            log_msg(NOTIFY_MSG, "CU", "Invalidating cached odd instruction.\n");
                        cpu.cycle = FETCH_cycle;
                    }
                    break;
                }
                decode_instr(&cu.IR, cu.IRODD);
            }

            if (sim_brk_summ) {
                if (sim_brk_test (PPR.IC, SWMASK ('E'))) {
                    // BUG: misses breakpoints on target of xed, rpt, and similar instructions
                    // because those instructions don't update the IC.  Some of those instructions
                    // do however provide their own breakpoint checks.
                    log_msg(WARN_MSG, "CU", "Execution Breakpoint\n");
                    reason = STOP_IBKPT;    /* stop simulation */
                    break;
                }
            }

            // We assume IC always points to the correct instr -- should be advanced after even instr
            uint IC_temp = PPR.IC;
            ic_history_add();
            execute_ir();

            // Check for fault from instr
            // todo: simplify --- cycle won't be EXEC anymore
            // Note: events.any zeroed by fault handler prior to xed even if other events are
            // pending, so if it's on now, we have a new fault

            flag_t is_fault = events.any && events.low_group && events.low_group < 7;   // Only fault groups 1-6 are recognized here, not interrupts or group 7 faults
            if (is_fault) {
                // faulted
                log_msg(WARN_MSG, "CU", "Probable fault detected after instruction execution\n");
                if (doing_xde || doing_xdo) {
                    char *which = doing_xde ? "even" : "odd";
                    log_msg(WARN_MSG, "CU", "XED %s instruction terminated by fault.\n", which);
                    // Note that we don't clear xde and xdo because they might be about to be stored by scu. Since we'll be in a FAULT cycle next, both flags will be set as part of the fault handler's xed
                }
                if (cu.rpt) {
                    log_msg(WARN_MSG, "CU", "Repeat instruction terminated by fault.\n");
                    cu.rpt = 0;
                }
            }
            if (! is_fault) {
                // no fault
                if (cu.rpt) {
                    if (cu.rpts) {
                        // Just executed the RPT instr
                        cu.rpts = 0;
                        ++ PPR.IC;
                        if (! cpu.ic_odd)
                            cpu.ic_odd = 1;
                        else
                            cpu.cycle = FETCH_cycle;
                    } else {
                        // Executed a repeated instruction
                        // log_msg(WARN_MSG, "CU", "Address handing for repeated instr was probably wrong.\n");
                        // Check for tally runout or termination conditions
                        uint t = reg_X[0] >> 10;    // bits 0..7 of 18bit register
                        if (cu.repeat_first && t == 0)
                            t = 256;
                        cu.repeat_first = 0;
                        --t;
                        reg_X[0] = ((t&0377) << 10) | (reg_X[0] & 01777);
                        // Note that we increment X[n] here, not in the APU. So, for
                        // instructions like cmpaq, the index register points to the
                        // entry after the one found.
                        int n = cu.tag & 07;
                        reg_X[n] += cu.delta;
                        if (opt_debug) log_msg(DEBUG_MSG, "CU", "Incrementing X[%d] by %#o to %#o.\n", n, cu.delta, reg_X[n]);
                        // Note that the code in bootload_tape.alm expects that the tally
                        // runout *not* be set when both the termination condition is met
                        // and bits 0..7 of reg X[0] hits zero.
                        if (t == 0) {
                            IR.tally_runout = 1;
                            cu.rpt = 0;
                            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Repeated instruction hits tally runout; halting rpt.\n");
                        }
                        // Check for termination conditions -- even if we hit tally runout
                        // Note that register X[0] is 18 bits
                        int terminate = 0;
                        if (getbit18(reg_X[0], 11))
                            terminate |= IR.zero;
                        if (getbit18(reg_X[0], 12))
                            terminate |= ! IR.zero;
                        if (getbit18(reg_X[0], 13))
                            terminate |= IR.neg;
                        if (getbit18(reg_X[0], 14))
                            terminate |= ! IR.neg;
                        if (getbit18(reg_X[0], 15))
                            terminate |= IR.carry;
                        if (getbit18(reg_X[0], 16))
                            terminate |= ! IR.carry;
                        if (getbit18(reg_X[0], 17)) {
                            log_msg(DEBUG_MSG, "CU", "Checking termination conditions for overflows.\n");
                            // Process overflows -- BUG: what are all the overflows?
                            if (IR.overflow || IR.exp_overflow) {
                                if (IR.overflow_mask)
                                    IR.overflow = 1;
                                else
                                    fault_gen(overflow_fault);
                                terminate = 1;
                            }
                        }
                        if (terminate) {
                            cu.rpt = 0;
                            log_msg(DEBUG_MSG, "CU", "Repeated instruction meets termination condition.\n");
                            IR.tally_runout = 0;
                            // BUG: need IC incr, etc
                        } else {
                            if (! IR.tally_runout)
                                if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Repeated instruction will continue.\n");
                        }
                    }
                    // TODO: if rpt double incr PPR.IC with wrap
                }
                // Retest cu.rpt -- we might have just finished repeating
                if (cu.rpt) {
                    // Don't do anything
                } else if (doing_xde) {
                    cu.xde = 0;
                    if (cpu.trgo) {
                        log_msg(NOTIFY_MSG, "CU", "XED even instruction was a transfer\n");
                        check_events();
                        cu.xdo = 0;
                        events.xed = 0;
                        cpu.cycle = FETCH_cycle;
                    } else {
                        log_msg(NOTIFY_MSG, "CU", "Resetting XED even flag\n");
                        // BUG? -- do we need to reset events.xed if cu.xdo isn't set?  -- but xdo must be set unless xed doesn't really mean double...
                    }
                } else if (doing_xdo) {
                    log_msg(NOTIFY_MSG, "CU", "Resetting XED odd flag\n");
                    cu.xdo = 0;
                    if (events.xed) {
                        events.xed = 0;
                        if (events.any)
                            log_msg(NOTIFY_MSG, "CU", "XED was from fault; other faults and/or interrupts occured during XED\n");
                        else {
                            log_msg(NOTIFY_MSG, "CU", "XED was from fault; checking if lower priority faults exist\n");
                            check_events();
                        }
                    }
                    cpu.cycle = FETCH_cycle;
                    if (!cpu.trgo) {
                        if (PPR.IC != IC_temp)
                            log_msg(NOTIFY_MSG, "CU", "No transfer instruction in XED, but IC changed from %#o to %#o\n", IC_temp, PPR.IC);
                        // ++ PPR.IC;       // this happens after executing the xde; we don't need to do it again
                    }
                } else if (! cpu.ic_odd) {
                    // Performed non-repeat instr at even loc (or finished last repetition)
                    if (cpu.cycle == EXEC_cycle) {
                            // After an xde, we'll increment PPR.IC.   Setting cpu.ic_odd will be ignored.
                            if (!cpu.trgo) {
                                if (PPR.IC == IC_temp) {
                                    cpu.ic_odd = 1; // execute odd instr of current pair
                                    ++ PPR.IC;
                                } else {
                                    if (! cpu.irodd_invalid)
                                        log_msg(NOTIFY_MSG, "CU", "No transfer instruction and IRODD not invalidated, but IC changed from %#o to %#o; changing to fetch cycle\n", IC_temp, PPR.IC);
                                    cpu.cycle = FETCH_cycle;
                                }
                            } else {
                                cpu.cycle = FETCH_cycle;    // IC changed; previously fetched instr for odd location isn't any good now
                            }
                    } else {
                        log_msg(WARN_MSG, "CU", "Changed from EXEC cycle to %d, not updating IC\n", cpu.cycle);
                    }
                } else {
                    // Performed non-repeat instr at odd loc (or finished last repetition)
                    // After an xde, we'll increment PPR.IC.   Setting cpu.ic_odd will be ignored.
                    if (cpu.cycle == EXEC_cycle) {
                        if (!cpu.trgo) {
                            if (PPR.IC == IC_temp) {
                                cpu.ic_odd = 0; // finished with odd half; BUG: restart issues?
                                ++ PPR.IC;
                            } else
                                if (!cpu.irodd_invalid)
                                    log_msg(NOTIFY_MSG, "CU", "No transfer instruction and IRODD not invalidated, but IC changed from %#o to %#o\n", IC_temp, PPR.IC);  // DEBUGGING
                        }
                    } else
                        log_msg(NOTIFY_MSG, "CU", "Cycle is %d after EXEC_cycle\n", cpu.cycle);
                    cpu.cycle = FETCH_cycle;
                }
            }
            break;
        default:
            log_msg(ERR_MSG, "CU", "Unknown cycle # %d\n", cpu.cycle);
            reason = STOP_BUG;
    }

    return reason;
}


//=============================================================================

void execute_ir(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)

    cpu.trgo = 0;
    execute_instr();    // located in opu.c
    ++ ninstr;  // BUG: exec instructions should increase this
}

//=============================================================================

static void check_events()
{
    // Called after executing an instruction pair for xed.   The instruction pair
    // may have including a rpt, rpd, transfer.   The instruction pair may even
    // have faulted, but if so, it was saved and restarted.

    events.any = events.int_pending || events.low_group || events.group7;
    if (events.any)
        log_msg(NOTIFY_MSG, "CU", "check_events: event(s) found (%d,%d,%d).\n", events.int_pending, events.low_group, events.group7);

    return;
}


//=============================================================================

void fault_gen(enum faults f)
{
    int group;

    if (f == oob_fault) {
        log_msg(ERR_MSG, "CU::fault", "Faulting for internal bug\n");
        f = trouble_fault;
        (void) cancel_run(STOP_BUG);
    }

    if (f < 1 || f > 32) {
        log_msg(ERR_MSG, "CU::fault", "Bad fault # %d\n", f);
        cancel_run(STOP_BUG);
        return;
    }
    group = fault2group[f];
    if (group < 1 || group > 7) {
        log_msg(ERR_MSG, "CU::fault", "Internal error.\n");
        cancel_run(STOP_BUG);
        return;
    }

    if (fault_gen_no_fault) {
        log_msg(DEBUG_MSG, "CU::fault", "Ignoring fault # %d in group %d\n", f, group);
        return;
    }

    events.any = 1;
    log_msg(DEBUG_MSG, "CU::fault", "Recording fault # %d in group %d\n", f, group);

    if (group == 7) {
        // Recognition of group 7 faults is delayed and we can have
        // multiple group 7 faults pending.
        events.group7 |= (1 << f);
    } else {
        // Groups 1-6 are handled more immediately and there can only be
        // one fault pending within each group
        if (cpu.cycle == FAULT_cycle) {
            f = trouble_fault;
            group = fault2group[f];
            log_msg(DEBUG_MSG, "CU::fault", "Double fault:  Recording current fault as a trouble fault (fault # %d in group %d).\n", f, group);
        } else {
            if (events.fault[group]) {
                // todo: error, unhandled fault
                log_msg(WARN_MSG, "CU::fault", "Found unhandled prior fault #%d in group %d.\n", events.fault[group], group);
            }
            if (cpu.cycle == EXEC_cycle) {
                // don't execute any pending odd half of an instruction pair
                cpu.cycle = FAULT_cycle;
            }
        }
        events.fault[group] = f;
    }
    if (events.low_group == 0 || group < events.low_group)
        events.low_group = group;   // new highest priority fault group
}

//=============================================================================

int fault_check(enum faults f)
{
    // Find fault

    if (f < 1 || f > 32) {
        log_msg(ERR_MSG, "CU::fault-check", "Bad fault # %d\n", f);
        cancel_run(STOP_BUG);
        return 1;
    }

    if (! events.any)
        return 0;
    int group = fault2group[f];
    if (group == 7)
        return events.group7 & (1 << f);
    else
        return events.fault[group] == f;
}

int fault_check_group(int group)
{
    // Returns whether or not any faults exist for the specifed group or a higher priority group

    if (group < 1 || group > 7) {
        log_msg(ERR_MSG, "CU::fault-check-group", "Bad group # %d\n", group);
        cancel_run(STOP_BUG);
        return 1;
    }

    if (! events.any)
        return 0;
    return events.low_group <= group;
}

//=============================================================================

int fetch_instr(uint IC, instr_t *ip)
{
    // Returns non-zero if a fault in groups 1-6 is detected

    t_uint64 word;
    int ret = fetch_word(IC, &word);
    if (ip)
        decode_instr(ip, word);
#if 0
    if (opt_debug) {
        instr_t i;
        if (ip == NULL) {
            ip = &i;
            decode_instr(ip, word);
        }
        if (opt_debug>0) log_msg(DEBUG_MSG, "CU::fetch-instr", "Fetched word %012llo => %s\n", word, instr2text(ip));
    }
#endif
    return ret;
}

//=============================================================================

int fetch_word(uint addr, t_uint64 *wordp)
{
    // todo: Allow SIMH to use segmented addressing to specify breakpoints. Next, check for such.
    // Returns non-zero if a fault in groups 1-6 is detected

    addr_modes_t mode = get_addr_mode();
#if 0
if (mode != cpu.orig_mode_BUG) {
log_msg(WARN_MSG, "CU::fetch", "Using mode %d not %d\n", cpu.orig_mode_BUG, mode);
mode = cpu.orig_mode_BUG;
}
#endif

    if (mode == APPEND_mode) {
        return fetch_appended(addr, wordp);
    } else if (mode == ABSOLUTE_mode) {
        return fetch_abs_word(addr, wordp);
    } else if (mode == BAR_mode) {
        if (addr >= (BAR.bound << 9)) {
            log_msg(NOTIFY_MSG, "CU::fetch", "Address %#o is out of BAR bounds of %#o.\n", addr, BAR.bound << 9);
            fault_gen(store_fault);
            // fault_reg.oob = 1;           // ERROR: fault_reg does not exist
            // cancel_run(STOP_WARN);
            return 1;
        }
        log_msg(DEBUG_MSG, "CU::fetch", "Translating offset %#o to %#o.\n", addr, addr + (BAR.base << 9));
        addr += BAR.base << 9;
        return fetch_abs_word(addr, wordp);
        // return fetch_appended(addr, wordp);
#if 0
        log_msg(ERR_MSG, "CU::fetch", "Addr=%#o:  BAR mode unimplemented.\n", addr);
        cancel_run(STOP_BUG);
        return fetch_abs_word(addr, wordp);
#endif
    } else {
        log_msg(ERR_MSG, "CU::fetch", "Addr=%#o:  Unknown addr mode %d.\n", addr, mode);
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
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from IOM mailbox area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from DN355 mailbox area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
#define CONFIG_DECK_LOW 012000
#define CONFIG_DECK_LEN 010000
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from CONFIG DECK area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr <= 030) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from 0..030 for addr %#o\n", addr);
    }

    if (addr >= ARRAY_SIZE(M)) {
            log_msg(ERR_MSG, "CU::fetch", "Addr %#o (%d decimal) is too large\n", addr, addr);
            (void) cancel_run(STOP_BUG);
            return 0;
    }

    if (sim_brk_summ) {
        if (sim_brk_test (addr, SWMASK ('M'))) {
            log_msg(WARN_MSG, "CU::fetch", "Memory Breakpoint, address %#o.  Fetched value %012llo\n", addr, M[addr]);
            (void) cancel_run(STOP_IBKPT);
        }
    }

    cpu.read_addr = addr;
    *wordp = M[addr];   // absolute memory reference
    if (get_addr_mode() == BAR_mode)
        log_msg(DEBUG_MSG, "CU::fetch-abs", "fetched word at %#o\n", addr);
    return 0;
}


int store_word(uint addr, t_uint64 word)
{
    // todo: Allow SIMH to use segmented addressing to specify breakpoints. Next, check for such.
    // Returns non-zero if a fault in groups 1-6 is detected

    addr_modes_t mode = get_addr_mode();
#if 0
if (mode != cpu.orig_mode_BUG){
log_msg(WARN_MSG, "CU::store", "Using mode %d not %d\n", cpu.orig_mode_BUG, mode);
mode = cpu.orig_mode_BUG;
}
#endif

    if (mode == APPEND_mode) {
        return store_appended(addr, word);
    } else if (mode == ABSOLUTE_mode) {
        return store_abs_word(addr, word);
    } else if (mode == BAR_mode) {
#if 0
        log_msg(ERR_MSG, "CU::store", "Addr=%#o:  BAR mode unimplemented.\n", addr);
        cancel_run(STOP_BUG);
        return store_abs_word(addr, word);
#else
        if (addr >= (BAR.bound << 9)) {
            log_msg(NOTIFY_MSG, "CU::store", "Address %#o is out of BAR bounds of %#o.\n", addr, BAR.bound << 9);
            fault_gen(store_fault);
            // fault_reg.oob = 1;           // ERROR: fault_reg does not exist
            // cancel_run(STOP_WARN);
            return 1;
        }
        log_msg(DEBUG_MSG, "CU::store", "Translating offset %#o to %#o.\n", addr, addr + (BAR.base << 9));
        addr += BAR.base << 9;
        return store_abs_word(addr, word);
        //return store_appended(addr, word);
#endif
    } else {
        log_msg(ERR_MSG, "CU::store", "Addr=%#o:  Unknown addr mode %d.\n", addr, mode);
        cancel_run(STOP_BUG);
        return 1;   // BUG: gen fault
    }
}


int store_abs_word(uint addr, t_uint64 word)
{
    // Store word to location given by 24bit abs memory addr

    // todo: efficiency: combine into a single min/max with sub-tests
    if (addr >= IOM_MBX_LOW && addr < IOM_MBX_LOW + IOM_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to IOM mailbox area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to DN355 mailbox area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to CONFIG DECK area for addr %#o\n", addr);
        //cancel_run(STOP_WARN);
    }
    if (addr <= 030) {
        //log_msg(DEBUG_MSG, "CU::store", "Fetch from 0..030 for addr %#o\n", addr);
    }
    if (addr >= ARRAY_SIZE(M)) {
            log_msg(ERR_MSG, "CU::store", "Addr %#o (%d decimal) is too large\n");
            (void) cancel_run(STOP_BUG);
            return 0;
    }
    if (sim_brk_summ)
        if (sim_brk_test(addr, SWMASK('W') | SWMASK('M') | SWMASK('E'))) {
            if (sim_brk_test(addr, SWMASK ('W'))) {
                log_msg(WARN_MSG, "CU::store", "Memory Write Breakpoint, address %#o\n", addr);
                (void) cancel_run(STOP_IBKPT);
            } else if (sim_brk_test(addr, SWMASK ('M'))) {
                log_msg(WARN_MSG, "CU::store", "Memory Breakpoint, address %#o\n", addr);
                (void) cancel_run(STOP_IBKPT);
            } else
                log_msg(NOTIFY_MSG, "CU::store", "Write to a location that has an execution breakpoint, address %#o\n", addr);
            log_msg(NOTIFY_MSG, "CU::store", "Address %08o: value was %012llo, storing %012llo\n", addr, M[addr], word);
        }

    M[addr] = word; // absolute memory reference
    if (addr == cpu.IC_abs) {
        log_msg(NOTIFY_MSG, "CU::store", "Flagging cached odd instruction from %o as invalidated.\n", addr);
        cpu.irodd_invalid = 1;
    }
    if (get_addr_mode() == BAR_mode)
        log_msg(DEBUG_MSG, "CU::store-abs", "stored word to %#o\n", addr);
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
    uint Y = (aligned) ? (addr / n) * n : addr;

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
    uint Y = (aligned) ? (addr / n) * n : addr;

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


void encode_instr(const instr_t *ip, t_uint64 *wordp)
{
        *wordp = setbits36(0, 0, 18, ip->addr);
        *wordp = setbits36(*wordp, 18, 10, ip->opcode);
        *wordp = setbits36(*wordp, 28, 1, ip->inhibit);
        if (! is_eis[ip->opcode&MASKBITS(10)]) {
            *wordp = setbits36(*wordp, 29, 1, ip->mods.single.pr_bit);
            *wordp = setbits36(*wordp, 30, 6, ip->mods.single.tag);
        } else {
            *wordp = setbits36(*wordp, 29, 1, ip->mods.mf1.ar);
            *wordp = setbits36(*wordp, 30, 1, ip->mods.mf1.rl);
            *wordp = setbits36(*wordp, 31, 1, ip->mods.mf1.id);
            *wordp = setbits36(*wordp, 32, 4, ip->mods.mf1.reg);
        }
}

//=============================================================================

void anal36 (const char* tag, t_uint64 word);
#include <ctype.h>

void tape_block(unsigned char *p, uint32 len, uint32 addr)
{
    log_msg(DEBUG_MSG, "CPU::boot", "Tape block: %u bytes, %u 36-bit words\n", len, len*8/36);
    if ((len * 8) % 36 != 0) {
        log_msg(ERR_MSG, "CPU::boot", "Length %u bytes is not a multiple of 36 bits.\n");
    }
    bitstream_t *bp = bitstm_new(p, len);
    uint32 nbits = len * 8;
    while (nbits >= 36) {
        bitstm_get(bp, 36, &M[addr++]);     // absolute addresses
        nbits -= 36;
    }
    if (nbits != 0) {
        log_msg(ERR_MSG, "CPU::boot", "Internal error.   Some bits left over while reading tape\n");
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
    printf("%s: %012llo octal, %llu decimal\n", tag, word, word);
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
    M[addr] = v;        // SIMH absolute reference; BUG: use store_abs_word in case we need cache invalidation
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
    is_eis[(opcode1_btd<<1)|1] = 1;
    is_eis[(opcode1_mvne<<1)|1] = 1;
}
