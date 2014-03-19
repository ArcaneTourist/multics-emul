/*
    hw6180_cpu.c

    Provides the topmost level of instruction processing, the sim_instr()
    function.  The SIMH package calls our sim_instr() function and expects
    it to fetch instructions, excecute them, and update the program counter.

    Also provides fault and interrupt handling.

    Also provides routines for fetching from memory and storing to memory.
    (Note that the physical CPU had no direct access to memory; memory 
    belonged to the system controllers.)

    Also provides the cpu_boot() routine which simulates the way an IOM or
    other hardware would load the first record from a boot tape into memory.

    Defines most of the global data structures related to a CPU.

    See opu.c for implemention of individual instructions.
    See apu.c for implemention of addressing.
    See README.source for more information about which files hold which
    parts of the emulator.

*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/


#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "hw6180.h"
#include "sim_tape.h"
// #include "bits.h"

//-----------------------------------------------------------------------------
// *** SIMH specific externs

// TODO: Remove these if they become available via the SIMH sim_defs.h header
extern int32 sim_interval;
extern uint32 sim_brk_summ;
extern int32 sim_switches;

//-----------------------------------------------------------------------------
// *** Main memory

t_uint64 *Mem;

//-----------------------------------------------------------------------------
// *** CPU Registers

// We give SIMH a description of our CPU "registers" and pointers to the
// variables that hold "register" contents.  This allows SIMH to provide the
// user with commands to display and change the registers.  By SIMH convention,
// the globals for the registers are defined here and are described in
// the cpu_reg[] array.  The cpu_reg[] array is itself referenced indirectly
// by the sim_devices[] array which is known to SIMH.   The sim_devices[]
// array is defined in h6180_sys.c

// SIMH expects the registers to be sized just large enough to hold the
// number of bits we claim when we describe the register to SIMH.
// Thus, the "18-bit" index registers use a 32-bit data type and the
// 36-bit registers use a 64-bit data type.

// TODO -- Re-review to make sure all data types referenced in sim_devices[]
// are sized as the min size for number of bits

// The first register that SIMH expects is a program counter.   However,
// we don't have a simple program counter.  We have hardware virtual memory
// and three addressing modes -- BAR, absolute, and appending (segmented mode).
// The PPR pseudo-register is really the program counter.   The PPR is
// composed of registers from the appending unit and the control unit:
//      AU:PSR -- segment number (15 bits); not used in absolute mode
//      CU:IC -- instruction counter (18 bits)
//      AU:P -- privileged mode flag
//      AU:PRR: 3 bits -- ring number
//      See also
//          BAR register -- only for BAR mode which Multics doesn't use.
// We tell SIMH that our "PC" is the saved_PPR_addr global which contains
// only the portions of the PPR that are needed to form addresses.

// TODO: If we wanted to support multiple CPUs, all of the following
// globals would need to be moved into a per-CPU data structure.

// The registers below are listed in the same order as the first page of
// section 3 of AL-39.

// 'E', 'A', and 'Q' Registers
// Note that the "AQ" register is just a combination of the A and Q registers
// Note that the "EAQ" register is a combination of the E, A, and Q registers
t_uint64 reg_A;         // Accumulator, 36 bits
t_uint64 reg_Q;         // Quotient, 36 bits
int8 reg_E;             // Exponent for floating point data

uint32 reg_X[8];        // Index Registers, 18 bits

IR_t IR;                // Indicator register
static uint32 saved_IR; // Only for sending to/from SIMH

BAR_reg_t BAR;          // Base Addr Register; 18 bits
static uint16 saved_BAR[2]; // Only for sending to/from SIMH

uint32 reg_TR;          // Timer Reg, 27 bits -- only valid after calls to SIMH clock routines
uint8 reg_RALR;         // Ring Alarm Reg, 3 bits

// PR and AR registers (42 bits and 24 bits respectively)
// We combine the "PR" Pointer Registers and the "AR" Address Registers into
// a single data structure.
// Note that the eight PR registers are also known by the names: ap,
// ab, bp, bb, lp, lb, sp, sb
AR_PR_t AR_PR[8];
// Packed versions of the AR/PR registers; format specific to this emulator
t_uint64 saved_ar_pr[8];    // Only for sending to/from SIMH

// See earlier comments re the PPR and IC
PPR_t PPR;              // Procedure Pointer Reg, 37 bits, internal only
static t_uint64 saved_PPR;  // Only for sending to/from SIMH; see also sim_PC and saved_IC
static t_uint64 saved_TPR;  // Only for sending to/from SIMH; see also sim_PC and saved_IC
static t_uint64 saved_PPR_addr; // Only for sending to/from SIMH; see also sim_PC and saved_IC
static uint32 saved_IC; // Only for sending to/from SIMH; duplicates portions of saved_PPR

TPR_t TPR;      // Temporary Pointer Reg, 42 bits, internal only
static t_uint64 saved_DSBR;     // Descriptor Segment Base Register, 51 bits
mode_reg_t MR;
t_uint64 CMR;   // Cache Mode Register (ignored); 28 bits scattered within a 36bit word
t_uint64 FR;        // "FR" Fault Register

// Other registers listed in the processor manual but not implemented
// as globals.
// SDWAM_t SDWAM[16];   // Segment Descriptor Word Associative Memory, 88 bits
// PTWAM_t PTWAM[16];   // Page Table Word Associative Memory, 51 bits
// static fault_reg_t FR;   // Fault Register, 35 bits
// mode register
// CU_hist[16];     // 72 bits
// OU_hist[16];     // 72 bits
// DU_hist[16];     // 72 bits
// APU_hist[16];    // 72 bits
// ConfigSwitch[5];
// CU data  // Control Unit data, 288 bits, internal only
// DU data  // Decimal Unit data, 288 bits, internal only

// A few registers are in a per-cpu data structure.  See also the hack
// just below where we give SIMH this struct as raw data without a
// description in order to support a limited save/restore.
static cpu_t cpu_info;
cpu_t *cpup = &cpu_info;    // but we still only have one CPU

//-----------------------------------------------------------------------------
// IOM
iom_t iom;                      // only one for now

//-----------------------------------------------------------------------------
// Descriptions of all of the registers for SIMH.
//
// TODO: modify simh to take a PV_ZLEFT flag (leftmost bit is bit 0) or
// perhaps PV_ZMSB (most significant bit is numbered zero).  Why did
// I say this... Perhaps that comment was before fprint_sym/fprint_addr?
// 
// NOTE: Instead of using REG_USER# flags, we could also check regp->name to
// detect which registers should have special formatting in fprint_sym.  Even
// better, we'll switch to description fields with GRDATADF.

// We populate this in save_to_simh() so that only the "on" fields are displayed
static BITFIELD IR_bits[19] = {
    { 0 },
};

REG cpu_reg[] = {
    // structure members: name="PC", loc=PC, radix=<8>, width=36, offset=<0>, depth=<1>, flags=, qptr=
    { ORDATA (PPR_addr, saved_PPR_addr, 64), REG_RO | REG_VMIO | REG_HIDDEN },
    { ORDATA (PPR, saved_PPR, 64), REG_VMIO },
    { ORDATA (IC, saved_IC, 18) },      // saved_IC address also stored in sim_PC external
    { GRDATADF (IR, saved_IR, 2, 18, 0, "Indicator Register", IR_bits), REG_RO},
    { ORDATA (A, reg_A, 36) },
    { ORDATA (Q, reg_Q, 36) },
    { ORDATA (E, reg_E, 8) },
    { BRDATA (X, reg_X, 8, 18, 8) },
    { ORDATA (FR, FR, 36) },
    { ORDATA (MR, MR.word, 36), REG_RO },
    { ORDATA (TR, reg_TR, 27), REG_RO },
    // TODO: Let SIMH modify the AR/PR registers -- use VM_AD flags, etc
    { BRDATA (PR, saved_ar_pr, 8, 42, 8), REG_VMIO | REG_USER2 },
    // stuff needed to yield a save/restore sufficent for examining memory dumps
    { BRDATA (BAR, saved_BAR, 8, 9, 2) },
    { ORDATA (DSBR, saved_DSBR, 51) },
    { ORDATA (TPR, saved_TPR, 46), REG_VMIO },
    { ORDATA (CMR, CMR, 36), REG_RO },
    // The following is a hack, but works as long as you don't save/restore
    // across different architectures, compiler implementations, or phases
    // of the moon.
        { BRDATA (CPUINFO, (&cpu_info), 8, 8, sizeof(cpu_info)), REG_RO },
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

extern int iom_show_mbx(FILE *st, UNIT *uptr, int val, void *desc); // FIXME
static int cpu_set_fault_base(UNIT *uptr, int32 val, char *cptr, void *desc);
static int cpu_show_fault_base(FILE *st, UNIT *uptr, int32 val, void *desc);
static MTAB cpu_mod[] = {
    // for SIMH "show" and "set" commands
    { MTAB_XTD | MTAB_VDV | MTAB_NMO | MTAB_SHP | MTAB_NC,
      0, "HISTORY", "HISTORY",
      cpu_set_history, cpu_show_history, NULL },
    { MTAB_XTD | MTAB_VDV | MTAB_NMO | MTAB_NC,
      0, "STACK", NULL,
      NULL, cpu_show_stack, NULL },
    { MTAB_XTD | MTAB_VDV | MTAB_NMO | MTAB_NC,
      0, "VM", NULL,
      NULL, apu_show_vm, NULL },
    { MTAB_XTD | MTAB_VDV | MTAB_NMO | MTAB_SHP | MTAB_NC,
      0, "SEG", NULL,
      NULL, apu_show_seg, NULL },
    { MTAB_XTD | MTAB_VDV | MTAB_NC,
      0, "FAULT_BASE", "FAULT_BASE",
      cpu_set_fault_base, cpu_show_fault_base, NULL },
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
    1, 8, 43, 1, 8, 36,                 // 43 because we give SIMH "packed" addresses; see addr_emul_to_simh()
    &cpu_ex, &cpu_dep, &cpu_reset,
    &cpu_boot, NULL, NULL,
    NULL, DEV_DEBUG
};

//-----------------------------------------------------------------------------
// *** Other devices

extern t_stat clk_svc(UNIT *up);
UNIT TR_clk_unit = { UDATA(&clk_svc, UNIT_IDLE, 0) };

extern t_stat mt_svc(UNIT *up);
UNIT mt_unit = {
    // NOTE: other SIMH tape sims don't set UNIT_SEQ
    UDATA (&mt_svc, UNIT_ATTABLE | UNIT_SEQ | UNIT_ROABLE | UNIT_DISABLE | UNIT_IDLE, 0)
};

DEVICE tape_dev = {
    "TAPE", &mt_unit, NULL, NULL, 1,
    10, 31, 1, 8, 9,
    NULL, NULL, NULL,
    NULL, &sim_tape_attach, &sim_tape_detach,
    NULL, DEV_DEBUG
};

/* unfinished; copied from tape_dev */
#define M3381_SECTORS 6895616 
// records per subdev: 74930 (127 * 590)
// number of sub-volumes: 3
// records per dev: 3 * 74930 = 224790
// cyl/sv: 590
// cyl: 1770 (3*590)
// rec/cyl 127
// tracks/cyl 15
// sector size: 512
// sectors: 451858
// data: 3367 MB, 3447808 KB, 6895616 sectors, 
//  3530555392 bytes, 98070983 records?

// extern t_stat disk_svc(UNIT *up);
UNIT disk_unit = {
    UDATA (&channel_svc, UNIT_FIX | UNIT_ATTABLE | UNIT_ROABLE | UNIT_DISABLE | UNIT_IDLE, M3381_SECTORS)
};

// No disks known to multics had more than 2^24 sectors...
DEVICE disk_dev = {
    "DISK", &disk_unit, NULL, NULL, 1,
    10, 24, 1, 8, 36,
    /* examine */ NULL, /* deposit */ NULL,
    /* reset */ NULL, /* boot */ NULL,
    /* attach */ NULL, /* detach */ NULL,
    /* context */ NULL, DEV_DEBUG
};


MTAB opcon_mod[] = {
    { MTAB_XTD | MTAB_VDV | MTAB_VALO | MTAB_NC | MTAB_QUOTE,
        0, NULL, "AUTOINPUT",
        opcon_autoinput_set, opcon_autoinput_show, NULL },
    { 0 }
};


// Need to provide at least one unit or simh display won't emit a newline
static t_stat con_svc(UNIT*up) { return 0; }
UNIT opcon_unit = { UDATA(&con_svc, 0, 0) };
DEVICE opcon_dev = {
    "OPCON", &opcon_unit, NULL, opcon_mod,
    1, 10, 8, 1, 8, 8,
    NULL, NULL, NULL,
    NULL, NULL, NULL,
    NULL, DEV_DEBUG
};

extern t_stat iom_svc(UNIT* up);
extern t_stat iom_reset(DEVICE *dptr);
UNIT iom_unit = { UDATA(&iom_svc, 0, 0) };
MTAB iom_mod[] = {
    { MTAB_XTD | MTAB_VDV | MTAB_NMO | MTAB_NC,
      0, "MBX", NULL,
      NULL, iom_show_mbx, NULL },
    { 0 }
};
REG iom_reg[] = {
    { ORDATA(IOM_NUM, iom.iom_num, 2) },
    { ORDATA(BASE_ADDR, iom.base, 12) },    // switches, not value
    { NULL }
};
DEVICE iom_dev = {
    "IOM", &iom_unit, iom_reg, iom_mod,
    1, 10, 8, 1, 8, 8,
    NULL, NULL, &iom_reset,
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
cpu_ports_t cpu_ports;          // Describes connections to SCUs
// the following two should probably be combined
cpu_state_t cpu;
ctl_unit_data_t cu; 

scu_t scu;                      // only one for now

// This is an out-of-band flag for the APU.  User commands to
// display or modify memory can invoke much the APU.  Howeveer, we don't
// want interactive attempts to access non-existant memory locations
// to register a fault.
flag_t fault_gen_no_fault;

// *** Other variables -- These do not need to be part of save/restore
// static int seg_debug[n_segments];

//-----------------------------------------------------------------------------
// ***  Other Externs

// This only controls warning messages
extern int bootimage_loaded;    // only relevent for the boot CPU ?

// Debugging and statistics
int opt_debug;
t_uint64 calendar_a; // Used to "normalize" A/Q dumps of the calendar as deltas
t_uint64 calendar_q;
stats_t sys_stats;

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

#define getbit18(x,n)  ((((x) >> (17-n)) & 1) != 0) // return nth bit of an 18bit half word

static t_stat control_unit(void);
static void execute_ir(void);
static void init_opcodes(void);
static void check_events(void);
static void save_to_simh(void);
static void save_PR_registers(void);
static void restore_PR_registers(void);
static int write72(FILE* fp, t_uint64 word0, t_uint64 word1);
static int read72(FILE* fp, t_uint64* word0p, t_uint64* word1p);
static void set_IR_bitnames(uint32 irval);
void init_memory_iom();
static t_uint64 save_TPR(const TPR_t *tprp);

//=============================================================================

static inline uint max3(uint a, uint b, uint c)
{
    return (a > b) ?
        ((a > c) ? a : c) :
        ((b > c) ? b : c);
}

//=============================================================================

/*
 *  cpu_boot()
 *
 *  Called by SIMH's BOOT command.
 *
 *  SIMH usually expects the boot routine to load a bootstrap program into
 *  memory and set the program counter.   Instead, we expect that control words
 *  for the IOM have already been loaded in memory.  This is a BUG; we should
 *  do that memory initializaion here.   With the IOM control words in memory,
 *  we simply send an interrupt signal to the IOM.
 *
 *  The above process results in the IOM loading a bootstrap program from
 *  tape into memory.   See cpu_reset() for a description of starting the
 *  CPU.
 */

t_stat cpu_boot (int32 unit_num, DEVICE *dptr)
{

    /*
     *  An earlier version of this code did not make use of an emulated IOM
     *  or IOX.
     *  It would be sufficient to read the first block, load it into memory,
     *  and leave the virtual tape positioned past the first block.   However,
     *  during attempts to emulate an IOX it was discovered that the code in
     *  bootload_tape_label.alm will check the result in the status mailbox
     *  area of main memory if an IOX is detected.
     *  Emulating an IOX was later abandoned because when using an IOX,
     *  bootload_tape_label.alm will try to execute the "ldo" instruction.
     *  The "ldo" instruction was never implemented on the L68 series. I
     *  originaly though "ldo" instruction was only implmented on on the
     *  ADP aka ORION aka DPS88.   However, years later I find that it also
     *  existed on the DPS8.
     *  This version does run the IOM or IOX to do the I/O.  Note that
     *  prior to booting, the emulated IOM or IOX must load IOM control words
     *  into memory.  This is done in init_memory_iom() or in init_memory_iox().
     *  This approach replaces the dozen or so lines of code that used to live
     *  here with a single call to iom_interrupt().
     */

    // ++ opt_debug; ++ cpu_dev.dctrl;
    if (cpu_dev.dctrl != 0) opt_debug = 1;  // todo: should CPU control all debug settings?


    // Initializing memory to reflect the existance of an IOM, not an
    // IOX.  Using an IOX causes use of the non L68 "ldo" instruction.
    // Also, the IOX has an undocumented mailbox architecture.
    // init_memory_iox();
    init_memory_iom();

    // Send an interrupt to the IOM -- not to the CPU
    int ret = 0;
    if (sys_opts.iom_times.connect < 0) {
        log_msg(INFO_MSG, "CPU::boot", "Issuing IOM interrupt.\n");
        iom_interrupt();
    } else {
        ret = sim_activate(&iom_dev.units[0], sys_opts.iom_times.connect);
        log_msg(INFO_MSG, "CPU::boot", "Queuing an IOM interrupt to occur in %d cycles\n", sys_opts.iom_times.connect);
        if (ret != 0)
            log_msg(ERR_MSG, "CPU::boot", "Cannot activate IOM.\n");
    }

    // -- opt_debug; -- cpu_dev.dctrl;

    bootimage_loaded = 1;
    log_msg(INFO_MSG, "CPU::boot", "Returning to SIMH.\n");

    return ret;
}

//=============================================================================

/*
 *  cpu_reset()
 *
 *  Reset -- Reset to initial state -- clear all device flags and cancel any
 *  any outstanding timing operations. Used by SIMH's RESET, RUN, and BOOT
 *  commands
 *
 *  BUG: Should reset *all* structures and registers to zero.
 *  
 *  Note that SIMH doesn't have much difference between reset and power-on
 *
 *  Real hardware had to wait for a connect signal from the SCU before
 *  doing anything -- multicians.org glossary.   So, presumably the CPU
 *  could not even fetch an instruction until the system controller had
 *  setup memory and given a go-ahead.   So, it's tempting simulate
 *  waiting for access to memory or perhaps to load the "idle until interrupt"
 *  instruction into the IR (instruction register).  However, the emulation
 *  always initializes memory and/or calls cpu_boot(), so it is sufficient to
 *  simply record a startup fault or power-on interrupt.
 *
 *  BUG: We should probably let the IOM send a "terminate" interrupt rather
 *  than generating one here.
 */

t_stat cpu_reset (DEVICE *dptr)
{

    log_msg(INFO_MSG, "CPU::reset", "Running\n");

    init_opcodes();
    ic_history_init();

    bootimage_loaded = 0;
    memset(&events, 0, sizeof(events));
    memset(&cpu, 0, sizeof(cpu));
    memset(&cu, 0, sizeof(cu));
    memset(&PPR, 0, sizeof(PPR));
    cu.SD_ON = 1;
    cu.PT_ON = 1;
    cpu.ic_odd = 0;

    // TODO: reset *all* other structures to zero

    set_addr_mode(ABSOLUTE_mode);

    // We startup with either a fault or an interrupt.  So, a trap pair from the
    // appropriate location will end up being the first instructions executed.
    if (sys_opts.startup_interrupt) {
        // We'll first generate interrupt #4.  The IOM will have initialized
        // memory to have a DIS (delay until interrupt set) instruction at the
        // memory location used to hold the trap words for this interrupt.
        cpu.cycle = INTERRUPT_cycle;
        events.int_pending = 1;
        events.interrupts[4] = 1;   // system fault, IOM zero, channels 0-31 -- MDD-005
    } else {
        // Generate a startup fault.
        // We'll end up in a loop of trouble faults for opcode zero until the IOM
        // finally overwites the trouble fault vector with a DIS from the tape
        // label.
        cpu.cycle = FETCH_cycle;
        fault_gen(startup_fault);   // pressing POWER ON button causes this fault
    }

    calendar_a = 0xdeadbeef;
    calendar_q = 0xdeadbeef;

#if FEAT_INSTR_STATS
    memset(&sys_stats, 0, sizeof(sys_stats));
#endif

    save_to_simh();     // pack private variables into SIMH's world
    return 0;
}

//=============================================================================

/*
 * sim_load()
 *
 * SIMH binary loader.
 *
 * The load normally starts at the current value of the PC.
 * Args
 *     fileref -- file opened by SIMH
 *     cptr -- VM specific args (from cmd line?)
 *     fnam -- filename
 *     write_flag -- indicates whether to load or write
 *
 * This is just a quick stub to save/load all of memory and may not have
 * any use beyond debugging.  Most files that we'd want to load are
 * bootable tapes...
 */

t_stat sim_load (FILE *fileref, char *cptr, char *fnam, int32 write_flag)
{
    // Assumes that MAXMEMSIZE is an even number...
    if (write_flag) {
        out_msg("Dumping memory to %s.\n", fnam);
        for (int i = 0; i < MAXMEMSIZE - 1; i += 2) {
            if (write72(fileref, Mem[i], Mem[i+1]) != 0) {
                log_msg(ERR_MSG, "DUMP", "Error writing %s: %s\n", fnam, strerror(errno));
                return SCPE_IOERR;
            }
        }
    } else {
        out_msg("Loading memory from %s.\n", fnam);
        for (int i = 0; i < MAXMEMSIZE - 1; i += 2) {
            if (feof(fileref)) {
                out_msg("EOF on %s after %d words\n", fnam, i);
                bootimage_loaded = 1;
                return SCPE_OK;
            }
            if (read72(fileref, &Mem[i], &Mem[i+1]) != 0) {
                log_msg(ERR_MSG, "DUMP", "Error reading %s: %s\n", fnam, strerror(errno));
                return SCPE_IOERR;
            }
        }
        bootimage_loaded = 1;
    }
    out_msg("Done.\n");
    return SCPE_OK;
}


//=============================================================================

/*
 * init_memory_iom()
 *
 * Load a few words into memory.   Simulates pressing the BOOTLOAD button
 * on an IOM or equivalent.
 *
 * All values are from bootload_tape_label.alm.  See the comments at the
 * top of that file.  See also doc #43A239854.
 *
 * NOTE: The values used here are for an IOM, not an IOX.
 * See init_memory_iox() below.
 *
 */

void init_memory_iom()
{
    // On the physical hardware, settings of various switchs are reflected
    // into memory.  We provide no support for simulation of of the physical
    // switches because there is only one useful value for almost all of the
    // switches.  So, we hard code the memory values that represent usable
    // switch settings.

    // The presence of a 0 in the top six bits of word 0 denote an IOM boot
    // from an IOX boot

    // " The channel number ("Chan#") is set by the switches on the IOM to be
    // " the channel for the tape subsystem holding the bootload tape. The
    // " drive number for the bootload tape is set by switches on the tape
    // " MPC itself.

    log_msg(INFO_MSG, "CPU::IOM", "Performing load of eleven words from IOM bootchanel to memory.\n");

    const int base = iom.base;      // 12 bits; IOM base; switch values
    // bootload_io.alm insists that pi_base match
    // template_slt_$iom_mailbox_absloc
    // const int multiplex_base_switches = 0120;    // only valid value
    const int pi_base = 01200; // 15 bits; interrupt cells

    const int iom_num = iom.iom_num;  // 3 bits; only IOM 0 would use vector 030

    log_msg(NOTIFY_MSG, "IOM::boot", "bootload starting\n");
    if (iom.iom_num == iom_num)
        log_msg(NOTIFY_MSG, "IOM::boot", "IOM # is %d.\n", iom_num);
    else
        log_msg(NOTIFY_MSG, "IOM::boot", "IOM # is %d, but acting as %d!\n", iom.iom_num, iom_num);
    //log_msg(NOTIFY_MSG, "IOM::boot", "Multiplex switches are %#o\n", multiplex_base_switches);
    log_msg(NOTIFY_MSG, "IOM::boot", "Mbx base is %#o\n", base);
    log_msg(NOTIFY_MSG, "IOM::boot", "Interrupt base is %#o\n", pi_base);
    log_msg(NOTIFY_MSG, "IOM::boot", "Tape channel is %#o (%d)\n", sys_opts.tape_chan, sys_opts.tape_chan);

    t_uint64 cmd = 5;       // 6 bits; 05 for tape, 01 for cards
    int dev = 0;            // 6 bits: drive number

    // Maybe an is-IMU flag; IMU is later version of IOM
    t_uint64 imu = 0;       // 1 bit

    /* Description of the bootload channel from 43A239854
        Legend
            BB - Bootload channel #
            C - Cmd (1 or 5)
            N - IOM #
            P - Port #
            XXXX00 - Base Addr -- 01400
            XXYYYY0 Program Interrupt Base
    */

    const t_uint64 dis0 = 0616200;
    /* 1*/ Mem[010 + 2 * iom_num] = (imu << 34) | dis0;         // system fault vector; DIS 0 instruction (imu bit not mentioned by 43A239854)
    // Zero other 1/2 of y-pair to avoid msgs re reading uninitialized
    // memory (if we have that turned on)
    Mem[010 + 2 * iom_num + 1] = 0;

    /* 2*/ Mem[030 + 2 * iom_num] = dis0;                       // terminate interrupt vector (overwritten by bootload)
    const int base_addr = base << 6; // 01400

    /* 3*/ Mem[base_addr + 7] = ((t_uint64) base_addr << 18) | 02000002;    // tally word for sys fault status
                // ??? Fault channel DCW

    // bootload_tape_label.alm says 04000, 43A239854 says 040000.  Since 43A239854 says
    // "no change", 40000 is correct; 4000 would be a large tally
    /* 4*/ Mem[base_addr + 010] = 040000;       // Connect channel LPW; points to PCW at 000000
    const int mbx = base_addr + 4 * sys_opts.tape_chan;
    /* 5*/ Mem[mbx] = 03020003;             // Boot device LPW; points to IDCW at 000003
    /* 6*/ Mem[4] = 030 << 18;              // Second IDCW: IOTD to loc 30 (startup fault vector)

    // Default SCW points at unused first mailbox.
    // T&D tape overwrites this before the first status is savec, though.
    /* 7*/ Mem[mbx + 2] = ((t_uint64)base_addr << 18);      // SCW

    /* 8*/ Mem[0] = 0720201;                    // 1st word of bootload channel PCW

    // "SCU port" # (deduced as meaning "to which bootload IOM is attached")
    // int port = iom.scu_port; // 3 bits; 

    // Why does bootload_tape_label.am claim that a port number belongs in the low bits
    // of the 2nd word of the PCW?  The lower 27 bits of the odd word of a PCW should
    // be all zero.
    /* 9*/ Mem[1] = ((t_uint64) sys_opts.tape_chan << 27) /*| port*/;       // 2nd word of PCW pair

    // following verified correct; instr 362 will not yield 1572 with a different shift
    /*10*/ Mem[2] = ((t_uint64) base_addr << 18) | pi_base | iom_num;   // word after PCW (used by program)

    /*11*/ Mem[3] = (cmd << 30) | (dev << 24) | 0700000;        // IDCW for read binary

    {
    int locs[] = {
        010 + 2 * iom_num, 030 + 2 * iom_num,
        base_addr + 7, base_addr + 010, mbx,
        4, mbx + 2, 0, 1, 2, 3 };

    for (unsigned i = 0; i < ARRAY_SIZE(locs); ++i) {
        int addr = locs[i];
        log_msg(NOTIFY_MSG, "IOM::boot", "Mem[%08o]: %012llo\n",
            addr, Mem[addr]);
        }
    }
}

//=============================================================================

/*
 * init_memory_iox()
 *
 * Not useful; bootload_tape_label.alm will try to execute an undocumented
 * ldo instruction if an IOX is detected.
 *
 */

#if 0

static void init_memory_iox()
{
    int iox_offset = 0;     // 12 bits; not sure what this is...

    int tape_chan = 036;                // 12 bits;
    int port = iom.scu_port;    // 3 bits;  SCU port (to which bootload IOM is attached (deduced))

    iom.channels[tape_chan] = DEVT_TAPE;
    iom.devices[tape_chan] = &tape_dev;

    int base = 014;         // 12 bits; IOM base
    int pi_base = 01200;    // 15 bits; interrupt cells; bootload_io.alm insists that we match template_slt_$iom_mailbox_absloc
    int iom = 0;            // 3 bits; only IOM 0 would use vector 030

    t_uint64 cmd = 5;       // 6 bits; 05 for tape, 01 for cards
    int dev = 0;            // 6 bits: drive number


    t_uint64 imu = 0;       // 1 bit; Maybe an is-IMU flag; IMU is later version of IOM

    //  6/Command, 6/Device#, 6/0, 18/700000
    Mem[0] = (cmd << 30) | (dev << 24) | 0700000;   // Bootload IDCW
    Mem[1] = 030 << 18;                         // Second IDCW: IOTD to loc 30 (startup fault vector)
    // 24/7000000,12/IOXoffset
    Mem[4] = ((t_uint64)07000000 << 12) | iox_offset;       // A register value for connect

    Mem[010] = (1<<18) | 0612000;                   // System fault vector; a HALT instruction
    Mem[030] = (010<<18) | 0612000;             // Terminate interrupt vector (overwritten by bootload)

    // IOX Mailbox
    Mem[001400] = 0;        // base addr 0
    Mem[001401] = 0;        // base addr 1
    Mem[001402] = 0;        // base addr 2
    Mem[001403] = 0;        // base addr 3
    Mem[001404] = ((t_uint64)0777777<<18);  // bound 0, bound 1
    Mem[001405] = 0;                // bound 2, bound 3
    Mem[001406] = 03034;            // channel link word
    Mem[001407] = (0400 << 9) | 0400;   // lpw
        // but what the heck lpw is Chan 01 -- [dcw=00 ires=1 hrel=0 ae=0 nc=0 trun=0 srel=0 tally=0400] [lbnd=00 size=05(5) idcw=020001]

    // The following were set by bootload..
    //Mem[001407] = 001402000002;   // Chan 01 -- [dcw=01402 ires=0 hrel=0 ae=0 nc=0 trun=0 srel=0 tally=02]
    //Mem[001410] = 000000040000;   // [lbnd=00 size=00(0) idcw=040000]
    //Mem[001402] = 000000000000;
    //Mem[001403] = 000000000000;
    //Mem[040000] = 013732054000;

    /* Described in A43A239854_600B_IOM_Spec_Jul75.pdf */
    // LPW for connect channel
    // Mem[001410] = 05040000;  // LPW for connect channel; NC=1; DCW=5
    Mem[001410] = 05020001; // LPW for connect channel; NC=0, tro=1, tally=1, DCW=5
    // PCW for connect channel -- we'll arbitrarily use words 5 and 6
    Mem[5] = 0720201;                   // Bootload channel PCW, word 1 (this is an 18 bit value)
    Mem[6] = ((t_uint64) tape_chan << 27) | port;       // Bootload channel PCW, word 2
    // LPW for bootload channel (channel #5) -- BUG, we probably need one...
    // NOTE: Two DCW words for bootload channel are at location zero
}
#endif

//=============================================================================
//=============================================================================

/*
 *  cancel_run()
 *
 *  Cancel_run can be called by any portion of the code to let
 *  sim_instr() know that it should stop looping and drop back
 *  to the SIMH command prompt.
 */

static t_stat cancel;

void cancel_run(enum sim_stops reason)
{
    // Maybe we should generate an OOB fault?

    (void) sim_cancel_step();
    if (cancel == 0 || (t_stat) reason < cancel)
        cancel = reason;
    log_msg(DEBUG_MSG, "CU", "Cancel requested: %d\n", reason);
}

//=============================================================================

/*
 *  sim_instr()
 *
 *  This function is called by SIMH to execute one or more instructions (or
 *  at least perform one or more processor cycles).
 *
 *  The principal elements of the 6180 processor are:
 *      The appending unit (addressing)
 *      The associative memory assembly (vitual memory registers)
 *      The control unit (responsible for addr mod, instr_mode, interrupt
 *      recognition, decode instr & indir words, timer registers
 *      The operation unit (binary arithmetic, boolean)
 *      The decimal unit (decimal arithmetic instructions, char string, bit
 *      string)
 *
 *  This code is essentially a wrapper for the "control unit" plus
 *  housekeeping such as debug controls, displays, and dropping in
 *  and out of the SIMH command processor.
 */

#define FEATURE_TIME_EXCL_EVENTS 1  // Don't count time spent in sim_process_event()

t_stat sim_instr(void)
{
    restore_from_simh();
    // setup_streams(); // Route the C++ clog and cdebug streams to match SIMH settings

    t_stat reason = 0;

    if (! bootimage_loaded) {
        // We probably should not do this
        // See AL70, section 8
        log_msg(WARN_MSG, "MAIN", "Memory is empty, no bootimage loaded yet\n");
        reason = STOP_MEMCLEAR;
    }

    // opt_debug = (cpu_dev.dctrl != 0);    // todo: should CPU control all debug settings?

    state_invalidate_cache();   // todo: only need to do when changing debug settings

    // Setup clocks
    (void) sim_rtcn_init(CLK_TR_HZ, TR_CLK);
    
    cancel = 0;

    uint32 start_cycles = sys_stats.total_cycles;
    sys_stats.n_instr = 0;
    uint32 start = sim_os_msec();
    uint32 delta = 0;

    // TODO: use sim_activate for the kbd poll
    // if (opt_debug && sim_interval > 32)
    if (opt_debug && sim_interval == NOQUEUE_WAIT) {
        // debug mode is slow, so be more responsive to keyboard interrupt
        sim_interval = 32;
    }

    unsigned prev_seg = PPR.PSR;
    int prev_debug = opt_debug;
    // Loop until it's time to bounce back to SIMH
//log_msg(DEBUG_MSG, "MAIN::CU", "Starting cycle loop; total cycles %lld; sim time is %f, %d events pending, first event at %d\n", sys_stats.total_cycles, sim_gtime(), sim_qcount(), sim_interval);
    while (reason == 0) {
//log_msg(DEBUG_MSG, "MAIN::CU", "Starting cycle %lld; sim time is %f, %d events pending, first event at %d\n", sys_stats.total_cycles, sim_gtime(), sim_qcount(), sim_interval);
        if (PPR.PSR != prev_seg) {
            check_seg_debug();
            prev_seg = PPR.PSR;
        }
        if (sim_interval<= 0) { /* check clock queue */
            // Process any SIMH timed events including keyboard halt
#if FEATURE_TIME_EXCL_EVENTS
            delta += sim_os_msec() - start;
#endif
            reason = sim_process_event();
#if FEATURE_TIME_EXCL_EVENTS
            start = sim_os_msec();
#endif
            if (reason != 0)
                break;
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

        if (prev_debug != opt_debug) {
            // stack tracking depends on being called after every instr
            state_invalidate_cache();
            prev_debug = opt_debug;
        }

        //
        // Log a message about where we're at -- if debugging, or if we want
        // procedure call tracing, or if we're displaying source code
        //

        const int show_source_lines = 1;
        int known_loc = 0;
        uint saved_seg;
        int saved_IC;
        int saved_cycle;
        if (opt_debug || show_source_lines) {
            known_loc = show_location(show_source_lines) == 0;
            saved_seg = PPR.PSR;
            saved_IC = PPR.IC;
            saved_cycle = cpu.cycle;
        }

        //
        // Execute instructions (or fault or whatever)
        //

        reason = control_unit();

        if (saved_cycle != FETCH_cycle) {
            if (!known_loc)
                known_loc = cpu.trgo;
            if (/*known_loc && */ (opt_debug || show_source_lines)) {
                // log_ignore_ic_change();
                show_variables(saved_seg, saved_IC);
                // log_notice_ic_change();
            }
        }

        //
        // And record history, etc
        //

        ++ sys_stats.total_cycles;
        sim_interval--; // todo: maybe only per instr or by brkpoint type?
        if (opt_debug) {
            log_ignore_ic_change();
            state_dump_changes();
            log_notice_ic_change();
            // Save all registers etc so that we can detect/display changes
            state_save();
        }
        if (cancel) {
            if (reason == 0)
                reason = cancel;
#if 0
            if (reason == STOP_DIS) {
                // Until we implement something fancier, DIS will just freewheel...
                cpu.cycle = DIS_cycle;
                reason = 0;
                cancel = 0;
            }
#endif
        }
    }   // while (reason == 0)
//log_msg(DEBUG_MSG, "MAIN::CU", "Finished cycle loop; total cycles %lld; sim time is %f, %d events pending, first event at %d\n", sys_stats.total_cycles, sim_gtime(), sim_qcount(), sim_interval);

    delta += sim_os_msec() - start;
    uint32 ncycles = sys_stats.total_cycles - start_cycles;
    sys_stats.total_msec += delta;
    sys_stats.total_instr += sys_stats.n_instr;
    if (delta > 500)
        log_msg(INFO_MSG, "CU", "Step: %.1f seconds: %d cycles at %d cycles/sec, %d instructions at %d instr/sec\n",
            (float) delta / 1000, ncycles, ncycles*1000/delta, sys_stats.n_instr, sys_stats.n_instr*1000/delta);

    save_to_simh();     // pack private variables into SIMH's world
    flush_logs();

    // Test in control_unit() fails during single step due to "step" timer being on the queue
    if (cpu.cycle == DIS_cycle && ! events.int_pending && sim_qcount() == 0) {
        log_msg(ERR_MSG, "CU", "DIS instruction running, but no activities are pending.\n");
        reason = STOP_BUG;
    }

    return reason;
}


//=============================================================================

/*
 *  save_to_simh
 *  
 *  Some of the data we give to SIMH is in simple encodings instead of
 *  the more complex structures used internal to the emulator.
 *  
*/

static void save_to_simh(void)
{
    // Note that we record the *current* IC and addressing mode.  These may
    // have changed during instruction execution.

    saved_IC = PPR.IC;
    addr_modes_t mode = get_addr_mode();
    saved_PPR = save_PPR(&PPR);
    saved_PPR_addr = addr_emul_to_simh(mode, PPR.PSR, PPR.IC);
    saved_TPR = save_TPR(&TPR);
    t_uint64 sIR;
    save_IR(&sIR);
    saved_IR = sIR & MASKBITS(18);
    set_IR_bitnames(saved_IR);
    save_PR_registers();

    saved_BAR[0] = BAR.base;
    saved_BAR[1] = BAR.bound;
    saved_DSBR = 
        cpup->DSBR.stack | // 12 bits
        (cpup->DSBR.u << 12) | // 1 bit
        ((t_uint64) cpup->DSBR.bound << 13) | // 14 bits
        ((t_uint64) cpup->DSBR.addr << 27); // 24 bits
}

//=============================================================================

/*
 *  set_IR_bitnames
 *  
 *  We only want to display the names of the flags in the IR that are
 *  are currently "on", so we update SIMH's descriptor array to only
 *  include descriptions for the IR bit positions that are on.
 *  
*/

static void set_IR_bitnames(uint32 irval)
{
    static const char *bool_names[] = { "off", "on" };
    static char *IR_bit_names[18] = {
        "zero", "neg", "carry", "overflow",
        "exp-overflow", "exp-underflow", "overflow-mask", "tally-run-out",
        "parity-error", "parity-mask", "not-bar-mode", "truncation",
        "mid-instr-intr-fault", "abs-mode", "hex-mode",
        "", "", ""
    };
    int nfields = 0;
    for (int i = 0; i < 18; ++i) {
        if ((irval >> i) & 1) {
            int idx = 18 - i - 1;
            IR_bits[nfields].name = IR_bit_names[idx];
            IR_bits[nfields].offset = i;
            // TODO: only initialize the following once
            IR_bits[nfields].width = 1;
            IR_bits[nfields].valuenames = bool_names;
            IR_bits[nfields].format = NULL;
            ++nfields;
        }
    }
    IR_bits[nfields].name = NULL;
}

//=============================================================================

/*
 *  restore_from_simh(void)
 *  
 *  Some of the data we give to SIMH is in simple encodings instead of
 *  the more complex structures used internal to the emulator.
 *  
*/

void restore_from_simh(void)
{

    PPR.IC = saved_IC;
    load_IR(&IR, saved_IR);

    restore_PR_registers();
    BAR.base = saved_BAR[0];
    BAR.bound = saved_BAR[1];
    load_PPR(saved_PPR, &PPR);
    PPR.IC = saved_IC;  // allow user to update "IC"
    load_TPR(saved_TPR, &TPR);
    cpup->DSBR.stack = saved_DSBR & MASKBITS(12);
    cpup->DSBR.u = (saved_DSBR >> 12) & 1;
    cpup->DSBR.bound = (saved_DSBR >> 13) & MASKBITS(14);
    cpup->DSBR.addr = (saved_DSBR >> 27) & MASKBITS(24);

    // Set default debug and check for a per-segment debug override
    check_seg_debug();
}

//=============================================================================

/*
 * load_IR()
 *
 * Set the contents of the Indicator Register from the given word.
 *
 */

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

//=============================================================================

/*
 *  save_IR()
 *
 *  Write the contents of the Indicator Register into the given word.
 *
 *  Saves 14 or 15 IR bits to the lower half of *wordp.
 *  Unused portions of destination word are zeroed.
 */

void save_IR(t_uint64* wordp)
{
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

/*
 * save_PPR() & load_PPR()
 *
 * Convert between PPR and bit string
 */

t_uint64 save_PPR(const PPR_t *pprp)
{
    t_uint64 word = pprp->IC & MASK18;          // 18 bits
    word |= (t_uint64) (pprp->P & 1) << 18;     //  1 bit
    word |= (t_uint64) (pprp->PSR & 077777) << 19;  // 15 bits
    word |= (t_uint64) (pprp->PRR & 7) << 34;   //  3 bits
    return word;
}


void load_PPR(t_uint64 word, PPR_t *pprp)
{
    pprp->IC = word & MASK18;       // 18 bits
    pprp->P = (word >> 18) & 1;     //  1 bit
    pprp->PSR = (word >> 19) & 077777;  //  15 bits
    pprp->PRR = (word >> 34) & 7;   //  3 bits
}

//=============================================================================

/*
 * save_TPR() & load_TPR()
 *
 * Convert between TPR and bit string
 */

t_uint64 save_TPR(const TPR_t *tprp)
{
    // Our bit arrangment is arbitrary; real HW spread across multiple CU words
    t_uint64 word = (t_uint64) tprp->CA & MASK18;   // 18 bits
    word |= (t_uint64) tprp->TBR << 18; // 6 bits
    word |= (t_uint64) tprp->TSR << 24; // 15 bits
    word |= (t_uint64) tprp->TRR << 39; // 3 bits
    return word;
}

void load_TPR(t_uint64 word, TPR_t *tprp)
{
    tprp->CA = word & MASK18;       // 18 bits
    tprp->TBR = (word >> 18) & 077;     //  6 bits
    tprp->TSR = (word >> 24) & 077777;  //  15 bits
    tprp->TRR = (word >> 39) & 7;   //  3 bits
}

//=============================================================================

/*
 * save_PR_registers()
 *
 * Convert from the AR/PR structures to an encoded form.
 */

static void save_PR_registers()
{
    for (unsigned i = 0; i < ARRAY_SIZE(saved_ar_pr); ++ i) {
        saved_ar_pr[i] =
            (AR_PR[i].PR.snr & 077777) |            // 15 bits
            ((AR_PR[i].PR.rnr & 07) << 15) |        //  3 bits
            ((AR_PR[i].PR.bitno & 077) << 18) |     //  6 bits
            ((t_uint64)(AR_PR[i].wordno & 0777777) << 24);  // 18 bits
    }
}

//=============================================================================

/*
 *  restore_PR_registers(void)
 *
 *  Convert the encoded values from SIMH into the emulator's structs
 */

static void restore_PR_registers(void)
{
    for (unsigned i = 0; i < ARRAY_SIZE(AR_PR); ++ i) {
        AR_PR[i].PR.snr = saved_ar_pr[i] & MASKBITS(15);
        AR_PR[i].PR.rnr = (saved_ar_pr[i] >> 15) & MASKBITS(3);
        AR_PR[i].PR.bitno = (saved_ar_pr[i] >> 18) & MASKBITS(6);
        AR_PR[i].AR.charno =  AR_PR[i].PR.bitno / 9;
        AR_PR[i].AR.bitno =  AR_PR[i].PR.bitno % 9;
        AR_PR[i].wordno = (saved_ar_pr[i] >> 24);
    }
}

//=============================================================================

/*
 *  control_unit()
 *
 *  Emulation of the control unit -- fetch cycle, execute cycle, 
 *  interrupt handling, fault handling, etc.
 *
 *  We allow SIMH to regain control between any of the cycles of
 *  the control unit.   This includes returning to SIMH on fault
 *  detection and between the even and odd words of a fetched
 *  two word instruction pair.   See cancel_run().
 */

static t_stat control_unit(void)
{
    // ------------------------------------------------------------------------

    // See the following portions of AL39:
    //    Various registers in Section 3: "CONTROL UNIT DATA", IR, Fault, etc
    //    Note word 5 etc in control unit data
    //    All of Section 7

    // SEE ALSO
    //  AN87 -- CPU history registers (interrupt is present, etc)
    //  AN87 -- Mode register's overlap inhibit settings

    // ------------------------------------------------------------------------

    // BUG: Check non group 7 faults?  No, expect cycle to have been reset
    // to FAULT_cycle

    int reason = 0;
    int break_on_fault = switches.FLT_BASE == 2;    // on for multics, off for t&d tape

    switch(cpu.cycle) {
        case DIS_cycle: {
            // TODO: Use SIMH's idle facility
            // Until then, just freewheel
            // 
            // We should probably use the inhibit flag to determine
            // whether or not to examine faults.  However, it appears
            // that we should accept external interrupts regardless of
            // the inhibit flag.   See AL-39 discussion of the timer
            // register for hints.
            if (events.int_pending) {
                cpu.cycle = INTERRUPT_cycle;
                if (cpu.ic_odd && ! cpu.irodd_invalid) {
                    log_msg(NOTIFY_MSG, "CU", "DIS sees an interrupt.\n");
                    log_msg(WARN_MSG, "CU", "Previously fetched odd instruction will be ignored.\n");
                    cancel_run(STOP_WARN);
                } else {
                    log_msg(NOTIFY_MSG, "CU", "DIS sees an interrupt; Auto breakpoint.\n");
                    cancel_run(STOP_IBKPT);
                }
                break;
            }
            // No interrupt pending; will we ever see one?
            uint32 n = sim_qcount();
            if (n > 0) {
                extern UNIT sim_con_unit;
                if (sim_is_active(&sim_con_unit))
                    --n;
            }
            if (n == 0) {
                log_msg(ERR_MSG, "CU", "DIS instruction running, but no activities are pending.\n");
                reason = STOP_BUG;
            } else {
                // WARNING: The only thing in the queue might be a "step"
                log_msg(DEBUG_MSG, "CU", "Delaying until an interrupt is set (%d entries are in the SIMH queue).\n", n);
            }
            break;
        }
            
        case FETCH_cycle:
            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Cycle = FETCH; IC = %0o (%dd)\n", PPR.IC, PPR.IC);
            // If execution of the current pair is complete, the processor
            // checks two? internal flags for group 7 faults and/or interrupts.
            if (events.any) {
                if (cu.IR.inhibit)
                    log_msg(DEBUG_MSG, "CU", "Interrupt or Fault inhibited.\n");
                else {
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
            }
            // Fetch a pair of words
            // AL39, 1-13: for fetches, procedure pointer reg (PPR) is
            // ignored. [PPR IC is a dup of IC]
            cpu.ic_odd = PPR.IC % 2;    // don't exec even if fetch from odd
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            cu.instr_fetch = 1;
            t_uint64 word;
            if (fetch_pair(PPR.IC, &word, &cu.IRODD) != 0) {
                cpu.cycle = FAULT_cycle;
                cpu.irodd_invalid = 1;
            } else {
                decode_instr(word);
                t_uint64 simh_addr = addr_emul_to_simh(get_addr_mode(), PPR.PSR, PPR.IC - PPR.IC % 2);
                cpu.irodd_invalid = 0;
                cpu.cycle = EXEC_cycle;
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
            // to an orderly halt -- TODO -- do we have any?
            cpu.cycle = FAULT_cycle;
            break;
#endif

        case FAULT_cycle:
            {
            log_msg(INFO_MSG, "CU", "Cycle = FAULT\n");

            // First, find the highest fault.   Group 7 type faults are handled
            // as a special case.  Group 7 faults have different detection
            // rules and there can be multiple pending  faults for group 7.

            int fault = 0;
            int group = 0;
            if (events.low_group != 0 && events.low_group <= 6) {
                group = events.low_group;
                fault = events.fault[group];
                if (fault == 0) {
                    log_msg(ERR_MSG, "CU", "Lost fault\n");
                    cancel_run(STOP_BUG);
                }
            }
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
            ic_history_add_fault(fault);
            log_msg(DEBUG_MSG, "CU", "fault = %d (group %d)\n", fault, group);
            if (fault != trouble_fault)
                cu_safe_store();

            // Faults cause the CPU to go into absolute mode.  The CPU will
            // remain in absolute mode until execution of a transfer instr
            // whose operand is obtained via explicit use of the appending
            // HW mechanism -- AL39, 1-3
            set_addr_mode(ABSOLUTE_mode);

            // We found a fault.
            // BUG: should we clear the fault?  Or does scr instr do that? Or
            // maybe the OS's fault handling routines do it?   At the
            // moment, we clear it and find the next pending fault.
            // Above was written before we had FR and scpr; perhaps
            // we need to revist the transitions to/from FAULT cycles.
            // AL-39, section 1 says that faults and interrupts are
            // cleared when their trap occurs (that is when the CPU decides
            // to recognize a perhaps delayed signal).
            // Morever, AN71-1 says that the SCU clears the interrupt cell after
            // reporting to the CPU the value of the highest priority interrupt.

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

            // Force computed addr and xed opcode into the instruction
            // register and execute (during FAULT CYCLE not EXECUTE CYCLE).
            // The code below is much the same for interrupts and faults...

            PPR.PRR = 0;    // set ring zero
            uint addr = (switches.FLT_BASE << 5) + 2 * fault; // ABSOLUTE mode
            cu.IR.addr = addr;
            cu.IR.opcode = (opcode0_xed << 1);
            cu.IR.inhibit = 1;
            cu.IR.mods.single.pr_bit = 0;
            cu.IR.mods.single.tag = 0;
            cu.IR.is_eis_multiword = 0;
            TPR.CA = cu.IR.addr;
            TPR.is_value = 0;
            TPR.TBR = 0;

            // Maybe instead of calling execute_ir(), we should just set a
            // flag and run the EXEC case?  // Maybe the following increments
            // and tests could be handled by EXEC and/or the XED opcode?

            // Update history (show xed at current location; next two will be addr and addr+1
            //uint IC_temp = PPR.IC;
            //PPR.IC = addr;
            ic_history_add();       // record the xed
            //PPR.IC = IC_temp;

            // TODO: Check for SIMH breakpoint on execution for the addr of
            // the XED instruction.  Or, maybe check in the code for the
            // xed opcode.
            log_msg(DEBUG_MSG, "CU::fault", "calling execute_ir() for xed\n");
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
                // cycle = FAULT_EXEC_cycle;
                //cpu.cycle = EXEC_cycle;       // NOTE: scu will be in EXEC not FAULT cycle
                cpu.cycle = FAULT_EXEC_cycle;
                events.xed = 1;     // BUG: is this a hack?
            }
            } // end case FAULT_cycle
            break;

        case INTERRUPT_cycle: {
            // This code is just a quick-n-dirty sketch to test booting via
            // an interrupt instead of via a fault

            // TODO: Merge the INTERRUPT_cycle and FAULT_cycle code

            // The CPU will
            // remain in absolute mode until execution of a transfer instr
            // whose operand is obtained via explicit use of the appending
            // HW mechanism -- AL39, 1-3
            set_addr_mode(ABSOLUTE_mode);

            log_msg(WARN_MSG, "CU", "Interrupts only partially implemented\n");
            int intr;
            for (intr = 0; intr < 32; ++intr)
                if (events.interrupts[intr])
                    break;
            if (intr == 32) {
                log_msg(ERR_MSG, "CU", "Interrupt cycle with no pending interrupt.\n");
                // BUG: Need error handling
            }
            ic_history_add_intr(intr);
            log_msg(WARN_MSG, "CU", "Interrupt %#o (%d) found.\n", intr, intr);
            events.interrupts[intr] = 0;

            // Force computed addr and xed opcode into the instruction
            // register and execute (during INTERRUPT CYCLE not EXECUTE CYCLE).
            // The code below is much the same for interrupts and faults...

            PPR.PRR = 0;    // set ring zero
            const int interrupt_base = 0;
            uint addr = interrupt_base + 2 * intr; // ABSOLUTE mode
            cu.IR.addr = addr;
            cu.IR.opcode = (opcode0_xed << 1);
            cu.IR.inhibit = 1;
            cu.IR.mods.single.pr_bit = 0;
            cu.IR.mods.single.tag = 0;
            cu.IR.is_eis_multiword = 0;
            TPR.CA = cu.IR.addr;
            TPR.is_value = 0;
            TPR.TBR = 0;

            // Maybe instead of calling execute_ir(), we should just set a
            // flag and run the EXEC case?  // Maybe the following increments
            // and tests could be handled by EXEC and/or the XED opcode?

            // Update history
            uint IC_temp = PPR.IC;
            // PPR.IC = 0;
            ic_history_add();       // record the xed
            PPR.IC = IC_temp;

            // TODO: Check for SIMH breakpoint on execution for the addr of
            // the XED instruction.  Or, maybe check in the code for the
            // xed opcode.
            log_msg(DEBUG_MSG, "CU::interrupt", "calling execute_ir() for xed\n");
            execute_ir();   // executing in INTERRUPT CYCLE, not EXECUTE CYCLE
            log_msg(WARN_MSG, "CU", "Interrupt -- lightly tested\n");
            (void) cancel_run(STOP_IBKPT);

            // We executed an XED just above.  XED set various CPU flags.
            // So, now, set the CPU into the EXEC cycle so that the
            // instructions referenced by the XED will be executed.
            // cpu.cycle = FAULT_EXEC_cycle;
            // cpu.cycle = EXEC_cycle;
            cpu.cycle = FAULT_EXEC_cycle;
            events.xed = 1;     // BUG: is this a hack?
            events.int_pending = 0;     // FIXME: make this a counter
            for (intr = 0; intr < 32; ++intr)
                if (events.interrupts[intr]) {
                    events.int_pending = 1;
                    break;
                }
            if (!events.int_pending && ! events.low_group && ! events.group7)
                events.any = 0;
            break;
        }   // end case INTERRUPT_cycle

        case EXEC_cycle:
            // Assumption: IC will be at curr instr, even
            // when we're ready to execute the odd half of the pair.
            // Note that the fetch cycle sets the cpu.ic_odd flag.
            TPR.TSR = PPR.PSR;
            TPR.TRR = PPR.PRR;
            
            // Fall through -- FAULT_EXEC_cycle is a subset of EXEC_cycle
    
        case FAULT_EXEC_cycle:
            {
            // FAULT-EXEC is a pseudo cycle not present in the actual
            // hardware.  The hardware does execute intructions (e.g. a
            // fault's xed) in a FAULT cycle.   We should be able to use the
            // FAULT-EXEC cycle for this to gain code re-use.

            flag_t do_odd = 0;

            // We need to know if we should execute an instruction from the
            // normal fetch process or an instruction loaded by XDE.  The
            // answer controls whether we execute a buffered even instruction
            // or a buffered odd instruction.
            int doing_xde = cu.xde;
            int doing_xdo = cu.xdo;
            if (doing_xde) {
                if (cu.xdo)     // xec too common
                    log_msg(INFO_MSG, "CU", "XDE-EXEC even\n");
                else
                    log_msg(DEBUG_MSG, "CU", "XDE-EXEC even\n");
            } else if (doing_xdo) {
                log_msg(INFO_MSG, "CU", "XDE-EXEC odd\n");
                do_odd = 1;
            } else if (! cpu.ic_odd) {
                if (opt_debug)
                    log_msg(DEBUG_MSG, "CU", "Cycle = EXEC, even instr\n");
            } else {
                if (opt_debug)
                    log_msg(DEBUG_MSG, "CU", "Cycle = EXEC, odd instr\n");
                do_odd = 1;
            }
            if (do_odd) {
                // Our previously buffered odd location instruction may have
                // later been invalidated by a write to that location.
                if (cpu.irodd_invalid) {
                    cpu.irodd_invalid = 0;
                    if (cpu.cycle != FETCH_cycle) {
                        reason = STOP_IBKPT;    /* stop simulation */
                        log_msg(NOTIFY_MSG, "CU", "Invalidating cached odd instruction; auto breakpoint\n");
                        cpu.cycle = FETCH_cycle;
                    }
                    break;
                }
                decode_instr(cu.IRODD);
            }

            // Do we have a breakpoint here?
            if (sim_brk_summ) {
                t_uint64 simh_addr = addr_emul_to_simh(get_addr_mode(), PPR.PSR, PPR.IC);
                if (sim_brk_test (simh_addr, SWMASK ('E'))) {
                    // BUG: misses breakpoints on target of xed, rpt, and
                    // similar instructions because those instructions don't
                    // update the IC.  Some of those instructions
                    // do however provide their own breakpoint checks.
                    log_msg(WARN_MSG, "CU", "Execution Breakpoint\n");
                    reason = STOP_IBKPT;    /* stop simulation */
                    break;
                }
            }

            // We assume IC always points to the correct instr -- should
            // be advanced after even instr
            int IC_temp = PPR.IC;
            // Munge PPR.IC for history debug
            if (cu.xde)
                PPR.IC = TPR.CA;
            else 
                if (cu.xdo) {
                    // BUG: This lie may be wrong if prior instr updated TPR.CA, so
                    // we should probably remember the prior xde addr
                    PPR.IC = TPR.CA + 1;
                }
            ic_history_add();
            PPR.IC = IC_temp;
            execute_ir();

            // Check for fault from instr
            // todo: simplify --- cycle won't be EXEC anymore
            // Note: events.any zeroed by fault handler prior to xed even
            // if other events are pending, so if it's on now, we have a
            // new fault

            // Only fault groups 1-6 are recognized here, not interrupts or
            // group 7 faults
            flag_t is_fault = events.any && events.low_group && events.low_group < 7;
            if (is_fault) {
                log_msg(WARN_MSG, "CU", "Fault detected after instruction execution\n");
                if (PPR.IC != IC_temp) {
                    // Our OPU always advances the IC, but should not do so
                    // on faults, so we restore it
                    log_msg(INFO_MSG, "CU", "Restoring IC to %06o (from %06o)\n",
                        IC_temp, PPR.IC);
                    // Note: Presumably, none of the instructions that change the PPR.PSR
                    // are capable of generating a fault after doing so...
                    PPR.IC = IC_temp;
                }
                if (doing_xde || doing_xdo) {
                    char *which = doing_xde ? "even" : "odd";
                    log_msg(WARN_MSG, "CU", "XED %s instruction terminated by fault.\n", which);
                    // Note that we don't clear xde and xdo because they might
                    // be about to be stored by scu. Since we'll be in a FAULT
                    // cycle next, both flags will be set as part of the fault
                    // handler's xed
                }
                if (cu.rpt || cu.rd) {
                    log_msg(WARN_MSG, "CU", "Repeat instruction terminated by fault.\n");
                    cu.rpt = 0;
                    cu.rd = 0;
                    extern DEVICE cpu_dev; if(opt_debug) {-- opt_debug; -- cpu_dev.dctrl;};
                }
            } else {
                /* No Fault */
                // First, Special handling for RPT and other "repeat" instructions
                if (cu.rpt || cu.rd) {
                    if (cu.rpts) {
                        // Just executed the RPT instr.  
                        cu.rpts = 0;    // finished "starting"
                        ++ PPR.IC;
                        if (! cpu.ic_odd)
                            cpu.ic_odd = 1;
                        else
                            cpu.cycle = FETCH_cycle;
                    } else {
                        // Executed a repeated instruction
                        // log_msg(WARN_MSG, "CU", "Address handing for repeated instr was probably wrong.\n");
                        // Check for tally runout or termination conditions
                        uint t = reg_X[0] >> 10; // bits 0..7 of 18bit register
                        if (cu.repeat_first) {
                            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Repeat-first flag is on.\n");
                            // Instr RD gets two repeat firsts -- one for the even instr and one for the odd
                            // FIXME: For RPD, should odd addr calc occur before even instr exec?  Guessing not...
                            if (cu.rpt || (cu.rd && ! do_odd)) {
                                if (t == 0)
                                    t = 256;
                                cu.repeat_first = 0;
                                if (opt_debug) log_msg(DEBUG_MSG, "CU", "Turning off repeat-first flag.\n");
                            }
                        }
                        --t;
                        reg_X[0] = ((t&0377) << 10) | (reg_X[0] & 01777);
                        // Note that we increment X[n] here, not in the APU.
                        // So, for instructions like cmpaq, the index register
                        // points to the entry after the one found.
                        int n = cu.IR.mods.single.tag & 07;
                        if (cu.rpt) {
                            reg_X[n] += cu.delta;
                            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Incrementing X[%d] by %#o to %#o.\n", n, cu.delta, reg_X[n]);
                        } else if (cu.rd) {
                            if (! do_odd) {
                                if (((reg_X[0] >> 9) & 1) == 1) {
                                    reg_X[n] += cu.delta;
                                    log_msg(INFO_MSG, "CU", "Incrementing X[%d] by %#o to %#o.\n", n, cu.delta, reg_X[n]);
                                } else
                                    log_msg(INFO_MSG, "CU", "Not Incrementing X[%d] by %#o; still %#o.\n", n, cu.delta, reg_X[n]);
                            } else {
                                if (((reg_X[0] >> 8) & 1) == 1) {
                                    reg_X[n] += cu.delta;
                                    log_msg(INFO_MSG, "CU", "Incrementing X[%d] by %#o to %#o.\n", n, cu.delta, reg_X[n]);
                                } else
                                    log_msg(INFO_MSG, "CU", "Not Incrementing X[%d] by %#o; still %#o.\n", n, cu.delta, reg_X[n]);
                            }
                        }
                        // Note that the code in bootload_tape.alm expects that
                        // the tally runout *not* be set when both the
                        // termination condition is met and bits 0..7 of
                        // reg X[0] hits zero.
                        const flag_t orig_rpt = cu.rpt;
                        const flag_t orig_rd = cu.rd;
                        if (t == 0) {
                            IR.tally_runout = 1;
                            cu.rpt = 0;
                            cu.rd = 0;
                            if (opt_debug) log_msg(DEBUG_MSG, "CU", "Repeated instruction hits tally runout; halting rpt.\n");
                            extern DEVICE cpu_dev; -- opt_debug; -- cpu_dev.dctrl;
                        }
                        // Check for termination conditions -- even if we hit
                        // the tally runout
                        // Note that register X[0] is 18 bits
                        int terminate = 0;
                        if (orig_rpt || (orig_rd && do_odd)) {
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
                                // Process overflows -- BUG: what are all the
                                // types of overflows?
                                if (IR.overflow || IR.exp_overflow) {
                                    if (IR.overflow_mask)
                                        IR.overflow = 1;
                                    else
                                        fault_gen(overflow_fault);
                                    terminate = 1;
                                }
                            }
                        }
                        if (terminate) {
                            cu.rpt = 0;
                            cu.rd = 0;
                            log_msg(DEBUG_MSG, "CU", "Repeated instruction meets termination condition.\n");
                            IR.tally_runout = 0;
                            extern DEVICE cpu_dev; if(opt_debug) {-- opt_debug; -- cpu_dev.dctrl;};
                            // BUG: need IC incr, etc
                        } else {
                            if (! IR.tally_runout)
                                if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Repeated instruction will continue.\n");
                        }
                        if (cu.rd) {
                            if (do_odd) {
                                -- PPR.IC;
                                if (opt_debug>0) log_msg(DEBUG_MSG, "CU", "Resetting IC to %#o for next loop.\n", PPR.IC);
                            }
                        }
                    }
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
                        if (cu.IR.is_eis_multiword) {
                            log_msg(WARN_MSG, "CU", "XEC/XED may mishandle EIS MW instructions; IC changed from %#o to %#o\n", IC_temp, PPR.IC);
                            (void) cancel_run(STOP_BUG);
                            -- PPR.IC;
                        }
                        if (cu.xdo)
                            log_msg(INFO_MSG, "CU", "Resetting XED even flag\n");
                        else {
                            // xec is very common, so use level debug
                            log_msg(DEBUG_MSG, "CU", "Resetting XED even flag\n");
                            ++ PPR.IC;
                        }
                        // BUG? -- do we need to reset events.xed if cu.xdo
                        // isn't set?  -- but xdo must be set unless xed
                        // doesn't really mean double...
                    }
                } else if (doing_xdo) {
                    log_msg(INFO_MSG, "CU", "Resetting XED odd flag\n");
                    cu.xdo = 0;
                    if (events.xed) {
                        events.xed = 0;
                        if (events.any)
                            log_msg(NOTIFY_MSG, "CU", "XED was from fault or interrupt; other faults and/or interrupts occured during XED\n");
                        else {
                            log_msg(NOTIFY_MSG, "CU", "XED was from fault or interrupt; checking if lower priority faults exist\n");
                            check_events();
                        }
                    }
                    cpu.cycle = FETCH_cycle;
                    if (!cpu.trgo) {
                        if (PPR.IC == IC_temp)
                            ++ PPR.IC;
                        else
                            if (cu.IR.is_eis_multiword)
                                log_msg(INFO_MSG, "CU", "Not updating IC after XED because EIS MW instruction updated the IC from %#o to %#o\n", IC_temp, PPR.IC);
                            else
                                log_msg(WARN_MSG, "CU", "No transfer instruction in XED, but IC changed from %#o to %#o\n", IC_temp, PPR.IC);
                    }
                } else if (! cpu.ic_odd) {
                    // Performed non-repeat instr at even loc (or finished the
                    // last repetition)
                    if (cpu.cycle == EXEC_cycle) {
                            // After an xde, we'll increment PPR.IC.   Setting
                            // cpu.ic_odd will be ignored.
                            if (cpu.trgo) {
                                // IC changed; previously fetched instr for odd location isn't any good now
                                cpu.cycle = FETCH_cycle;
                            } else {
                                if (PPR.IC == IC_temp) {
                                    // cpu.ic_odd ignored if cu.xde or cu.xdo
                                    cpu.ic_odd = 1; // execute odd instr of current pair
                                    if (! cu.xde && ! cu.xdo)
                                        ++ PPR.IC;
                                    else
                                        log_msg(DEBUG_MSG, "CU", "Not advancing IC after even instr because of xde/xdo\n");
                                } else {
                                    if (cpu.irodd_invalid) {
                                        // possibly an EIS multi-word instr
                                    } else
                                        log_msg(NOTIFY_MSG, "CU", "No transfer instruction and IRODD not invalidated, but IC changed from %#o to %#o; changing to fetch cycle\n", IC_temp, PPR.IC);
                                    cpu.cycle = FETCH_cycle;
                                }
                            }
                    } else {
                        log_msg(WARN_MSG, "CU", "Changed from EXEC cycle to %d, not updating IC\n", cpu.cycle);
                    }
                } else {
                    // Performed non-repeat instr at odd loc (or finished last
                    // repetition)
                    if (cpu.cycle == EXEC_cycle) {
                        if (cpu.trgo) {
                            cpu.cycle = FETCH_cycle;
                        } else {
                            if (PPR.IC == IC_temp) {
                                if (cu.xde || cu.xdo)
                                    log_msg(INFO_MSG, "CU", "Not advancing IC or fetching because of cu.xde or cu.xdo.\n");
                                else {
                                    cpu.ic_odd = 0; // finished with odd half; BUG: restart issues?
                                    ++ PPR.IC;
                                    cpu.cycle = FETCH_cycle;
                                }
                            } else {
                                if (!cpu.irodd_invalid)
                                    log_msg(NOTIFY_MSG, "CU", "No transfer instruction and IRODD not invalidated, but IC changed from %#o to %#o\n", IC_temp, PPR.IC);  // DEBUGGING; BUG: this shouldn't happen?
                                cpu.cycle = FETCH_cycle;
                            }
                        }
                    } else {
                        log_msg(NOTIFY_MSG, "CU", "Cycle is %d after EXEC_cycle\n", cpu.cycle);
                        //cpu.cycle = FETCH_cycle;
                    }
                }
            }   // if (! is_fault)
            }   // case FAULT_EXEC_cycle
            break;
        default:
            log_msg(ERR_MSG, "CU", "Unknown cycle # %d\n", cpu.cycle);
            reason = STOP_BUG;
    }   // switch(cpu.cycle)

    return reason;
}


//=============================================================================

/*
 *  execute_ir()
 *
 *  execute whatever instruction is in the IR instruction register (and
 *  not whatever the IC points at)
 */

static void execute_ir(void)
{
    cpu.trgo = 0;       // will be set true by instructions that alter flow
    execute_instr();    // located in opu.c
}

//=============================================================================

/*
 *  check_events()
 *
 *  Called after executing an instruction pair for xed.   The instruction pair
 *  may have included a rpt, rpd, or transfer.   The instruction pair may even
 *  have faulted, but if so, it was saved and restarted.
 */

static void check_events()
{
    events.any = events.int_pending || events.low_group || events.group7;
    if (events.any)
        log_msg(NOTIFY_MSG, "CU", "check_events: event(s) found (%d,%d,%d).\n", events.int_pending, events.low_group, events.group7);

    return;
}


//=============================================================================

/*
 *  fault_gen()
 *
 *  Called by instructions or the addressing code to record the
 *  existance of a fault condition.
 */

void fault_gen(enum faults f)
{
    int group;

#if 0
    if (f == oob_fault) {
        log_msg(ERR_MSG, "CU::fault", "Faulting for internal bug\n");
        f = trouble_fault;
        (void) cancel_run(STOP_BUG);
    }
#endif

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

    if (f == illproc_fault)
        FR |= fr_ill_proc;
 
    events.any = 1;
    log_msg(DEBUG_MSG, "CU::fault", "Recording fault # %d in group %d\n", f, group);

    // Note that we never simulate a (hardware) op_not_complete_fault
    if (MR.mr_enable && (f == op_not_complete_fault || MR.fault_reset)) {
        if (MR.strobe) {
            log_msg(INFO_MSG, "CU::fault", "Clearing MR.strobe.\n");
            MR.strobe = 0;
        } else
            log_msg(INFO_MSG, "CU::fault", "MR.strobe was already unset.\n");
    }

    if (group == 7) {
        // Recognition of group 7 faults is delayed and we can have
        // multiple group 7 faults pending.
        events.group7 |= (1 << f);
    } else {
        // Groups 1-6 are handled more immediately and there can only be
        // one fault pending within each group
        //if (cpu.cycle == FAULT_cycle)
        if (cpu.cycle == FAULT_cycle || cpu.cycle == FAULT_EXEC_cycle) {
// FIXME: || events.xed AND/OR || cpu.cycle == FAULT_EXEC_cycle
            f = trouble_fault;
            group = fault2group[f];
            log_msg(WARN_MSG, "CU::fault", "Double fault:  Recording current fault as a trouble fault (fault # %d in group %d).\n", f, group);
            cpu.cycle = FAULT_cycle;
            //cancel_run(STOP_DIS); // BUG: not really
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

/*
 * fault_check()
 *
 * No longer used.
 */

#if 0

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
    int group = fault2group[f]; // result is 0..7
    if (group == 7)
        return events.group7 & (1 << f);
    else if (group < 1 || group > 6) {
        log_msg(ERR_MSG, "CU::fault-check", "Fault # %d has bad group %d\n", f, group);
        cancel_run(STOP_BUG);
        return 1;
    } else
        return events.fault[group] == f;
}

#endif

//=============================================================================

/*
 * fault_check_group
 *
 * Returns true if faults exist for the specifed group or for a higher
 * priority group.
 *
 */

int fault_check_group(int group)
{

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

/*
 * fetch_word()
 *
 * Fetches a word at the specified address according to the current
 * addressing mode.
 *
 * Returns non-zero if a fault in groups 1-6 is detected
 */

int fetch_word(uint addr, t_uint64 *wordp)
{

    addr_modes_t mode = get_addr_mode();

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


//=============================================================================

/*
 * fetch_abs_word(uint addr, t_uint64 *wordp)
 *
 * Fetch word at given 24-bit absolute address.
 * Returns non-zero if a fault in groups 1-6 is detected
 *
 */

int fetch_abs_word(uint addr, t_uint64 *wordp)
{

#define CONFIG_DECK_LOW 012000
#define CONFIG_DECK_LEN 010000

    // TODO: Efficiency: If the compiler doesn't do it for us, the tests
    // below should be combined under a single min/max umbrella.
    if (addr >= IOM_MBX_LOW && addr < IOM_MBX_LOW + IOM_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from IOM mailbox area for addr %#o\n", addr);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from DN355 mailbox area for addr %#o\n", addr);
    }
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from CONFIG DECK area for addr %#o\n", addr);
    }
    if (addr <= 030) {
        log_msg(DEBUG_MSG, "CU::fetch", "Fetch from 0..030 for addr %#o\n", addr);
    }

    if (addr >= MAXMEMSIZE) {
            log_msg(ERR_MSG, "CU::fetch", "Addr %#o (%d decimal) is too large\n", addr, addr);
            (void) cancel_run(STOP_BUG);
            return 1;
    }

    if (sim_brk_summ) {
        // Check for absolute mode breakpoints.  Note that fetch_appended()
        // has its own test for appending mode breakpoints.
        t_uint64 simh_addr = addr_emul_to_simh(ABSOLUTE_mode, 0, addr);
        if (cpu.cycle != FETCH_cycle && sim_brk_test (simh_addr, SWMASK ('D'))) {
            out_sym(0, simh_addr, &Mem[addr], &cpu_unit, SWMASK('M') | SWMASK('A'));
        }
        if (sim_brk_test (simh_addr, SWMASK ('M'))) {
            log_msg(WARN_MSG, "CU::fetch", "Memory Breakpoint, address %#o.  Fetched value %012llo\n", addr, Mem[addr]);
            (void) cancel_run(STOP_IBKPT);
        }
    }

    cpu.read_addr = addr;   // Should probably be in scu
#if FEAT_MEM_CHECK_UNINIT
    {
    t_uint64 word = Mem[addr];  // absolute memory reference
    if (word == ~ (t_uint64) 0) {
        word = 0;
        if (sys_opts.warn_uninit)
            log_msg(WARN_MSG, "CU::fetch", "Fetch from uninitialized absolute location %#o.\n", addr);
    }
    *wordp = word;
    }
#else
    *wordp = Mem[addr]; // absolute memory reference
#endif
    if (get_addr_mode() == BAR_mode)
        log_msg(DEBUG_MSG, "CU::fetch-abs", "fetched word at %#o\n", addr);
    return 0;
}

//=============================================================================

/*
 * store_word()
 *
 * Store a word to the specified address according to the current
 * addressing mode.
 *
 * Returns non-zero if a fault in groups 1-6 is detected
 */

int store_word(uint addr, t_uint64 word)
{

    addr_modes_t mode = get_addr_mode();

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
        // impossible
        log_msg(ERR_MSG, "CU::store", "Addr=%#o:  Unknown addr mode %d.\n", addr, mode);
        cancel_run(STOP_BUG);
        return 1;
    }
}

//=============================================================================

/*
 * store-abs_word()
 *
 * Store word to the given 24-bit absolute address.
 */

int store_abs_word(uint addr, t_uint64 word)
{

    // TODO: Efficiency: If the compiler doesn't do it for us, the tests
    // below should be combined under a single min/max umbrella.
    if (addr >= IOM_MBX_LOW && addr < IOM_MBX_LOW + IOM_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to IOM mailbox area for addr %#o\n", addr);
    }
    if (addr >= DN355_MBX_LOW && addr < DN355_MBX_LOW + DN355_MBX_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to DN355 mailbox area for addr %#o\n", addr);
    }
    if (addr >= CONFIG_DECK_LOW && addr < CONFIG_DECK_LOW + CONFIG_DECK_LEN) {
        log_msg(DEBUG_MSG, "CU::store", "Store to CONFIG DECK area for addr %#o\n", addr);
    }
    if (addr <= 030) {
        //log_msg(DEBUG_MSG, "CU::store", "Fetch from 0..030 for addr %#o\n", addr);
    }

    if (addr >= MAXMEMSIZE) {
            log_msg(ERR_MSG, "CU::store", "Addr %#o (%d decimal) is too large\n");
            (void) cancel_run(STOP_BUG);
            return 1;
    }
    if (sim_brk_summ) {
        // Check for absolute mode breakpoints.  Note that store_appended()
        // has its own test for appending mode breakpoints.
        t_uint64 simh_addr = addr_emul_to_simh(ABSOLUTE_mode, 0, addr);
        uint mask;
        if ((mask = sim_brk_test(simh_addr, SWMASK('W') | SWMASK('M') | SWMASK('E'))) != 0) {
            if (sim_brk_test (simh_addr, SWMASK ('D'))) {
                out_sym(1, simh_addr, &Mem[addr], &cpu_unit, SWMASK('M') | SWMASK('A'));
            }
            if ((mask & SWMASK ('W')) != 0) {
                log_msg(NOTIFY_MSG, "CU::store", "Memory Write Breakpoint, address %#o\n", addr);
                (void) cancel_run(STOP_IBKPT);
            } else if ((mask & SWMASK ('M')) != 0) {
                log_msg(NOTIFY_MSG, "CU::store", "Memory Breakpoint, address %#o\n", addr);
                (void) cancel_run(STOP_IBKPT);
            } else if ((mask & SWMASK ('E')) != 0) {
                log_msg(NOTIFY_MSG, "CU::store", "Write to a location that has an execution breakpoint, address %#o\n", addr);
            } else {
                log_msg(NOTIFY_MSG, "CU::store", "Write to a location that has an unknown type of breakpoint, address %#o\n", addr);
                (void) cancel_run(STOP_IBKPT);
            }
            log_msg(INFO_MSG, "CU::store", "Address %08o: value was %012llo, storing %012llo\n", addr, Mem[addr], word);
        }
    }

    Mem[addr] = word;   // absolute memory reference
    if (addr == cpu.IC_abs) {
        log_msg(NOTIFY_MSG, "CU::store", "Flagging cached odd instruction from %o as invalidated.\n", addr);
        cpu.irodd_invalid = 1;
    }
    if (get_addr_mode() == BAR_mode)
        log_msg(DEBUG_MSG, "CU::store-abs", "stored word to %#o\n", addr);
    return 0;
}

//=============================================================================

/*
 * store_abs_pair()
 * 
 * Store to even and odd words at Y-pair given by 24-bit absolute address
 * addr.  Y-pair addresses are always constrained such that the first
 * address used will be even.
 *
 * Returns non-zero if fault in groups 1-6 detected
 */

int store_abs_pair(uint addr, t_uint64 word0, t_uint64 word1)
{

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

//=============================================================================

/*
 * store_pair()
 * 
 * Store to even and odd words at given Y-pair address according to the
 * current addressing mode.
 * Y-pair addresses are always constrained such that the first address used
 * will be even.
 *
 * Returns non-zero if fault in groups 1-6 detected
 *
 * BUG: Is it that the given y-pair offset must be even or is it that the
 * final computed addr must be even?   Note that the question may be moot --
 * it might be that segements always start on an even address and contain
 * an even number of words.
 */

int store_pair(uint addr, t_uint64 word0, t_uint64 word1)
{

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

//=============================================================================

/*
 * fetch_abs_pair()
 *
 * Fetch even and odd words at Y-pair given by the specified 24-bit absolute
 * address.
 *
 * Returns non-zero if fault in groups 1-6 detected
 */

int fetch_abs_pair(uint addr, t_uint64* word0p, t_uint64* word1p)
{

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

//=============================================================================

/*
 * fetch_pair()
 * 
 * Fetch from the even and odd words at given Y-pair address according to the
 * current addressing mode.
 * Y-pair addresses are always constrained such that the first address used
 * will be even.
 *
 * Returns non-zero if fault in groups 1-6 detected
 *
 * BUG: see comments at store_pair() re what does "even" mean?
 */

int fetch_pair(uint addr, t_uint64* word0p, t_uint64* word1p)
{
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

//=============================================================================

/*
 * fetch_yblock()
 * 
 * Aligned or un-aligned fetch from the given Y-block address according to
 * the current addressing mode.
 *
 * Returns non-zero if fault in groups 1-6 detected
 *
 * BUG: What does "aligned" mean for appending mode?  See comments at
 * store_pair().
 */

int fetch_yblock(uint addr, int aligned, uint n, t_uint64 *wordsp)
{
    int ret;
    uint Y = (aligned) ? (addr / n) * n : addr;

    for (uint i = 0; i < n; ++i)
        if ((ret = fetch_word(Y++, wordsp++)) != 0)
            return ret;
    return 0;
}


//=============================================================================

/*
 * fetch_yblock8()
 * 
 * Aligned fetch from the given Y-block-8 address according to
 * the current addressing mode.
 *
 * Returns non-zero if fault in groups 1-6 detected
 *
 * BUG: What does "aligned" mean for appending mode?  See comments at
 * fetch_yblock() and store_pair().
 */

int fetch_yblock8(uint addr, t_uint64 *wordsp)
{
    return fetch_yblock(addr, 1, 8, wordsp);
}


//=============================================================================

/*
 * store_yblock()
 * 
 * Aligned or un-aligned store to the given Y-block address according to
 * the current addressing mode.
 *
 * Returns non-zero if fault in groups 1-6 detected
 *
 * BUG: What does "aligned" mean for appending mode?  See comments at
 * store_pair().
 */

static int store_yblock(uint addr, int aligned, int n, const t_uint64 *wordsp)
{
    int ret;
    uint Y = (aligned) ? (addr / n) * n : addr;

    for (int i = 0; i < n; ++i)
        if ((ret = store_word(Y++, *wordsp++)) != 0)
            return ret;
    return 0;
}

//=============================================================================

int store_yblock8(uint addr, const t_uint64 *wordsp)
{
    return store_yblock(addr, 1, 8, wordsp);
}

//=============================================================================

int store_yblock16(uint addr, const t_uint64 *wordsp)
{
    return store_yblock(addr, 1, 16, wordsp);
}


//=============================================================================

/*
 * word2_instr()
 *
 * Convert a 36-bit word into a instr_t struct.
 * 
 */

void word2instr(t_uint64 word, instr_t *ip)
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

/*
 * decode_instr()
 *
 * Convert a 36-bit word into a instr_t struct and perform initial address movement.
 * 
 */


void decode_instr(t_uint64 word)
{
    const char* moi = "CU::decode";
    word2instr(word, &cu.IR);
    // Note that the CU doesn't do the bit 29 handling; that's done by the APU.
    TPR.CA = cu.IR.addr;
    TPR.is_value = 0;
    TPR.TBR = 0;

    if (cu.rpt || cu.rd) {
        instr_t* ip = &cu.IR;
        // Special handling for repeat instructions
        // TPR.TBR = 0;
        if (cu.repeat_first) {
            if (opt_debug)
                log_msg((cu.rd) ? INFO_MSG : DEBUG_MSG, moi,
                    "RP*: First repetition; incr will be 0%o(%d).\n",
                    ip->addr, ip->addr);
            // TPR.CA = ip->addr;
        } else {
            TPR.CA = 0;
            if (opt_debug) {
                uint td = cu.IR.mods.single.tag & 017;
                int n = td & 07;
                log_msg((cu.rd) ? INFO_MSG : DEBUG_MSG, moi,
                    "RP*: X[%d] is 0%o(%d).\n", n, reg_X[n], reg_X[n]);
            }
        }
    }
}

//=============================================================================

/*
 * encode_instr()
 *
 * Convert an instr_t struct into a  36-bit word.
 * 
 */

void encode_instr(const instr_t *ip, t_uint64 *wordp)
{
        *wordp = setbits36(0, 0, 18, ip->addr);
#if 1
        *wordp = setbits36(*wordp, 18, 10, ip->opcode);
#else
        *wordp = setbits36(*wordp, 18, 9, ip->opcode & 0777);
        *wordp = setbits36(*wordp, 27, 1, ip->opcode >> 9);
#endif
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

/*
 * cpu_ex()
 *
 * Called by SIMH to retrieve memory.
 *
 * WARNING: SIMH Documention is incorrect; SIMH code expects examine() to only
 * write a single value (often a member of an array of size sim_emax)
 */

static t_stat cpu_ex (t_value *eval_array, t_addr simh_addr, UNIT *uptr, int32 switches)
{
    // BUG: sanity check args
    // NOTE: We ignore UNIT, because all CPUS see the same memory

    // log_msg(INFO_MSG, "CPU:examine", "Examine address is %012llo; uptr is %p\n", simh_addr, uptr);

    addr_modes_t mode;
    unsigned segno;
    unsigned offset;
    if (addr_simh_to_emul(simh_addr, &mode, &segno, &offset) != 0) {
        log_msg(NOTIFY_MSG, "CPU:examine", "Cannot convert %012llo in unit %p to an address\n", simh_addr, uptr);
        return SCPE_ARG;    // BUG: is this the right failure code
    }
    unsigned abs_addr;
    if (addr_any_to_abs(&abs_addr, mode, segno, offset) != 0) {
        if (mode == APPEND_mode)
            log_msg(NOTIFY_MSG, "CPU:examine", "Cannot convert %012llo=>{%03o|%06o} in unit %p to an address\n", simh_addr, segno, offset, uptr);
        log_msg(NOTIFY_MSG, "CPU:examine", "Cannot convert %012llo=>{mode=%d,seg=%#o,off=%#o} in unit %p to an address\n", simh_addr, mode, segno, offset, uptr);
        return SCPE_ARG;    // BUG: is this the right failure code
    }

    // TODO: handle MEM_CHECK_UNINIT
    memcpy(eval_array, &Mem[abs_addr], sizeof(Mem[0])); // SIMH absolute reference
    return 0;
}

//=============================================================================

/*
 * cpu_dep()
 *
 * Called by SIMH to write to memory.
 *
 */

static t_stat cpu_dep (t_value v, t_addr simh_addr, UNIT *uptr, int32 switches)
{
    // BUG: sanity check args
    // NOTE: We ignore UNIT, because all CPUS see the same memory

    addr_modes_t mode;
    unsigned segno;
    unsigned offset;
    if (addr_simh_to_emul(simh_addr, &mode, &segno, &offset) != 0)
        return SCPE_ARG;    // BUG: is this the right failure code
    unsigned abs_addr;
    if (addr_any_to_abs(&abs_addr, mode, segno, offset) != 0)
        return SCPE_ARG;    // BUG: is this the right failure code

    // We use the translated absolute reference here.
    return store_abs_word(abs_addr, v);
}

//=============================================================================

/*
 * init_opcodes()
 *
 * This initializes the is_eis[] array which we use to detect whether or
 * not an instruction is an EIS instruction. 
 *
 * TODO: Change the array values to show how many operand words are
 * used.  This would allow for better symbolic disassembly.
 *
 * BUG: unimplemented instructions may not be represented
 */

static void init_opcodes()
{
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
    is_eis[(opcode1_dtb<<1)|1] = 1;
    is_eis[(opcode1_dv3d<<1)|1] = 1;
}

// ============================================================================

static int write72(FILE* fp, t_uint64 word0, t_uint64 word1)
{
    for (int i = 0; i < 4; ++i) {
        unsigned char c;
        c = (word0 >> (36 - 8 - i * 8)) & 0xff;
        if (fputc(c, fp) == EOF)
            return 1;
    }
    word1 |= (word0 & 0xf) << 36;
    for (int i = 0; i < 5; ++i) {
        unsigned char c;
        c = (word1 >> ((4-i) * 8)) & 0xff;
        if (fputc(c, fp) == EOF)
            return 1;
    }
    return 0;
}

//=============================================================================

static int read72(FILE* fp, t_uint64* word0p, t_uint64* word1p)
{
    unsigned char buf[9];
    memset(buf, 0, sizeof(buf));
    unsigned nread = fread(buf, 1, sizeof(buf), fp);
    if (nread == 0)
        return 0;   // caller should test for feof() and ferror()
    *word0p = 0;
    for (int i = 0; i < 4; ++i) {
        *word0p <<= 8;
        *word0p |= buf[i];
    }
    *word0p <<= 4;
    *word0p |= buf[4] >> 4;
    if (nread < 5)
        return 1;   // read less than 36 bits, don't stomp 2nd word
    *word1p = buf[4] & 0xf;
    for (int i = 5; i < 9; ++i) {
        *word1p <<= 8;
        *word1p |= buf[i];
    }
    return (nread == 9) ? 0 : 1;
}

//=============================================================================
static int cpu_show_fault_base(FILE *st, UNIT *uptr, int32 val, void *desc)
{
    // FIXME: use FILE *st
    // FIXME: use uptr to find one of multiple CPUs

    out_msg("FAULT_BASE: %sb aka %03o => %04o",
        bin2text(switches.FLT_BASE, 7), switches.FLT_BASE,
        switches.FLT_BASE << 5);
    return 0;
}

//=============================================================================

static int cpu_set_fault_base(UNIT *uptr, int32 val, char *cptr, void *desc)
{
    const char* sw_name = "FLT_BASE";
    if (cptr == NULL) {
        out_msg("Error, usage is set cpu %s=<value>\n", sw_name);
        return SCPE_ARG;
    }
    char c;
    int n;
    if (sscanf(cptr, "%d %c", &n, &c) != 1) {
        out_msg("Error, expecting a number.\n");
        return SCPE_ARG;
    }

    if (n < 0) {
        out_msg("Error, expecting a value between 0 and %#o.\n", 1<<7);
        return SCPE_ARG;
    }
    if (n >= (1<<7)) {
        out_msg("Error, expecting a value between 0 and %#o.\n", 1<<7);
        return SCPE_ARG;
    }
    switches.FLT_BASE = n;

    return 0;
}

//=============================================================================
