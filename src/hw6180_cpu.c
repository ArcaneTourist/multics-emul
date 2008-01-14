#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "hw6180.h"
#include "bits.h"


//-----------------------------------------------------------------------------
//*** SIMH specific externs

#define MAXMEMSIZE (16*1024*1024)
t_uint64 M[MAXMEMSIZE]; /* memory */
extern int32 sim_interval;
extern uint32 sim_brk_summ;

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
    


static t_uint64 A;  // Accumulator, 36 bits
static t_uint64 Q;  // Quotient, 36 bits
// Note: AQ register is just a combination of the A and Q registers
static t_uint64 E;  // Exponent
// Note: EAQ register is just a combination of the E, A, and Q registers
static t_uint64 X[8];   // Index Registers
static IR_t IR;     // indicator register
static t_uint64 saved_IR;
// static int32 IC; // APU (appending unit) Instruction Counter, 18 bits -- see PPR.IC !!!
static t_uint64 saved_IC;
static t_uint64 BAR;    // Base Addr Register; 18 bits
static t_uint64 TR;     // Timer Reg, 27 bits
static t_uint64 RALR;   // Ring Alarm Reg, 3 bits
static t_uint64 PR[8];  // Pointer Registers, 42 bits
static t_uint64 AR[8];  // Address Registers, 24 bits
static PPR_t PPR;       // Procedure Pointer Reg, 37 bits, internal only
static TPR_t TPR;       // Temporary Pointer Reg, 42 bits, internal only
// DSBR;    // Descriptor Segment Base Register, 51 bits
// SDWAM[16];   // Segment Descriptor Word Associative Memory, 88 bits
// PTWAM[16];   // Page Table Word Associative Memory, 51 bits
static fault_reg_t FR;  // Fault Register, 35 bits
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
    { ORDATA (IR, saved_IR, 14) },
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

static UNIT cpu_unit = {
    // TODO: idle svc
    UDATA (NULL, UNIT_FIX|UNIT_BINK, MAXMEMSIZE)
};

static MTAB cpu_mod[] = {
    // for SIMH "show" and "set" commands; Todo: fill in MTAB
    { 0 }
};

t_stat cpu_boot (int32 unit_num, DEVICE *dptr);;
t_stat cpu_reset (DEVICE *dptr);

DEVICE cpu_dev = {
    // todo: revisit cpu_dev: add debug, examine, deposit, etc
    "CPU", &cpu_unit, cpu_reg, cpu_mod,
    1, 8, 18, 1, 8, 8,
    NULL, NULL, &cpu_reset,
    &cpu_boot, NULL, NULL
};

//-----------------------------------------------------------------------------
//*** Other Globals holding state values
//      If SIMH uses known devices to switch between CPUs, we'll have to 
//      add these to cpu_reg as read only (perhaps even hidden) registers.
//      In order to suppport SIMH's save/restore commands, we'll at least
//      have to register a dummy register and examine/deposit routine.

static cycles_t cycle;
static events_t events;
static switches_t switches;
// the following two should probably be combined
cpu_state_t cpu;
ctl_unit_data_t cu; 

//-----------------------------------------------------------------------------
//***  Other Externs
extern int bootimage_loaded;    // only relevent for the boot CPU ?


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

//-----------------------------------------------------------------------------
//***  Function prototypes

t_stat control_unit(void);
int fetch_instr(uint IC, instr_t *ip);
int fetch_word(uint IC, t_uint64 *wordp);
void execute_ir(void);
void fault_gen(enum faults f);
void set_addr_mode(addr_modes_t mode);
addr_modes_t get_addr_mode(void);
void decode_instr(instr_t *ip, t_uint64 word);

void tape_block(unsigned char *p, uint32 len);


//=============================================================================

t_stat cpu_boot (int32 unit_num, DEVICE *dptr)
{
    // Boot -- Copy bootstrap loader into memory & set PC (actually send startup fault)
    // Issue: have to specify boot tape or file out-of-band

    // quick & dirty

    // from the web site:
    //      tape image format:
    //      <32 bit little-endian blksiz> <data> <32bit little-endian blksiz>
    //      a single 32 bit word of zero represents a file mark
    // question
    //      What type of data does blksiz measure?  Count of 32bit words?  Count of 9 bit "bytes" ?
    
    char *fname = "boot.tape";
    printf("Loading file %s\n", fname);
    int fd;
    if ((fd = open(fname, O_RDONLY)) == -1) {
        perror(fname);  // BUG
        return STOP_BUG;    // todo: better code
    }
    uint addr = 0;
    uint nblocks = 0;
    int read_unit = 8;  // 8, 9, 18, 32, 36, 288, 9216, etc
    int invert = 0;
    uint totread = 0;
    for(;;) {
        ssize_t nread;
        unsigned char word[4];
        uint32 n;
        uint32 nalt;
        // stats
        // printf ("%u bytes read (incl %d 32bit counts)\n", totread, nblocks);
        // read 32bit count
        int i;
        if (1) {
            if ((nread = read(fd, word, 4)) != 4) {
                if (nread == 0) {
                    printf("EOF at index %d\n", i);
                    close(fd);
                    return STOP_BUG;    // BUG: bogus, but prevents "go"
                    return 0;
                } else {
                    perror("begin count read"); // BUG
                    close(fd);
                    return STOP_BUG;
                }
            }
            n = (word[0]) | (word[1] << 8) | (word[2] << 16) | (word[3] << 24);
            nalt = (word[3]) | (word[2] << 8) | (word[1] << 16) | (word[0] << 24);
        } else {
            if ((nread = read(fd, &n, 4)) != 4) {
                perror("count read");   // BUG
                close(fd);
                return STOP_BUG;
            }
        }
        ++ nblocks;
        totread += 4;
        if (n == 0) {
            printf("record mark found\n");
            continue;
        }
        // convert count
#if 0
        if (read_unit == 36) {
            if ((n * 36) % 32 == 0) {
                printf("Block size %d is a multiple of 36/32\n", n);
                n = n * 36 / 32;
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 36/8\n", (unsigned long) n, (unsigned long) n);
                close(fd);
                return STOP_BUG;
            }
        }
#endif
        if (invert) {
            if ((n * 8) % read_unit  == 0) {
                uint32 nold = n;
                n = n * 8 / read_unit;
                printf("Block size %u is a multiple of 8/%d ==> %u\n", nold, read_unit, n);
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 8/%d\n", (unsigned long) n, (unsigned long) n, read_unit);
                close(fd);
                return STOP_BUG;
            }
        } else {
            if ((n * read_unit) % 8  == 0) {
                uint32 nold = n;
                n = n * read_unit / 8;
                // printf("Block size %u is a multiple of %d/8 ==> %u\n", nold, read_unit, n);
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 8\n", (unsigned long) n, (unsigned long) n);
                close(fd);
                return STOP_BUG;
            }
        }
        // read block
        //printf("Start read block of %lu bytes for %d-bit units\n", (unsigned long) n, read_unit);
        if(1) {
            unsigned char *bufp = malloc(n);
            if (bufp == NULL) {
                perror("malloc");
                close(fd);
                return STOP_BUG;
            }
            if ((nread = read(fd, bufp, n)) != n) {
                if (nread < 0) {
                    perror("block read");
                } else if (nread == 0) {
                    printf("Unexpected EOF\n");
                } else {
                    printf("Short read of %u bytes (expecting %u)\n", nread, n);
                }
                close(fd);
                return STOP_BUG;
            }
            // use bytes
            tape_block(bufp, n);
            bootimage_loaded = 1;
            // Only read one record
            return STOP_BUG;
        } else {
        for (i = 0; i < n; ++i) {
            ++addr;
            unsigned char byte;
            uint32 word;
            int err;
            err = (nread = read(fd, &byte, 1)) != 1;
            if (!err) ++ totread;
            if (err) {
                if (nread == 0) {
                    printf("EOF at index %d\n", i);
                } else {
                    perror("byte read");    // BUG
                    printf("error at byte %d; return was %d\n", i, nread);
                }
                close(fd);
                return STOP_BUG;
            }
        }
        }
        if ((nread = read(fd, word, 4)) != 4) {
            perror("end-of-block count read");  // BUG
            close(fd);
            return STOP_BUG;
        }
        uint32 n2 = (word[0]) | (word[1] << 8) | (word[2] << 16) | (word[3] << 24);
        if (n == n2) {
            // printf("End block count %d matches begin block count\n", n2);
        } else {
            printf("End block count %d does not matche begin block count %u.\n", n2, n);
            close(fd);
            return STOP_BUG;
        }
    }
abort();
    return 0;
}


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

    bootimage_loaded = 0;
    memset(&events, 0, sizeof(events));
    memset(&switches, 0, sizeof(switches));
    memset(&cpu, 0, sizeof(cpu));
    memset(&cu, 0, sizeof(cu));
    memset(&PPR, 0, sizeof(PPR));
    cpu.ic_odd = 0;

    // BUG: reset *all* other structures to zero

    // multics uses same vector for interrupts & faults?
    // OTOH, AN87, 1-41 claims faults are at 100o ((flt_base=1)<<5)
    switches.FLT_BASE = 0;  // multics uses same vector for interrupts & faults?

    cycle = FETCH_cycle;
    set_addr_mode(ABSOLUTE_mode);
    fault_gen(startup_fault);   // pressing POWER ON button causes this fault

    return 0;
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
    
    if (! bootimage_loaded) {
        // We probably should not do this
        complain_msg("CPU::CU", "Memory is empty, no bootimage loaded yet\n");
        // return STOP_MEMCLEAR;
    }

    int reason = 0;
    while (reason == 0) {   /* loop until halted */
        if (sim_interval <= 0) { /* check clock queue */
            // process any SIMH timed events including keyboard halt
            if ((reason = sim_process_event()) != 0) break;
        }
        reason = control_unit();
        sim_interval--; // todo: maybe only per instr or by brkpoint type?
    }

    // BUG: pack private variables into SIMH's world
    saved_IC = PPR.IC;
    return reason;
}


//=============================================================================


t_stat control_unit(void)
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

    switch(cycle) {
        case FETCH_cycle:
            debug_msg("CPU::CU", "Cycle = FETCH; IC = %u\n", PPR.IC);
            // If execution of the current pair is complete, the processor
            // checks two? internal flags for group 7 faults and/or interrupts.
            if (events.any) {
                if (events.low_group != 0) {
                    // BUG: don't need test below now that we detect 1-6 here
                    cycle = FAULT_cycle;
                    break;
                }
                if (events.group7 != 0) {
                    // Group 7 -- See tally runout in IR, connect fields of the
                    // fault register.  DC power off must come via an interrupt?
                    cycle = FAULT_cycle;
                    break;
                }
                if (events.int_pending) {
                    cycle = INTERRUPT_cycle;
                    break;
                }
            }
            // fetch a pair of words
            if (PPR.IC % 2 != 0) {
                reason = STOP_ODD_FETCH;
            } else {
                cpu.ic_odd = 0; // execute even instr of current pair
                // todo:  Is cpu.ic_odd unnecessary?  Can we just look at
                // the IC to determine which to execute?
                if (fetch_instr(PPR.IC, &cu.IR) != 0 || fetch_word(PPR.IC + 1, &cu.IRODD) != 0)
                    cycle = FAULT_cycle;
                else
                    cycle = EXEC_cycle;
            }
            break;

#if 0
    we don't use an ABORT cycle
        case ABORT_cycle:
            debug_msg("CPU::CU", "Cycle = ABORT\n");
            // Invoked when control unit decides to handle fault
            // Bring all overlapped functions to an orderly halt -- however,
            // the simulator has no overlapped functions?
            // Also bring asynchronous functions within the processor
            // to an orderly halt -- todo -- do we have any?
            cycle = FAULT_cycle;
            break;
#endif

        case FAULT_cycle:
            {
            debug_msg("CPU::CU", "Cycle = FAULT\n");
            addr_modes_t saved_addr_mode = get_addr_mode();
            set_addr_mode(ABSOLUTE_mode); // until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3

            // find highest fault
            int fault = 0;
            int i;
            for (i = 0; i <= 6; ++ i) {
                if ((fault = events.fault[i]) != 0)
                    break;
            }
            if (fault == 0) {
                if (events.group7 == 0) {
                    // bogus fault
                    complain_msg("CPU::CU", "Fault cycle with no faults set\n");
                    reason = STOP_BUG;
                    break;
                } else {
                    // find highest priority group 7 fault
                    int hi = -1;
                    for (i = 0; i < 31; ++i) {
                        if (fault2group[i] == 7 && (events.group7 & (1<<fault)))
                            if (hi == -1 || fault2prio[i] < fault2prio[hi])
                                hi = i;
                    }
                    if (hi == -1) {
                        // no group 7 fault
                        complain_msg("CPU::CU", "Fault cycle with missing group-7 fault\n");
                        reason = STOP_BUG;
                        break;
                    } else
                        fault = hi;
                }
            }
            debug_msg("CPU::CU", "fault = %d\n", fault);
            if (fault != trouble_fault) {
                // TODO: Safe store control unit data into invisible registers
                // in prep for a store control unit (scu) instr
            }

            // BUG: clear fault?  Or does scr instr do that?

            PPR.PRR = 0;    // set ring zero
            uint addr = switches.FLT_BASE + 2 * fault; // ABSOLUTE mode
            // Force computed addr and xed opcode into the instruction
            // register and execute (during FAULT CYCLE not EXECUTE CYCLE).
            cu.IR.offset = addr;
            cu.IR.opcode = (opcode0_xed << 1);
            cu.IR.inhibit = 1;
            cu.IR.pr_bit = 0;
            cu.IR.tag = 0;

            // Maybe just set a flag and run the EXEC case?
            // Maybe the following increments and tests are handled by EXEC and/or the XED opcode?

            // todo: Check for SIMH breakpoint on execution for that addr or
            // maybe in the code for the xed opcode.
            debug_msg("CPU::CU", "calling execute_ir() for xed\n");
            execute_ir();   // executing in FAULT CYCLE, not EXECUTE CYCLE
            if (events.any && events.fault[fault2group[trouble_fault]] == trouble_fault) {
                // Fault occured during execution, so fault_gen() flagged
                // a trouble fault
                // Stay in FAULT CYCLE
                debug_msg("CPU::CU", "re-faulted, remaining in fault cycle\n");
            } else {
                if (!cpu.xfr) {
                    debug_msg("CPU::CU", "no re-fault, no transfer -- incrementing IC\n");
                    // BUG: Faulted instr doesn't get re-invoked?
                            sim_interval = 0;   // force return
                    ++ PPR.IC;
                    if (cpu.ic_odd) {
                        if (PPR.IC % 2 != 0) { complain_msg("CPU::CU", "Fault on odd half of instr pair results in next instr being at an odd IC\n"); }
                        cycle = FETCH_cycle;
                    } else {
                        if (PPR.IC % 2 != 1) { complain_msg("CPU::CU", "Fault on even half of instr pair results in odd half pending with an even IC\n"); }
                        cycle = EXEC_cycle;
                    }
                    // Note that we don't automatically return to the original
                    // ring, but hopefully we're executed an rcu instruction
                    // to set the correct ring
                    set_addr_mode(saved_addr_mode);
                } else {
                    debug_msg("CPU::CU", "no re-fault, but a transfer done -- not incrementing IC\n");
                }
            }
            } // end case FAULT_cycle
            break;

        case INTERRUPT_cycle:
            set_addr_mode(ABSOLUTE_mode); // until execution of a transfer instr whose operand is obtained via explicit use of the appending HW mechanism -- AL39, 1-3
            abort();
            break;

        case EXEC_cycle:
            // todo: Assumption: IC will be at curr instr, even
            // when we're ready to execute the odd half of the pair
            // todo: check for IC matching curr instr or at least even/odd sanity
            if (! cpu.ic_odd) {
                debug_msg("CPU::CU", "Cycle = EXEC, even instr\n");
                if (sim_brk_summ) {
                    if (sim_brk_test (PPR.IC, SWMASK ('E'))) {
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                (void) execute_ir();
#if 0
                // delay any fault handling for faults by even instr
                if (cycle == EXEC_cycle && !cpu.xfr) {
                    cpu.ic_odd = 1; // execute odd instr of current pair
                    ++ PPR.IC;
                }
#else
                // Check for fault from even instr
                // todo: simplify --- cycle won't be EXEC anymore
                if (events.any && events.low_group && events.low_group < 7) {
                    // faulted
                    debug_msg("CPU::CU", "Probable fault after EXEC even instr\n");
                } else {
                    if (!cpu.xfr) {
                        cpu.ic_odd = 1; // execute odd instr of current pair
                        ++ PPR.IC;
                    }
                }
#endif
            } else {
                debug_msg("CPU::CU", "Cycle = EXEC, odd instr\n");
                // We assume IC was advanced after even instr
                decode_instr(&cu.IR, cu.IRODD);
                if (sim_brk_summ) {
                    if (sim_brk_test (PPR.IC, SWMASK ('E'))) {
                        reason = STOP_IBKPT;    /* stop simulation */
                        break;
                    }
                }
                execute_ir();
                // todo: simplify --- cycle won't be EXEC anymore
                if (events.any && events.low_group && events.low_group < 7) {
                    // faulted
                    debug_msg("CPU::CU", "Probable fault after EXEC odd instr\n");
                } else {
                    if (!cpu.xfr) {
                        cpu.ic_odd = 0; // finished with odd half
                        ++ PPR.IC;
                    }
                    cycle = FETCH_cycle;
                }
            }
            break;
        default:
            complain_msg("CPU::CU", "Unknown cycle # %d\n", cycle);
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
}

//=============================================================================

void fault_gen(enum faults f)
{
    int group;
    events.any = 1;

    if (f == oob_fault) {
        complain_msg("CPU::fault", "Faulting for internal bug\n");
        f = trouble_fault;
        sim_interval = 0;   // insufficient
    }

    if (f < 1 || f > 32) {
        complain_msg("CPU::fault", "Bad fault # %d\n", f);
        abort();
    }
    group = fault2group[f];
    debug_msg("CPU::fault", "Recording fault # %d in group %d\n", f, group);
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
            debug_msg("CPU::fault", "Double fault:  Recording current fault as a trouble fault (fault # %d in group %d).\n", f, group);
        } else {
            if (events.fault[group]) {
                // todo: error, unhandled fault
                debug_msg("CPU::fault", "Found unhandled prior fault #%d in group %d.\n", events.fault[group], group);
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

void set_addr_mode(addr_modes_t mode)
{
    if (mode == ABSOLUTE_mode) {
        IR.abs_mode = 1;
    } else if (mode == APPEND_mode) {
        abort();
    } else if (mode == BAR_mode) {
        abort();
    } else {
        complain_msg("CPU", "Unable to determine address mode.\n");
        abort();
    }
}

addr_modes_t get_addr_mode()
{
    // this function should be replaced
    if (IR.abs_mode)
        return ABSOLUTE_mode;
    complain_msg("CPU", "Unknown address mode\n");
    abort();
}

//=============================================================================

int fetch_instr(uint IC, instr_t *ip)
{
    // returns non-zero if fault detected

    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        abort();
    }

    // todo: check for read breakpoints

    decode_instr(ip, M[IC]);
    return 0;
}


int fetch_word(uint IC, t_uint64 *wordp)
{
    // returns non-zero if fault in groups 1-6 detected
    // WAS returns non-zero if fault detected

    if (get_addr_mode() != ABSOLUTE_mode) {
        // BUG: IC needs conversion to abs mem addr
        abort();
    }

    // todo: check for read breakpoints

    *wordp = M[IC];
    return 0;
}

int fetch_pair(uint IC, t_uint64* word0p, t_uint64* word1p)
{
    // Fetch even and odd words at Y-pair given by IC
    // returns non-zero if fault in groups 1-6 detected

    int ret;
    uint Y = (IC % 2 == 0) ? IC : IC - 1;

    if ((ret = fetch_word(Y, word0p) != 0)) {
        return ret;
    }
    if ((ret = fetch_word(Y+1, word1p) != 0)) {
        return ret;
    }
    return 0;
}

void decode_instr(instr_t *ip, t_uint64 word)
{
    ip->offset = getbits36(word, 0, 18);
    ip->opcode = getbits36(word, 18, 10);
    ip->inhibit = getbits36(word, 28, 1);
    ip->pr_bit = getbits36(word, 29, 1);
    ip->tag = getbits36(word, 30, 6);
}

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


//=============================================================================

void anal36 (const char* tag, t_uint64 word);
char *bin(t_uint64 word, int n);
#include <ctype.h>

void tape_block(unsigned char *p, uint32 len)
{
    static size_t hack = 0;
    bitstream_t *bp = bitstm_new(p, len);
    if ((len * 8) % 36 != 0) {
        complain_msg("CPU::boot", "Length %u bytes is not a multiple of 36 bits.\n");
    }
    printf("Tape block: %u bytes, %u 36-bit words\n", len, len*8/36);
    uint32 nbits = len * 8;
    while (nbits >= 36) {
        bitstm_get(bp, 36, &M[hack++]);
        nbits -= 36;
    }
    if (nbits != 0) {
        complain_msg("CPU::boot", "Internal error getting bits from tape\n");
    }
#if 0
    t_uint64 word;
    word = 0670314355245;   // magic
    anal36("magic", word);
    printf("\n");
    uint32 nbits = len * 8;
    int j;
    for (j = 0; nbits >= 36 && j < 256; ++ j) {
        bitstm_get(bp, 36, &word);
        //bitstm_get(bp, 9, &word);
        //printf("9bit: %030lu oct, %lu decimal\n", word, word);
        nbits -= 36;
        anal36("WORD", word);
        if ((word >> 36) != 0) {
            complain_msg("CPU::boot", "bitstm_get() is broken\n");
        }
        t_uint64 rev = 0;
        if (0) {
            // test reveals string MULTICS w/o reversing
            int i;
            for (i = 0; i < 36; ++i) {
                rev <<= 1;
                if (word % 2 == 1)
                    rev |= 1;
                word >>= 1;
            }
            anal36("REV", rev); printf("\n");
        }
    }

    complain_msg("CPU::boot", "bye!\n"); exit(1);
#endif
}

void anal36 (const char* tag, t_uint64 word)
{
    unsigned char nines[4];
    nines[3] = word & 0777;
    nines[2] = (word >> 9) & 0777;
    nines[1] = (word >> 18) & 0777;
    nines[0] = (word >> 27) & 0777;
    printf("%s: %012Lo octal, %Lu decimal\n", tag, word, word);
    printf("bin64: %s\n", bin(word, 64));
    printf("bin36: %s\n", bin(word, 36));
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

char *bin(t_uint64 word, int n)
{
    static char str[65];
    str[n] = 0;
    int i;
    for (i = 0; i < n; ++ i) {
        str[n-i-1] = ((word % 2) == 1) ? '1' : '0';
        word >>= 1;
    }
    return str;
}
