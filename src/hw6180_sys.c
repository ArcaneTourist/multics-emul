/*
    hw6180_sys.c -- Most of the interfaces to SIMH.
*/

#include "hw6180.h"
#include <ctype.h>

#define MEM_CHECK_UNINIT 1

// The following are assigned to SIMH function pointers
static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr);
static void fprint_addr(FILE *stream, DEVICE *dptr, t_addr addr);
static void hw6180_init(void);

extern DEVICE cpu_dev;
extern DEVICE tape_dev;
extern DEVICE opcon_dev;
extern DEVICE iom_dev;
extern DEVICE disk_dev;
extern REG cpu_reg[];

extern UNIT TR_clk_unit;
extern switches_t switches;
extern cpu_ports_t cpu_ports;
extern scu_t scu;
extern iom_t iom;

//=============================================================================

/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
   sim_load             binary loader/dumper

*/

char sim_name[] = "hw6180";

REG *sim_PC = &cpu_reg[0];

int32 sim_emax = 4;

/* Multics allows up to 7 CPUs... */

DEVICE *sim_devices[] = {
    &cpu_dev,
    // &cpu_dev2,
    &tape_dev,
    &opcon_dev,
    &iom_dev,
    &disk_dev,
    NULL
};

const char *sim_stop_messages[] = {
    "<error zero>",
    "Memory is empty",
    "BUG-STOP -- Internal error, further execution probably pointless",
    "WARN-STOP -- Internal error, further processing might be ok",
    "Breakpoint",
    "DIS -- A 'Delay Until Interrupt Set' instruction has been executed",
    "SIMH requested stop",
    // "Invalid Opcode"
    0
};

extern CTAB *sim_vm_cmd;
static struct sim_ctab sim_cmds[] =  {
    { "XDEBUG",   cmd_xdebug, 0,       "xdebug seg <#> {on|default_off}  finer grained debugging\n" },
    { "XFIND",    cmd_find, 0,         "xfind <string> <range>           search memory for string\n" },
    { "XHISTORY", cmd_dump_history, 0, "xhistory                         display recent instruction counter values\n" },
    { "XLIST",    cmd_load_listing, 0, "xlist <addr> <source>            load pl1 listing\n" },
    { "XSEGINFO", cmd_seginfo, 0,      "xseginfo <seg>                   walk segment linkage table\n" },
    { "XSTACK",   cmd_stack_trace, 0,  "xstack                           dump Multics procedure call stack\n" },
    { "XSTATS",   cmd_stats, 0,  "xstats                           display statistics\n" },
    { "XSYMTAB",  cmd_symtab_parse, 0, "xsymtab [...]                    define symtab entries\n" },
    { "XVMDUMP",  cmd_dump_vm, 0,      "xvmdump                          dump virtual memory caches\n" },
    { 0, 0, 0, 0}
};

// One-time-only initialization for emulator
void (*sim_vm_init)(void) = hw6180_init;

//=============================================================================

/*
    Variables custom to HW6180 that aren't normally present in other
    SIMH emulators.
*/

int bootimage_loaded = 0;
sysinfo_t sys_opts;

//-----------------------------------------------------------------------------

static void init_memory_iom(void);

//=============================================================================

/*
 * hw6180_init()
 *
 * Once-only simulator initialization.  Called by SIMH via the sim_vm_init
 * pointer.
 *
 * This function mostly sets the values of simulated physical switches
 * and sets up structures representing how the various devices are
 * physically cabled together.
 *
 * TODO: Provide a more general function interface for specifying
 * that two devices are interconnected.
 *
 * TODO: Move some of this code into device reset routines.
 */

static void hw6180_init(void)
{
    log_msg(INFO_MSG, "SYS::init", "Once-only initialization running.\n");

    fault_gen_no_fault = 0;
    sim_vm_parse_addr = parse_addr;
    sim_vm_fprint_addr = fprint_addr;
    sim_vm_cmd = sim_cmds;

    mt_init();
    console_init();
    iom_init();

    // todo: set debug flags for all devices
    // cpu_dev.dctrl = 1;
    // tape_dev.dctrl = 1;

    sim_brk_types = SWMASK('E');    // execution
    sim_brk_types |= SWMASK('M');   // memory read/write
    sim_brk_types |= SWMASK('W');   // memory write
    sim_brk_dflt = SWMASK('E');

    // System-wide options
    memset(&sys_opts, 0, sizeof(sys_opts));
    sys_opts.clock_speed = 250000; // about 1/4 of a MIP
    // Negative times imply instantaneous operation without making
    // use of sim_activate().  Zero times have almost the same
    // result, except that the caller queues an immediate run via
    // sim_activate() and then returns.  The zero wait event(s) will
    // be noticed and handled prior to the next instruction execution.
    sys_opts.iom_times.connect = 0; // 3    // 10 seconds is too long...
    sys_opts.iom_times.chan_activate = -1;  // unimplemented
    sys_opts.mt_times.read = -1;    // 100; // 1000;
    sys_opts.mt_times.xfer = -1;            // unimplemented
    sys_opts.warn_uninit = 1;

    // Hardware config -- todo - should be based on config cards!
    // BUG/TODO: need to write config deck at 012000 ? Probably not

    // Only one CPU
    memset(&cpu_ports, 0, sizeof(cpu_ports));
    for (int i = 0; i < ARRAY_SIZE(cpu_ports.ports); ++i)
        cpu_ports.ports[i] = -1;

    // CPU Switches
    memset(&switches, 0, sizeof(switches));
    switches.cpu_num = 0;   // CPU 'A' is bootload cpu (init_early_config.pl1)
    // FLT_BASE switches are the 7 MSB of a 12bit addr
    // AN87, 1-41 claims multics requires faults to be at 100o
    switches.FLT_BASE = 2;  // 2<<5 == 0100
    // At one time, it seemed that the diag tape required using different
    // fault base switch settings.  However, the diag tape may just choose
    // different addresses for crashing depending on the setting of FLT_BASE...
    // switches.FLT_BASE = 0163; // diag tape allows any loc *except* 2->0100
    // switches.FLT_BASE = 0; // diag tape allows any location *except* 2->0100
//switches.FLT_BASE = 0;

    // Only one SCU
    memset(&scu, 0, sizeof(scu));
    scu.mode = 1;   // PROGRAM mode
    for (int i = 0; i < ARRAY_SIZE(scu.ports); ++i)
        scu.ports[i].idnum = -1;

    // BUG/TODO: the following belongs in a scu_reset()
    for (int i = 0; i < ARRAY_SIZE(scu.ports); ++i)
        scu.ports[i].is_enabled = 0;
    for (int i = 0; i < ARRAY_SIZE(scu.interrupts); ++i) {
        scu.interrupts[i].mask_assign.unassigned = 1;
        scu.interrupts[i].exec_intr_mask = ~0 & MASKBITS(32);
    }

    // Only two of the four SCU masks are used; these correspond to the "A"
    // and "B" rotary switches
    scu.interrupts[0].avail = 1;
    scu.interrupts[1].avail = 1;

    // Only one IOM
    iom.iom_num = 0;    // IOM "A"

    Mem = malloc(sizeof(*Mem) * MAXMEMSIZE);
    if (Mem == NULL) {
        log_msg(ERR_MSG, "SYS::init", "Cannot allocate memory.\n");
        return;
    }
#if MEM_CHECK_UNINIT
    memset(Mem, 0xff, MAXMEMSIZE*sizeof(Mem[0]));
#else
    memset(Mem, 0, MAXMEMSIZE*sizeof(Mem[0]));
#endif

    // TODO: init_memory_iom() should probably be called by boot()

    // Initializing memory to reflect the existance of an IOM, not an
    // IOX.  Using an IOX causes use of the non L68 "ldo" instruction.
    // The "ldo" instruction was implmented on on the ADP aka ORION aka DPS88.
    // Also, the IOX has an undocumented mailbox architecture.

    //init_memory_iox();
    init_memory_iom();

    // CPU port 'd' (1) connected to port '0' of SCU
    // scas_init's call to make_card seems to require that the CPU be connected
    // to SCU port zero.
    // Also, rsw_util$port_info claims base addr is: port-assignment * size/1024
    int cpu_port = 4;       // CPU port 'd' or 4
    // int cpu_port = 1;        // CPU port 'b' or 1
    cpu_ports.scu_port = 0;
    cpu_ports.ports[cpu_port] = 0;  // CPU connected to SCU "A"
    scu.ports[cpu_ports.scu_port].is_enabled = 1;
    scu.ports[cpu_ports.scu_port].type = ADEV_CPU;
    scu.ports[cpu_ports.scu_port].idnum = switches.cpu_num;
    scu.ports[cpu_ports.scu_port].dev_port = cpu_port;

    // The following should probably be in scu_rest() because mask
    // assignments can be changed by running code
    // GB61, pages 9-1 and A-2: Set Mask A to port that the bootload CPU is
    // connected to; Set Mask B to off
    scu.interrupts[0].mask_assign.unassigned = 0;
    scu.interrupts[0].mask_assign.port = cpu_ports.scu_port;
    scu.interrupts[0].mask_assign.raw = 1 << (8 - cpu_ports.scu_port);

    // IOM port 3 connected to port 1 of bootload SCU
    // Some sources say that we must use the same port (a-h) on the IOM as
    // we used on the CPU.  However, scas_init.pl1 will complain about being
    // unable to setup cyclic port priority if IOMS and CPUS use the same
    // SCU ports.
    int iom_port = 3;   // BUG
    // int iom_port = cpu_port; // required by AM81 and AN70
    iom.scu_port = 1;
    iom.ports[iom_port] = 0;    // port C connected to SCU "A"
    scu.ports[iom.scu_port].is_enabled = 1;
    scu.ports[iom.scu_port].type = ADEV_IOM;
    scu.ports[iom.scu_port].idnum = iom.iom_num;
    scu.ports[iom.scu_port].dev_port = iom_port;

    /* Console */
    int con_chan = 012; // channels 010 and higher are probed for an operators console
    iom.channels[con_chan].type = DEV_CON;
    iom.channels[con_chan].dev = &opcon_dev;

    /* Disk */
    const int disk_chan = 20;
    iom.channels[disk_chan].type = DEV_DISK;
    iom.channels[disk_chan].dev = &disk_dev;

    log_msg(DEBUG_MSG, "SYS::init", "Once-only initialization complete.\n");
    log_msg(DEBUG_MSG, "SYS::init", "Activity queue has %d entries.\n", sim_qcount());
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

static void init_memory_iom()
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

    // Which controller channel is used for the tape drive would seem to be
    // arbitrary.  However, the T&D tape actually executes an IMW word.
    // Perhaps this is due to a bug elsewhere.  If not, our previous
    // channel choice of 036 caused the instruction word to have an illegal
    // tag.  The IMW at 00214 is 006715/075000 (lda 06715).  GB61 has an
    // example of using channel 14; we'll use that which just happens to
    // leave the 006715 value unchanged
    //
    // Channels range from 1 to 037; the first several channels are
    // reserved.
    int tape_chan = 14;     // 12 bits or 6 bits; controller channel
    // "SCU port" # (deduced as meaning "to which bootload IOM is attached")
    int port = iom.scu_port;    // 3 bits; 

    iom.channels[tape_chan].type = DEV_TAPE;
    iom.channels[tape_chan].dev = &tape_dev;

    int base = 014;         // 12 bits; IOM base
    // bootload_io.alm insists that pi_base match
    // template_slt_$iom_mailbox_absloc
    int pi_base = 01200;    // 15 bits; interrupt cells
    int iom = 0;            // 3 bits; only IOM 0 would use vector 030

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

    t_uint64 dis0 = 0616200;
    /* 1*/ Mem[010 + 2 * iom] = (imu << 34) | dis0;         // system fault vector; DIS 0 instruction (imu bit not mentioned by 43A239854)
    // Zero other 1/2 of y-pair to avoid msgs re reading uninitialized
    // memory (if we have that turned on)
    Mem[010 + 2 * iom + 1] = 0;

    /* 2*/ Mem[030 + 2 * iom] = dis0;                       // terminate interrupt vector (overwritten by bootload)
    int base_addr = base << 6; // 01400

    /* 3*/ Mem[base_addr + 7] = ((t_uint64) base_addr << 18) | 02000002;    // tally word for sys fault status
                // ??? Fault channel DCW

    // bootload_tape_label.alm says 04000, 43A239854 says 040000.  Since 43A239854 says
    // "no change", 40000 is correct; 4000 would be a large tally
    /* 4*/ Mem[base_addr + 010] = 040000;       // Connect channel LPW; points to PCW at 000000
    int mbx = base_addr + 4 * tape_chan;
    /* 5*/ Mem[mbx] = 03020003;             // Boot device LPW; points to IDCW at 000003
    /* 6*/ Mem[4] = 030 << 18;              // Second IDCW: IOTD to loc 30 (startup fault vector)

    // Default SCW points at unused first mailbox.
    // T&D tape overwrites this before the first status is savec, though.
    /* 7*/ Mem[mbx + 2] = ((t_uint64)base_addr << 18);      // SCW

    /* 8*/ Mem[0] = 0720201;                    // 1st word of bootload channel PCW

    // Why are we putting a port # in the 2nd word of the PCW?  The lower 27
    // bits of the odd word of a PCW should be all zero.
    /* 9*/ Mem[1] = ((t_uint64) tape_chan << 27) | port;        // 2nd word of PCW pair

    // following verified correct; instr 362 will not yield 1572 with a different shift
    /*10*/ Mem[2] = ((t_uint64) base_addr << 18) | pi_base | iom;   // word after PCW (used by program)

    /*11*/ Mem[3] = (cmd << 30) | (dev << 24) | 0700000;        // IDCW for read binary

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

    iom.channels[tape_chan] = DEV_TAPE;
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

extern UNIT cpu_unit;   // BUG: put in hdr
extern char* print_instr(t_uint64 word); // BUG: put in hdr
extern char* print_lpw(t_addr addr);    // BUG: put in hdr

static const char* prior_line;
static int prior_lineno;

//=============================================================================

/*
 * fprint_sym()
 *
 * Called by SIMH to print a memory location.
 *
 */

t_stat fprint_sym (FILE *ofile, t_addr simh_addr, t_value *val, UNIT *uptr, int32 sw)
{
    // log_msg(INFO_MSG, "SYS:fprint_sym", "addr is %012llo; val-ptr is %p, uptr is %p\n", simh_addr, val, uptr);

    if (uptr == &cpu_unit) {
        // memory request -- print memory specified by SIMH 
        addr_modes_t mode;
        unsigned segno;
        unsigned offset;
        if (addr_simh_to_emul(simh_addr, &mode, &segno, &offset) != 0)
            return SCPE_ARG;
        unsigned abs_addr;
        if (addr_any_to_abs(&abs_addr, mode, segno, offset) != 0)
            return SCPE_ARG;
        // note that parse_addr() was called by SIMH to determine the absolute addr.
        static int need_init = 1;
        if (need_init) {
            need_init = 0;
            prior_lineno = 0;
            prior_line = NULL;
        }
        /* First print matching source line if we're dumping instructions */
        if (sw & SWMASK('M')) {
            // M -> instr -- print matching source line if we have one
            const char* line;
            int lineno;
            seginfo_find_line(segno, offset, &line, &lineno);
            if (line == NULL && prior_lineno != lineno && prior_line != line) {
                fprintf(ofile, "\r\n");
                fprint_addr(ofile, NULL, simh_addr);    // UNIT doesn't include a reference to a DEVICE
                fprintf(ofile, ":\t");
                fprintf(ofile, "Line %d: %s\n", lineno, line);
                // fprintf(ofile, "%06o:\t", abs_addr); // BUG: print seg|offset too
                fprint_addr(ofile, NULL, simh_addr);    // UNIT doesn't include a reference to a DEVICE
                fprintf(ofile, ":\t");
                prior_lineno = lineno;
                prior_line = line;
            }
        }
        /* Next, we always output the numeric value */
        t_addr alow = abs_addr;
        t_addr ahi = abs_addr;
        fprintf(ofile, "%012llo", Mem[abs_addr]);
        if (sw & SWMASK('S')) {
            // SDWs are two words
            ++ ahi;
            fprintf(ofile, " %012llo", Mem[ahi]);
        }
        /* User may request (A)scii in addition to another format */
        if (sw & SWMASK('A')) {
            for (t_addr a = alow; a <= ahi; ++ a) {
                t_uint64 word = Mem[a];
                fprintf(ofile, " ");
                for (int i = 0; i < 4; ++i) {
                    uint c = word >> 27;
                    word = (word << 9) & MASKBITS(36);
                    if (c <= 0177 && isprint(c)) {
                        fprintf(ofile, "  '%c'", c);
                    } else {
                        fprintf(ofile, " \\%03o", c);
                    }
                }
            }
        }
        /* See if any other format was requested (but don't bother honoring multiple formats */
        if (sw & SWMASK('M')) {
            // M -> instr
            char *instr = print_instr(Mem[abs_addr]);
            fprintf(ofile, " %s", instr);
        } else if (sw & SWMASK('L')) {
            // L -> LPW
            fprintf(ofile, " %s", print_lpw(abs_addr));
        } else if (sw & SWMASK('P')) {
            // P -> PTW
            char *s = print_ptw(Mem[abs_addr]);
            fprintf(ofile, " %s", s);
        } else if (sw & SWMASK('S')) {
            // S -> SDW
            char *s = print_sdw(Mem[abs_addr], Mem[abs_addr+1]);
            fprintf(ofile, " %s", s);
        } else if (sw & SWMASK('A')) {
            // already done
        } else if (sw) {
            return SCPE_ARG;
        } else {
            // we already printed the numeric value, so nothing else to do
        }
        fflush(ofile);
        return SCPE_OK;
    } else if (sw & SIM_SW_REG) {
        // Print register
        REG* regp = (void*) uptr;
        // NOTE: We could also check regp->name to detect which registers should have special formatting
        if (regp && (regp->flags&REG_USER2)) {
            // PR registers
            // NOTE: Another implementation would be to have the value of each register always be its
            // index -- e.g. saved_ar_pr[5] would hold value 5.  Then the examine and deposit routines
            // could simply operate on the associated AR_PR registers.
            AR_PR_t pr;
            pr.PR.snr = *val & 077777;          // 15 bits
            pr.PR.rnr = (*val >> 15) & 07;      //  3 bits
            pr.PR.bitno = (*val >> 18) & 077;   //  6 bits
            pr.wordno = (*val >> 24);           // 18 bits
            pr.AR.charno = pr.PR.bitno / 9;
            pr.AR.bitno = pr.PR.bitno % 9;
            fprintf(ofile, "[ring %0o, address %0o|%0o, bitno %d]", pr.PR.rnr, pr.PR.snr, pr.wordno, pr.PR.bitno);
            fflush(ofile);
            return SCPE_OK;
        } else if (regp && (regp->flags&REG_USER1)) {
            // IR register
            fprintf(ofile, "%s", bin2text(*val, 18));
            IR_t ir;
            load_IR(&ir, *val);
            fprintf(ofile, " %s", ir2text(&ir));
            fflush(ofile);
            return SCPE_OK;
        } else if (regp && strcmp(regp->name,"PPR") == 0) {
            PPR_t ppr;
            load_PPR(*val, &ppr);
            fprintf(ofile, "[ring %0o, address %0o|%0o, priv %d]", ppr.PRR, ppr.PSR, ppr.IC, ppr.P);
            fflush(ofile);
            return SCPE_OK;
        } else
            return SCPE_ARG;
    } else 
        return SCPE_ARG;
}

//=============================================================================

t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
    log_msg(ERR_MSG, "SYS::parse_sym", "unimplemented\n");
    return SCPE_ARG;
}

//=============================================================================

int activate_timer()
{
    uint32 t;
    log_msg(DEBUG_MSG, "SYS::clock", "TR is %lld %#llo.\n", reg_TR, reg_TR);
    if (bit_is_neg(reg_TR, 27)) {
        if ((t = sim_is_active(&TR_clk_unit)) != 0)
            log_msg(DEBUG_MSG, "SYS::clock", "TR cancelled with %d time units left.\n", t);
        else
            log_msg(DEBUG_MSG, "SYS::clock", "TR loaded with negative value, but it was alread stopped.\n", t);
        sim_cancel(&TR_clk_unit);
        return 0;
    }
    if ((t = sim_is_active(&TR_clk_unit)) != 0) {
        log_msg(DEBUG_MSG, "SYS::clock", "TR was still running with %d time units left.\n", t);
        sim_cancel(&TR_clk_unit);   // BUG: do we need to cancel?
    }

    (void) sim_rtcn_init(CLK_TR_HZ, TR_CLK);
    sim_activate(&TR_clk_unit, reg_TR);
    if ((t = sim_is_active(&TR_clk_unit)) == 0)
        log_msg(DEBUG_MSG, "SYS::clock", "TR is not running\n", t);
    else
        log_msg(DEBUG_MSG, "SYS::clock", "TR is now running with %d time units left.\n", t);
    return 0;
}

//=============================================================================

t_stat clk_svc(UNIT *up)
{
    // only valid for TR
    (void) sim_rtcn_calb (CLK_TR_HZ, TR_CLK);   // calibrate clock
    uint32 t = sim_is_active(&TR_clk_unit);
    log_msg(INFO_MSG, "SYS::clock::service", "TR has %d time units left\n", t);
    return 0;
}

t_stat XX_clk_svc(UNIT *up)
{
    // only valid for TR
#if 0
    tmr_poll = sim_rtcn_calb (clk_tps, TMR_CLK);            /* calibrate clock */
    sim_activate (&clk_unit, tmr_poll);                     /* reactivate unit */
    tmxr_poll = tmr_poll * TMXR_MULT;                       /* set mux poll */
    todr_reg = todr_reg + 1;                                /* incr TODR */
    if ((tmr_iccs & TMR_CSR_RUN) && tmr_use_100hz)          /* timer on, std intvl? */
        tmr_incr (TMR_INC);                                 /* do timer service */
    return 0;
#else
    return 2;
#endif
}

//=============================================================================

static int inline is_octal_digit(char x)
{
    return isdigit(x) && x != '8' && x != '9';
}

//=============================================================================

/*
 * parse_addr()
 *
 * SIMH calls this function to parse an address.
 *
 * We return a packed format that encodes addressing mode, segment, and offset.
 *
 */

static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr)
{

    // Obsolete comments:
    //      SIMH wants the absolute address.   However, SIMH may pass the
    //      resulting address to fprint_sym() or other simulator functions
    //      that need to know the segment and offset.   So, we set the
    //      following globals to in order to communicate that info:
    //          int last_parsed_seg, int last_parsed_offset, t_addr
    //          last_parsed_addr

    // BUG/TODO: cleanup the last_parsed gunk that's no longer needed
    addr_modes_t last_parsed_mode;
    int last_parsed_seg;
    int last_parsed_offset;
    t_addr last_parsed_addr;

    char *cptr_orig = cptr;
    char debug_strp[1000]; strcpy(debug_strp, cptr);
    *optr = cptr;
    int force_abs = 0;
    int force_seg = 0;

    char *offsetp;
    int seg = -1;
    int pr = -1;
    unsigned int offset = 0;
    if ((offsetp = strchr(cptr, '|')) != NULL || ((offsetp = strchr(cptr, '$')) != NULL)) {
        // accept an octal segment number
        force_seg = 1;
        //log_msg(WARN_MSG, "parse_addr", "arg is '%s'\n", cptr);
        if (cptr[0] == 'P' && cptr[1] == 'R' && is_octal_digit(cptr[2]) && cptr+3 == offsetp) {
            // handle things like pr4|2,x7
            pr = cptr[2] - '0'; // BUG: ascii only
            seg = AR_PR[pr].PR.snr;
            offset = AR_PR[pr].wordno;
            //log_msg(WARN_MSG, "parse_addr", "PR[%d] uses 0%o|0%o\n", pr, seg, offset);
            cptr += 4;
        } else {
            if (!is_octal_digit(*cptr)) {
                out_msg("ERROR: Non octal digit starting at: %s\n.", cptr);
                return 0;
            }
            sscanf(cptr, "%o", (unsigned int *) &seg);
            cptr += strspn(cptr, "01234567");
            if (cptr != offsetp) {
out_msg("DEBUG: parse_addr: non octal digit within: %s\n.", cptr);
                return 0;
            }
            ++cptr;
        }
    } else
        if (*cptr == '#' || *cptr == '=') {     // SIMH won't let us use '=', so we provide '#'
            force_abs = 1;  // ignore TPR.TRS, interpret as absolute mode reference
            ++ cptr;
        }

    if (*cptr == 'X' && is_octal_digit(cptr[1]) && cptr[2] == '*') {
        int n = cptr[1] - '0';  // BUG: ascii only
        offset += reg_X[n];
        cptr += 3;
    } else {
        unsigned int off;
        sscanf(cptr, "%o", &off);
        offset += off;
        cptr += strspn(cptr, "01234567");
    }
    int mod_x = 0;
    if (cptr[0] == ',' && cptr[1] == 'X' && is_octal_digit(cptr[2])) {
        int n = cptr[2] - '0';  // BUG: ascii only
        mod_x = reg_X[n];
        offset += mod_x;
        cptr += 3;
    }
#if 0
    int is_indir;
    if ((is_indir = cptr[0] == ',' && cptr[1] == '*'))
        cptr += 2;
#endif
    
    prior_line = NULL;

    // uint addr;
    if (force_abs || (seg == -1 && get_addr_mode() == ABSOLUTE_mode)) {
        last_parsed_mode = ABSOLUTE_mode;
        *optr = cptr;
        // addr = offset;
        last_parsed_seg = -1;
        last_parsed_offset = offset;
        // last_parsed_addr = addr;
    } else {
        last_parsed_mode = APPEND_mode;
        last_parsed_seg = (seg == -1) ? TPR.TSR : seg;
        last_parsed_offset = offset;
        *optr = cptr;
    }

#if 0
    // This is too simple -- need to handle ITS/ITP tag fields, etc
    if (is_indir) {
        t_uint64 word;
        if (fetch_abs_word(addr, &word) != 0)
            return 0;
        addr = word & MASKBITS(24);
    }
#endif

#if 0
    if (last_parsed_mode == APPEND_mode)
        log_msg(INFO_MSG, "SYS::parse_addr", "String '%s' is %03o|%06o\n", debug_strp, last_parsed_seg, last_parsed_offset);
    else
        log_msg(INFO_MSG, "SYS::parse_addr", "String '%s' is %08o\n", debug_strp, last_parsed_offset);
    log_msg(INFO_MSG, "SYS::parse_addr", "Used %d chars; residue is '%s'.\n", cptr - cptr_orig, *optr);
#endif
    return addr_emul_to_simh(last_parsed_mode, last_parsed_seg, last_parsed_offset);
}

//=============================================================================

/*
 * fprint_addr()
 *
 * Called by SIMH to display an address
 *
 * Note that all addresses given by the simulator to SIMH are in a packed
 * format.
 */

static void fprint_addr(FILE *stream, DEVICE *dptr, t_addr simh_addr)
{
    // log_msg(INFO_MSG, "SYS:fprint_addr", "Device is %s; addr is %012llo; dptr is %p\n", dptr->name, simh_addr, dptr);

    addr_modes_t mode;
    unsigned segno;
    unsigned offset;
    if (addr_simh_to_emul(simh_addr, &mode, &segno, &offset) != 0)
        fprintf(stream, "<<<%08llo>>>", simh_addr);
    else
        if (mode == APPEND_mode)
            fprintf(stream, "%03o|%06o", segno, offset);
        else if (mode == BAR_mode)
            fprintf(stream, "BAR<<<%llo->%08o>>>", simh_addr, offset);  // BUG
        else
            fprintf(stream, "%08o", offset);
}

//=============================================================================

/*
 * addr_emul_to_simh()
 *
 * Encode an address for handoff to SIMH.
 *
 * The emulator gives SIMH a "packed" address form that encodes mode,
 * segment, and offset
 * 
 */

t_uint64 addr_emul_to_simh(addr_modes_t mode, unsigned segno, unsigned offset)
{
    if (mode == APPEND_mode) {
        if (offset >> 18 != 0) {
            log_msg(NOTIFY_MSG, "SYS::addr", "EMUL %03o|%06o overflows 18-bit offset.\n", segno, offset);
            cancel_run(STOP_BUG);
        }
        if (segno >> 15 != 0) {
            log_msg(NOTIFY_MSG, "SYS::addr", "EMUL %03o|%06o overflows 15-bit segment.\n", segno, offset);
            cancel_run(STOP_BUG);
        }
    } else {
        if (offset >> 24 != 0) {
            log_msg(NOTIFY_MSG, "SYS::addr", "EMUL %08o overflows 24-bit address.\n", offset);
            cancel_run(STOP_BUG);
        }
        segno = 0;
    }

    t_uint64 addr = offset & MASKBITS(24);
    addr |= (t_uint64) (segno & MASKBITS(15)) << 25;
    addr |= (t_uint64) (mode & MASKBITS(2)) << 41;
#if 0
    if (mode == APPEND_mode)
        log_msg(INFO_MSG, "SYS::addr", "EMUL %03o|%06o packs to %012llo\n", segno, offset, addr);
    else
        log_msg(INFO_MSG, "SYS::addr", "EMUL %08o packs to %012llo\n", offset, addr);
#endif
    return addr;
}

//=============================================================================

/*
 * addr_simh_to_emul()
 *
 * Decode an address returned by SIMH.
 *
 * The emulator gives SIMH a "packed" address form that encodes mode,
 * segment, and offset.
 * 
 */

int addr_simh_to_emul(t_uint64 addr, addr_modes_t *modep, unsigned *segnop, unsigned *offsetp)
{
    if (((addr >> 24) & 1) != 0) {
        log_msg(NOTIFY_MSG, "SYS::addr", "SIMH %012llo has an overflow on offset #.\n", addr);
        return 1;
    }
    if (((addr >> 40) & 1) != 0) {
        log_msg(NOTIFY_MSG, "SYS::addr", "SIMH %012llo has an overflow on segment #.\n", addr);
        return 1;
    }
    *offsetp = addr & MASKBITS(24);
    *segnop = (addr >> 25) & MASKBITS(15);
    *modep = (addr >> 41) & MASKBITS(2);
    if (*modep == APPEND_mode)
        if (*offsetp >> 18 != 0) {
            log_msg(NOTIFY_MSG, "SYS::addr", "SIMH %012llo aka %03o|%06o overflows 18-bit offset.\n", addr, *segnop, *offsetp);
        return 1;
    }
#if 0
    if (*modep == APPEND_mode)
        log_msg(INFO_MSG, "SYS::addr", "SIMH %012llo is %03o|%06o\n", addr, *segnop, *offsetp);
    else
        log_msg(INFO_MSG, "SYS::addr", "SIMH %012llo is %08o\n", addr, *offsetp);
#endif
    return 0;
}

//=============================================================================
