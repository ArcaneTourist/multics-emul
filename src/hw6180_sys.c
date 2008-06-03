#include "hw6180.h"
#include <ctype.h>

extern uint32 sim_brk_types, sim_brk_dflt, sim_brk_summ; /* breakpoint info */
extern t_addr (*sim_vm_parse_addr)(DEVICE *, char *, char **);
extern void (*sim_vm_fprint_addr)(FILE *, DEVICE *, t_addr);
static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr);
static void fprint_addr(FILE *stream, DEVICE *dptr, t_addr addr);

extern DEVICE cpu_dev;
extern DEVICE tape_dev;
extern DEVICE opcon_dev;
//extern DEVICE dsk_dev;
extern REG cpu_reg[];
extern t_uint64 M[];

extern UNIT TR_clk_unit;
extern switches_t switches;
extern cpu_ports_t cpu_ports;
extern scu_t scu;
extern iom_t iom;

/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
   sim_load             binary loader
*/

char sim_name[] = "hw6180";

REG *sim_PC = &cpu_reg[0];

int32 sim_emax = 4;

/* Multics allows up to 7 CPUs... */

DEVICE *sim_devices[] = {
    &cpu_dev,
    &tape_dev,
    &opcon_dev,
    NULL
};

const char *sim_stop_messages[] = {
    "<error zero>",
    "Memory is empty",
    "BUG-STOP -- Internal error, further execution probably pointless",
    "WARN-STOP -- Internal error, further processing might be ok",
    "Fetch on Odd address",
    "Breakpoint",
    // "Invalid Opcode"
    0
};


// One-time-only initialization for emulator
static void hw6180_init(void);
void (*sim_vm_init)(void) = hw6180_init;

/*
    Variables custom to HW6180 that aren't normally present in other
    SIMH emulators.
*/
int bootimage_loaded = 0;

/*  SIMH binary loader.
        The load normally starts at the current value of the PC.
    Args
        fileref -- file opened by SIMH
        cptr -- VM specific args (from cmd line?)
        fnam -- filename
        write_flag -- indicates whether to load or write
    
*/

t_stat sim_load (FILE *fileref, char *cptr, char *fnam, int32 write_flag)
{
    /*  Maybe:
            This emulator will load boot tape format files starting
            at location 30 as an emulation of the physical boot loaders.
        Or:
            Define a boot procedure so the user can use the "boot" command.
            However, the boot command may not allow specification of a
            command.
    */
    bootimage_loaded = 1;
    abort();
}

static void init_memory_iox(void);
static void init_memory_iom(void);

static void hw6180_init(void)
{
    debug_msg("SYS::init", "Once-only initialization running.\n");

    fault_gen_no_fault = 0;
    sim_vm_parse_addr = parse_addr;
    sim_vm_fprint_addr = fprint_addr;

    mt_init();

    // todo: set debug flags for all devices
    cpu_dev.dctrl = 1;  // todo: don't default debug to on
    tape_dev.dctrl = 1;

    sim_brk_types = SWMASK('E') | SWMASK('M');  // M memory (absolute address)
    sim_brk_dflt = SWMASK('E');

    // Hardware config -- todo - should be based on config cards!
    // BUG: need to write config deck at 012000

    // CPU Switches
    memset(&switches, 0, sizeof(switches));
    // multics uses same vector for interrupts & faults?
    // OTOH, AN87, 1-41 claims faults are at 100o ((flt_base=1)<<5)
    switches.FLT_BASE = 0;  // multics uses same vector for interrupts & faults?

    // Only one SCU
    memset(&scu, 0, sizeof(scu));
    for (int i = 0; i < ARRAY_SIZE(scu.ports); ++i)
        scu.ports[i] = -1;

    // Only one IOM
    memset(&iom, 0, sizeof(iom));
    for (int i = 0; i < ARRAY_SIZE(iom.ports); ++i) {
        iom.ports[i] = -1;
        iom.channels[i] = DEV_NONE;
    }

    // Only one CPU
    memset(&cpu_ports, 0, sizeof(cpu_ports));
    for (int i = 0; i < ARRAY_SIZE(cpu_ports.ports); ++i)
        cpu_ports.ports[i] = -1;

    init_memory_iom();      // IOX includes unknown instr ldo

    // CPU port 'b'(1) connected to SCU port '7' -- arbitrary
    cpu_ports.scu_port = 7;
    cpu_ports.ports[1] = cpu_ports.scu_port;    // port B connected to SCU
    scu.ports[cpu_ports.scu_port] = 1;  // SCU port '7' connected to CPU port 'b'
    scu.mask_assign[0] = 1 << cpu_ports.scu_port;       // GB61, page 9-1

    // IOM port 'a'(0) connected to SCU port 0
    iom.scu_port = 0;
    iom.ports[0] = iom.scu_port;    // port A connected to SCU
    scu.ports[iom.scu_port] = 0;

    int con_chan = 012; // channels 010 and higher are probed for an operators console
    iom.channels[con_chan] = DEV_CON;
    iom.devices[con_chan] = &opcon_dev;
}


static void init_memory_iom()
{
    // All values from bootload_tape_label.alm
    // See also doc #43A239854.
    // BUG: This is for an IOM.  Do we want an IOM or an IOX?
    // The presence of a 0 in the top six bits of word 0 denote an IOM boot from an IOX boot

    // " The channel number ("Chan#") is set by the switches on the IOM to be the
    // " channel for the tape subsystem holding the bootload tape. The drive number
    // " for the bootload tape is set by switches on the tape MPC itself.

    int tape_chan = 036;        // 12 bits or 6 bits;   // Arbitrary; controller channel; max=40
    int port = iom.scu_port;    // 3 bits;  // SCU port # to which bootload IOM is attached (deduced)

    iom.channels[tape_chan] = DEV_TAPE;
    iom.devices[tape_chan] = &tape_dev;

    int base = 014;         // 12 bits; IOM base
    int pi_base = 01200;    // 15 bits; interrupt cells
    int iom = 0;            // 3 bits; only IOM 0 would use vector 030

    t_uint64 cmd = 5;       // 6 bits; 05 for tape, 01 for cards
    int dev = 0;        // 6 bits: drive number

    t_uint64 imu = 0;       // 1 bit; Maybe an is-IMU flag; IMU is later version of IOM

    // arbitrary -- zero first 4K words
    //for (int i = 0; i < 4096; ++i)
    //  M[i] = 0;
#define MAXMEMSIZE (16*1024*1024)   /* BUG */
    memset(M, 0, MAXMEMSIZE*sizeof(M[0]));

    M[0] = 0720201;                 // Bootload channel PCW, word 1 (this is an 18 bit value)
    //  3/0, 6/Chan#, 30/0, 3/Port -- NOT 3/0, 12/Chan#, 24/0, 3/Port# -- also non-zero port may not be valid for low bits
    M[1] = ((t_uint64) tape_chan << 27) | port;     // Bootload channel PCW, word 2
    // 12/Base, 6/0, 15/PIbase, 3/IOM#
    // M[2] = ((t_uint64) base << 24) | (pi_base << 3) | iom;   // Info used by bootloaded pgm
    M[2] = ((t_uint64) base << 24) | pi_base;   // Info used by bootloaded pgm -- force iom zero
    // 6/Command, 6/Device#, 6/0, 18/700000; Bootload IDCW - Command is 05 for tape, 01 for cards.
    M[3] = (cmd << 30) | (dev << 24) | 0700000;     // Bootload IDCW
    M[4] = 030 << 18;               // Second IDCW: IOTD to loc 30 (startup fault vector)

    // bootload_info$cold_tape_mpc is at location 7.   bootload_tape_fw$boot examines this value
    // M[7] = 1;

    t_uint64 dis0 = 0616200;
    M[010 + 2 * iom] = (imu << 34) | dis0;          // system fault vector; DIS 0 instruction
    M[030 + 2 * iom] = dis0;                        // terminate interrupt vector (overwritten by bootload)

    // IOM Mailbox, at Base*6
    int mbx = base * 64;
    M[mbx+07] = (base << 24) | (02 << 18) | 02;     // Fault channel DCW
    // debug_msg("SYS", "IOM MBX @%0o: %0Lo\n", mbx+7, M[mbx+7]);
    M[mbx+010] = 04000;                             // Connect channel LPW -> PCW at 000000

    // Channel mailbox, at Base*64 + 4*Chan#
    mbx = (base * 64) + 4 * tape_chan;
    M[mbx+0] = (3<<18) | (2<<12) | 3;                   //  Boot dev LPW -> IDCW @ 000003
    // debug_msg("SYS", "Channel MBX @%0o: %0Lo\n", mbx, M[mbx]);
    M[mbx+2] = (base <<24);                         //  Boot dev SCW -> IOM mailbox
    
}

#if 0

init_memory_iox() untested.   Never updated for a console

static void init_memory_iox()
{
    // All values from bootload_tape_label.alm
    // See also doc #43A239854.
    // This is for an IOX
    // init_memory_iom() is more up to date...

// " The channel number ("Chan#") is set by the switches on the IOM to be the
// " channel for the tape subsystem holding the bootload tape. The drive number
// " for the bootload tape is set by switches on the tape MPC itself.

    int cmd = 5;        // 6 bits; 05 for tape, 01 for cards
    int dev = -1;           // 6 bits

    int iox_offset = -1;    // 12 bits

    //int chan = -1;        // 12 bits;
    //int port = -1;        // 3 bits; 

    //int base = 012;       // 12 bits; IOM base; must be 0012 for Multics
    //int pi_base = -1; // 15 bits
    //int iom = 0;      // 3 bits; only IOM 0 would use vector 030


    int imu = -1;       // 1 bit

    // arbitrary -- zero first 4K words
    for (int i = 0; i < 4096; ++i)
        M[i] = 0;

    //  6/Command, 6/Device#, 6/0, 18/700000
    M[0] = (cmd << 30) | (dev << 24) | 0700000; // Bootload IDCW
    M[1] = 030 << 18;                           // Second IDCW: IOTD to loc 30 (startup fault vector)
    // 24/7000000,12/IOXoffset
    M[3] = (07000000 << 12) | iox_offset;       // A register value for connect

    M[010] = (1<<18) | 0612000;                 // System fault vector; a HALT instruction
    M[030] = (010<<18) | 0612000;               // Terminate interrupt vector (overwritten by bootload)

    // IOM Mailbox
    M[001400] = 0;      // base addr 0
    M[001401] = 0;      // base addr 1
    M[001402] = 0;      // base addr 2
    M[001403] = 0;      // base addr 3
    M[001404] = (0777777<<18);  // bound 0, bound 1
    M[001405] = 0;              // bound 2, bound 3
    M[001406] = 03034;          // channel link word
    M[001407] = (0400 << 27) | 0400;    // lpw
}
#endif


extern UNIT cpu_unit;   // BUG: put in hdr
extern char* print_instr(t_uint64 word); // BUG: put in hdr
extern char* print_lpw(t_addr addr);    // BUG: put in hdr

t_stat fprint_sym (FILE *ofile, t_addr addr, t_value *val, UNIT *uptr, int32 sw)
{
    if (uptr == &cpu_unit) {
        // memory request -- print memory specified by SIMH absolute M[addr].  However
        // note that parse_addr() was called by SIMH to determine the absolute addr.
        if (sw & SWMASK('M')) {
            // M -> instr
            char *instr = print_instr(M[addr]);
            fprintf(ofile, "%012Lo %s", M[addr], instr);
        } else if (sw & SWMASK('L')) {
            // L -> LPW
            fprintf(ofile, "%012Lo %s", M[addr], print_lpw(addr));
        } else if (sw & SWMASK('P')) {
            // P -> PTW
            char *s = print_ptw(M[addr]);
            fprintf(ofile, "%012Lo %s", M[addr], s);
        } else if (sw & SWMASK('S')) {
            // S -> SDW
            char *s = print_sdw(M[addr], M[addr+1]);
            fprintf(ofile, "%012Lo %012Lo %s", M[addr], M[addr+1], s);
        } else if (sw & SWMASK('A')) {
            t_uint64 word = M[addr];
            fprintf(ofile, "%012Lo ", word);
            for (int i = 0; i < 4; ++i) {
                uint c = word >> 27;
                word = (word << 9) & MASKBITS(36);
                if (c <= 0177 && isprint(c)) {
                    fprintf(ofile, " '%c'", c);
                } else {
                    fprintf(ofile, " \\%03o", c);
                }
            }
        } else if (sw) {
            return SCPE_ARG;
        } else {
            fprintf(ofile, "%012Lo", M[addr]);
        }
        fflush(ofile);
        return SCPE_OK;
    } else
        return SCPE_ARG;
}

t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
    complain_msg("SYS::parse_sym", "unimplemented\n");
    return SCPE_ARG;
}

int activate_timer()
{
    uint32 t;
    debug_msg("SYS::clock", "TR is %Ld 0%Lo.\n", reg_TR, reg_TR);
    if (bit27_is_neg(reg_TR)) {
        if ((t = sim_is_active(&TR_clk_unit)) != 0)
            debug_msg("SYS::clock", "TR cancelled with %d time units left.\n", t);
        else
            debug_msg("SYS::clock", "TR loaded with negative value, but it was alread stopped.\n", t);
        sim_cancel(&TR_clk_unit);
        return 0;
    }
    if ((t = sim_is_active(&TR_clk_unit)) != 0) {
        debug_msg("SYS::clock", "TR was still running with %d time units left.\n", t);
        sim_cancel(&TR_clk_unit);   // BUG: do we need to cancel?
    }

    (void) sim_rtcn_init(CLK_TR_HZ, TR_CLK);
    sim_activate(&TR_clk_unit, reg_TR);
    if ((t = sim_is_active(&TR_clk_unit)) == 0)
        debug_msg("SYS::clock", "TR is not running\n", t);
    else
        debug_msg("SYS::clock", "TR is now running with %d time units left.\n", t);
    return 0;
}


t_stat clk_svc(UNIT *up)
{
    // only valid for TR
    (void) sim_rtcn_calb (CLK_TR_HZ, TR_CLK);   // calibrate clock
    uint32 t = sim_is_active(&TR_clk_unit);
    debug_msg("SYS::clock::service", "TR has %d time units left\n");
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

static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr)
{
    *optr = cptr;
    //if (strcmp(dptr->name, "CPU") != 0) {
    //  return 0;
    //}
    int force_abs = 0;
    int force_seg = 0;
    if (*cptr == '#') {     // SIMH won't let us use '='
        force_abs = 1;
        ++ cptr;
    } else if (*cptr == '|' || *cptr == '$') {
        force_seg = 1;
        ++ cptr;
    }
    if (!isdigit(*cptr) || *cptr == '8' || *cptr == '9')
        return 0;
    int seg = -1;
    unsigned int offset;
    sscanf(cptr, "%o", &offset);
    cptr += strspn(cptr, "01234567");
    if (*cptr == '|' || *cptr == '$') {
        if (force_abs || force_seg)
            return 0;
        ++ cptr;
        seg = offset;
        sscanf(cptr, "%o", &offset);
        cptr += strspn(cptr, "01234567");
    }
    if (force_abs || (seg == -1 && get_addr_mode() == ABSOLUTE_mode)) {
        *optr = cptr;
        return offset;
    }
    uint addr;
    uint saved_seg = -1;
    if (seg != -1) {
        TPR.TSR = seg;
    }
    fault_gen_no_fault = 1;
    if (get_seg_addr(offset, 0, &addr) != 0) {
        if (saved_seg != -1)
            TPR.TSR = saved_seg;
        fault_gen_no_fault = 0;
        return 0;
    }
    fault_gen_no_fault = 0;
    if (saved_seg != -1)
        TPR.TSR = saved_seg;
    *optr = cptr;
    return addr;
}

static void fprint_addr(FILE *stream, DEVICE *dptr, t_addr addr)
{
    fprintf(stream, "%06o", addr);
}
