/*
    hw6180_sys.c -- Most of the interfaces to SIMH.
*/

#include "hw6180.h"
#include <ctype.h>

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
    // &cpu_dev2,
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

extern CTAB *sim_vm_cmd;
static struct sim_ctab sim_cmds[] =  {
    { "XSYMTAB",  cmd_symtab_parse, 0, "xsymtab [...]            define symtab entries\n" },
    { "XVMDUMP",  cmd_dump_vm, 0,      "xvmdump                  dump virtual memory caches\n" },
    { "XHISTORY", cmd_dump_history, 0, "xhistory                 display recent instruction counter values\n" },
    { "XSEGINFO", cmd_seginfo, 0,      "xseginfo <seg>           walk segment linkage table\n" },
    { "XFIND",    cmd_find, 0,         "xfind <string> <range>   search memory for string\n" },
    { "XLIST",    cmd_load_listing, 0, "xlist <addr> <source>    load pl1 listing\n" },
    { "XDEBUG",   cmd_xdebug, 0,       "xdebug seg <#> {on|off}  finer grained debugging\n" },
    { 0, 0, 0, 0}
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

// The following are set by parse_addr() and used by fprint_sym()
static int last_parsed_seg;
static int last_parsed_offset;
static t_addr last_parsed_addr;

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

// static void init_memory_iox(void);
static void init_memory_iom(void);

static void hw6180_init(void)
{
    out_msg("DEBUG:    SYS::init", "Once-only initialization running.\n");

    fault_gen_no_fault = 0;
    sim_vm_parse_addr = parse_addr;
    sim_vm_fprint_addr = fprint_addr;
    sim_vm_cmd = sim_cmds;

    mt_init();
    console_init();

    // todo: set debug flags for all devices
    // cpu_dev.dctrl = 1;
    // tape_dev.dctrl = 1;

    sim_brk_types = SWMASK('E');    // execution
    sim_brk_types |= SWMASK('M');   // memory (absolute address)
    sim_brk_types |= SWMASK('W');   // memory write (absolute address)
    sim_brk_dflt = SWMASK('E');

    // Hardware config -- todo - should be based on config cards!
    // BUG: need to write config deck at 012000

    // CPU Switches
    memset(&switches, 0, sizeof(switches));
    switches.cpu_num = 0;   // only one cpu
    // multics uses same vector for interrupts & faults?
    // OTOH, AN87, 1-41 claims faults are at 100o ((flt_base=1)<<5) // BUG 1<<5 is 040...
    // switches.FLT_BASE = 0;   // multics uses same vector for interrupts & faults?
#if 0
    switches.FLT_BASE = 0100;
#else
    // FLT_BASE switches are 7 MSB of 12bit addr
    switches.FLT_BASE = 2;  // multics requires setting 02 resulting in vector at 0100
    // switches.FLT_BASE = 0163; // diag tape allows any location *except* 02 --> 0100
    // switches.FLT_BASE = 0; // diag tape allows any location *except* 02 --> 0100
#endif

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


    init_memory_iom();      // IOX includes unknown instr ldo and has an undocumented mailbox architecture
    //init_memory_iox();

    // Only two of the four SCU masks are used; these correspond to the "A" and "B" rotary switches
    scu.eima_data[0].avail = 1;
    scu.eima_data[1].avail = 1;

    // CPU port 'b'(1) connected to SCU port '7' -- arbitrary
    int cpu_port = 1;
    cpu_ports.scu_port = 7;
    cpu_ports.ports[cpu_port] = cpu_ports.scu_port; // port B connected to SCU
    scu.ports[cpu_ports.scu_port] = cpu_port;   // SCU port '7' connected to CPU port 'b'
    // GB61, pages 9-1 and A-2: Set Mask A to port that the bootload CPU is connected to; Set Mask B to off
    // scu.mask_assign[0] = 1 << cpu_ports.scu_port;
    scu.eima_data[0].raw = 1 << (8 - cpu_ports.scu_port);
    scu.eima_data[0].assigned = 1;
    scu.eima_data[0].port = cpu_ports.scu_port;

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
    // On the physical hardware, settings of various switchs are reflected into memory.  We provide
    // no support for simulation of the physical switches because there is only one
    // useful value for almost all of the switches.  So, we hard code the memory values
    // that represent usable switch settings.
    //
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
    int pi_base = 01200;    // 15 bits; interrupt cells; bootload_io.alm insists that we match template_slt_$iom_mailbox_absloc
    int iom = 0;            // 3 bits; only IOM 0 would use vector 030

    t_uint64 cmd = 5;       // 6 bits; 05 for tape, 01 for cards
    int dev = 0;        // 6 bits: drive number

    t_uint64 imu = 0;       // 1 bit; Maybe an is-IMU flag; IMU is later version of IOM

#define MAXMEMSIZE (16*1024*1024)   /* BUG */
    memset(M, 0, MAXMEMSIZE*sizeof(M[0]));

#if 0
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
    M[mbx+07] = ((t_uint64) base << 24) | (02 << 18) | 02;      // Fault channel DCW
    // log_msg(DEBUG_MSG, "SYS", "IOM MBX @%#o: %#llo\n", mbx+7, M[mbx+7]);
    M[mbx+010] = 04000;                             // Connect channel LPW -> PCW at 000000

    // Channel mailbox, at Base*64 + 4*Chan#
    mbx = (base * 64) + 4 * tape_chan;
    M[mbx+0] = (3<<18) | (2<<12) | 3;                   //  Boot dev LPW -> IDCW @ 000003
    // log_msg(DEBUG_MSG, "SYS", "Channel MBX @%0o: %#llo\n", mbx, M[mbx]);
    M[mbx+2] = ((t_uint64) base <<24);                          //  Boot dev SCW -> IOM mailbox
#endif

#if 1
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
    /* 1*/ M[010 + 2 * iom] = (imu << 34) | dis0;           // system fault vector; DIS 0 instruction (imu bit not mentioned by 43A239854)
    /* 2*/ M[030 + 2 * iom] = dis0;                     // terminate interrupt vector (overwritten by bootload)
    int base_addr = base << 6; // 01400

// 18 bit or 24 bit?
    // /* 3*/ M[base_addr + 7] = ((t_uint64) base_addr << 24) | 02000002;   // tally word for sys fault status
    /* 3*/ M[base_addr + 7] = ((t_uint64) base_addr << 18) | 02000002;  // tally word for sys fault status

    /* 4*/ M[base_addr + 010] = 040000;     // Connect channel LPW
    int mbx = base_addr + 4 * tape_chan;
    /* 5*/ M[mbx] = 03020003;               // Boot device LPW
    /* 6*/ M[4] = 030 << 18;                // Second IDCW: IOTD to loc 30 (startup fault vector)
    /* 7*/ M[mbx + 2] = ((t_uint64)base_addr << 24);        // SCW -- verified correct
    /* 8*/ M[0] = 0720201;                  // 1st word of bootload channel PCW
    /* 9*/ M[1] = ((t_uint64) tape_chan << 27) | port;      // 2nd word of PCW pair

    // following verified correct; instr 362 will not yield 1572 with a different shift
    /*10*/ M[2] = ((t_uint64) base_addr << 18) | pi_base | iom; // word after PCW (used by program)

    /*11*/ M[3] = (cmd << 30) | (dev << 24) | 0700000;      // IDCW for read binary

#endif
    
}

#if 0

static void init_memory_iox()
{
    // On the physical hardware, settings of various switchs are reflected into memory.  We provide
    // no support for simulation of the physical switches because there is only one
    // useful value for almost all of the switches.  So, we hard code the memory values
    // that represent usable switch settings.
    //
    // All values from bootload_tape_label.alm
    // See also doc #43A239854.
    // BUG: This is for an IOM.  Do we want an IOM or an IOX?
    // The presence of a 0 in the top six bits of word 0 denote an IOM boot from an IOX boot

    // " The channel number ("Chan#") is set by the switches on the IOM to be the
    // " channel for the tape subsystem holding the bootload tape. The drive number
    // " for the bootload tape is set by switches on the tape MPC itself.

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

#define MAXMEMSIZE (16*1024*1024)   /* BUG */
    memset(M, 0, MAXMEMSIZE*sizeof(M[0]));

    //  6/Command, 6/Device#, 6/0, 18/700000
    M[0] = (cmd << 30) | (dev << 24) | 0700000; // Bootload IDCW
    M[1] = 030 << 18;                           // Second IDCW: IOTD to loc 30 (startup fault vector)
    // 24/7000000,12/IOXoffset
    M[4] = ((t_uint64)07000000 << 12) | iox_offset;     // A register value for connect

    M[010] = (1<<18) | 0612000;                 // System fault vector; a HALT instruction
    M[030] = (010<<18) | 0612000;               // Terminate interrupt vector (overwritten by bootload)

    // IOX Mailbox
    M[001400] = 0;      // base addr 0
    M[001401] = 0;      // base addr 1
    M[001402] = 0;      // base addr 2
    M[001403] = 0;      // base addr 3
    M[001404] = ((t_uint64)0777777<<18);    // bound 0, bound 1
    M[001405] = 0;              // bound 2, bound 3
    M[001406] = 03034;          // channel link word
    M[001407] = (0400 << 9) | 0400; // lpw
        // but what the heck lpw is Chan 01 -- [dcw=00 ires=1 hrel=0 ae=0 nc=0 trun=0 srel=0 tally=0400] [lbnd=00 size=05(5) idcw=020001]

    // The following were set by bootload..
    //M[001407] = 001402000002; // Chan 01 -- [dcw=01402 ires=0 hrel=0 ae=0 nc=0 trun=0 srel=0 tally=02]
    //M[001410] = 000000040000; // [lbnd=00 size=00(0) idcw=040000]
    //M[001402] = 000000000000;
    //M[001403] = 000000000000;
    //M[040000] = 013732054000;

    /* Described in A43A239854_600B_IOM_Spec_Jul75.pdf */
    // LPW for connect channel
    // M[001410] = 05040000;    // LPW for connect channel; NC=1; DCW=5
    M[001410] = 05020001;   // LPW for connect channel; NC=0, tro=1, tally=1, DCW=5
    // PCW for connect channel -- we'll arbitrarily use words 5 and 6
    M[5] = 0720201;                 // Bootload channel PCW, word 1 (this is an 18 bit value)
    M[6] = ((t_uint64) tape_chan << 27) | port;     // Bootload channel PCW, word 2
    // LPW for bootload channel (channel #5) -- BUG, we probably need one...
    // NOTE: Two DCW words for bootload channel are at location zero
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

        /* First print matching source line if we're dumping instructions */
        if (sw & SWMASK('M')) {
            // M -> instr -- print matching source line if we have one
            int offset = last_parsed_offset + addr - last_parsed_addr;
            if (offset >= 0) {
                where_t where;
                seginfo_find_all(last_parsed_seg, offset, &where);
                if (where.line != NULL) {
                    fprintf(ofile, "Line %d: %s\n", where.line_no, where.line);
                    fprintf(ofile, "%06o:\t", addr);
                }
            }
        }
        /* Next, we always output the numeric value */
        t_addr alow = addr;
        t_addr ahi = addr;
        fprintf(ofile, "%012llo", M[addr]);
        if (sw & SWMASK('S')) {
            // SDWs are two words
            ++ ahi;
            fprintf(ofile, " %012llo", M[ahi]);
        }
        /* User may request (A)scii in addition to another format */
        if (sw & SWMASK('A')) {
            for (t_addr a = alow; a <= ahi; ++ a) {
                t_uint64 word = M[a];
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
            char *instr = print_instr(M[addr]);
            fprintf(ofile, " %s", instr);
        } else if (sw & SWMASK('L')) {
            // L -> LPW
            fprintf(ofile, " %s", print_lpw(addr));
        } else if (sw & SWMASK('P')) {
            // P -> PTW
            char *s = print_ptw(M[addr]);
            fprintf(ofile, " %s", s);
        } else if (sw & SWMASK('S')) {
            // S -> SDW
            char *s = print_sdw(M[addr], M[addr+1]);
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
        } else
            return SCPE_ARG;
    } else 
        return SCPE_ARG;
}


t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
    log_msg(ERR_MSG, "SYS::parse_sym", "unimplemented\n");
    return SCPE_ARG;
}

int activate_timer()
{
    uint32 t;
    log_msg(DEBUG_MSG, "SYS::clock", "TR is %lld %#llo.\n", reg_TR, reg_TR);
    if (bit27_is_neg(reg_TR)) {
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


t_stat clk_svc(UNIT *up)
{
    // only valid for TR
    (void) sim_rtcn_calb (CLK_TR_HZ, TR_CLK);   // calibrate clock
    uint32 t = sim_is_active(&TR_clk_unit);
    log_msg(DEBUG_MSG, "SYS::clock::service", "TR has %d time units left\n");
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

static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr)
{

    // SIMH calls this function to parse an address.   SIMH wants the
    // absolute address.   However, SIMH may pass the resulting address
    // to fprint_sym() or other simulator functions that need to know
    // the segment and offset.   So, we set the following globals to in
    // order to communicate that info:
    //      int last_parsed_seg, int last_parsed_offset, t_addr last_parsed_addr

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
    
    uint addr;
    if (force_abs || (seg == -1 && get_addr_mode() == ABSOLUTE_mode)) {
        *optr = cptr;
        addr = offset;
        last_parsed_seg = -1;
        last_parsed_offset = offset;
        last_parsed_addr = addr;
    } else {
        uint saved_seg = -1;
        if (seg != -1) {
            saved_seg = TPR.TSR;
            TPR.TSR = seg;
        }
        fault_gen_no_fault = 1;
        if (get_seg_addr(offset, 0, &addr) != 0) {
            if (saved_seg != -1)
                TPR.TSR = saved_seg;
            fault_gen_no_fault = 0;
            *optr = cptr;
            return 0;
        }
        fault_gen_no_fault = 0;
        if (saved_seg != -1)
            TPR.TSR = saved_seg;
        last_parsed_seg = seg;
        last_parsed_offset = offset;
        last_parsed_addr = addr;
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
    return addr;
}


static void fprint_addr(FILE *stream, DEVICE *dptr, t_addr addr)
{
    fprintf(stream, "%06o", addr);
}
