/*
    hw6180_sys.c -- Most of the interfaces to SIMH.
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#include "hw6180.h"
#include "seginfo.hpp"
#include <ctype.h>


// The following are assigned to SIMH function pointers
static t_addr parse_addr(DEVICE *dptr, char *cptr, char **optr);
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
    { "XDEBUG",   cmd_xdebug, 0,       "xdebug seg <#> {on|default|off}  finer grained debugging\n" },
    { "XFIND",    cmd_find, 0,         "xfind <string> <range>           search memory for string\n" },
    { "XLIST",    cmd_load_listing, 0, "xlist <addr> <source>            load pl1 listing\n" },
    { "XSYMTAB",  cmd_symtab_parse, 0, "xsymtab [...]                    define symtab entries\n" },
    { "XSTATS",   cmd_stats, 0,        "xstats                           display statistics\n" },
#if 0
    // replaced by "show" modifiers
    { "XHISTORY", cmd_dump_history, 0, "xhistory                         display recent instruction counter values\n" },
    { "XSTACK",   cmd_stack_trace, 0,  "xstack                           dump Multics procedure call stack\n" },
    { "XSEGINFO", cmd_seginfo, 0,      "xseginfo <seg>                   walk segment linkage table\n" },
    { "XVMDUMP",  cmd_dump_vm, 0,      "xvmdump                          dump virtual memory caches\n" },
#endif
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
 * Note that this runs before SIMH loads any "ini" file specified on
 * the command line.
 *
 * TODO: Provide a more general function interface for specifying
 * that two devices are interconnected.
 *
 * TODO: Move some of this code into device reset routines (especially
 * anything depending on sys_opts that will be allowed to change).
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
    sim_brk_types |= SWMASK('D');   // auto-display (w/o stopping)
    sim_brk_dflt = SWMASK('E');

    // System-wide options
    memset(&sys_opts, 0, sizeof(sys_opts));
    sys_opts.clock_speed = 250000; // about 1/4 of a MIP
    // Negative times imply instantaneous operation without making
    // use of sim_activate().  Zero times have almost the same
    // result, except that the caller queues an immediate run via
    // sim_activate() and then returns.  The zero wait event(s) will
    // be noticed and handled prior to the next instruction execution.
    sys_opts.iom_times.connect = 3; // 0
    sys_opts.iom_times.chan_activate = -1;  // unimplemented
    sys_opts.mt_times.read = 3; // -1; 100; 1000;
    sys_opts.mt_times.xfer = -1;            // unimplemented
    sys_opts.warn_uninit = 1;
    sys_opts.startup_interrupt = 1;


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
    sys_opts.tape_chan = 14;        // 12 bits or 6 bits; controller channel

    // Hardware config -- todo - should be based on config cards!
    // BUG/TODO: need to write config deck at 012000 ? Probably not

    // Only one CPU
    memset(&cpu_ports, 0, sizeof(cpu_ports));
    for (unsigned i = 0; i < ARRAY_SIZE(cpu_ports.ports); ++i)
        cpu_ports.ports[i] = -1;

    // CPU Switches
    memset(&switches, 0, sizeof(switches));
    switches.cpu_num = 0;   // CPU 'A' is bootload cpu (init_early_config.pl1)
    // FLT_BASE switches are the 7 MSB of a 12bit addr
    // AN87, 1-41 claims multics requires faults to be at 100o
    switches.FLT_BASE = 2;  // 2<<5 == 0100
    // At one time, it seemed that the diag tape required using different
    // fault base switch settings.  However, that no longer seems to be the case.

    // Only one SCU
    memset(&scu, 0, sizeof(scu));
    scu.mode = 1;   // PROGRAM mode
    for (unsigned i = 0; i < ARRAY_SIZE(scu.ports); ++i)
        scu.ports[i].idnum = -1;

    // BUG/TODO: the following belongs in a scu_reset()
    for (unsigned i = 0; i < ARRAY_SIZE(scu.ports); ++i)
        scu.ports[i].is_enabled = 0;
    for (unsigned i = 0; i < ARRAY_SIZE(scu.interrupts); ++i) {
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
#if FEAT_MEM_CHECK_UNINIT
    memset(Mem, 0xff, MAXMEMSIZE*sizeof(Mem[0]));
#else
    memset(Mem, 0, MAXMEMSIZE*sizeof(Mem[0]));
#endif

    // CPU port 'a' connected to port '5' of SCU
    // scas_init's call to make_card seems to require that the CPU be connected
    // to SCU port zero.
    // Also, rsw_util$port_info claims base addr is: port-assignment * size/1024
    //int cpu_port = 4;       // CPU port 'd' or 4
    // However, MR12.1 error_msgs.compout says CPUs should be on higher
    // port numbers than IOMs and Bulk Stores
    int cpu_port = 0;       // CPU port 'a' or 0
    cpu_ports.scu_port = 5;
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
    sys_opts.opcon_chan = 012; // channels 010 and higher are probed for an operators console
    iom.channels[sys_opts.opcon_chan].type = DEVT_CON;
    iom.channels[sys_opts.opcon_chan].dev = &opcon_dev;

    /* Disk */
    const int disk_chan = 20;
    iom.channels[disk_chan].type = DEVT_DISK;
    iom.channels[disk_chan].dev = &disk_dev;

    /* Tape */
    iom.channels[sys_opts.tape_chan].type = DEVT_TAPE;
    iom.channels[sys_opts.tape_chan].dev = &tape_dev;

    log_msg(INFO_MSG, "SYS::init", "Once-only initialization complete.\n");
    log_msg(INFO_MSG, "SYS::init", "Activity queue has %d entries.\n", sim_qcount());
}


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
    //log_msg(INFO_MSG, "SYS:fprint_sym", "addr is %012llo; val-ptr is %p, uptr is %p, sw is %#o\n", simh_addr, val, uptr, sw);

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
            if (line != NULL && prior_lineno != lineno && prior_line != line) {
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
        if (sw & SWMASK('S') || (sw & SWMASK('X'))) {
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
        /* See if any other format was requested (but don't bother honoring
           multiple formats */
        if (sw & SWMASK('A')) {
            // already done
        }
        if (sw & SWMASK('L')) {
            // L -> LPW
            fprintf(ofile, " %s", print_lpw(abs_addr));
        } else if (sw & SWMASK('M')) {
            // M -> instr
            char *instr = print_instr(Mem[abs_addr]);
            fprintf(ofile, " %s", instr);
        } else if (sw & SWMASK('P') || (sw & SWMASK('Y'))) {
            // P/Y -> PTW
            char *s = print_ptw(Mem[abs_addr]);
            fprintf(ofile, " %s", s);
        } else if (sw & SWMASK('S') || (sw & SWMASK('X'))) {
            // S/X -> SDW
            char *s = print_sdw(Mem[abs_addr], Mem[abs_addr+1]);
            fprintf(ofile, " %s", s);
        } else if (sw & SWMASK('W')) {
            // W -> DCW
            char *s = print_dcw(abs_addr);
            fprintf(ofile, " %s", s);
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
        } else if (regp && strcmp(regp->name,"TPR") == 0) {
            TPR_t tpr;
            load_TPR(*val, &tpr);
            fprintf(ofile, "[ring %#o, address %#o|%#o (bit %d)]", tpr.TRR, tpr.TSR, tpr.CA, tpr.TBR);
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

inline static int is_octal_digit(char x)
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
    uint last_parsed_seg;
    int last_parsed_offset;
    t_addr last_parsed_addr;

    char *cptr_orig = cptr;
    char debug_strp[1000]; strcpy(debug_strp, cptr);
    *optr = cptr;
    int force_abs = 0;
    int force_seg = 0;

    char *offsetp;
    uint seg = ~0;
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
            sscanf(cptr, "%o", &seg);
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
    if (force_abs || (seg == ~(uint)0 && get_addr_mode() == ABSOLUTE_mode)) {
        last_parsed_mode = ABSOLUTE_mode;
        *optr = cptr;
        // addr = offset;
        last_parsed_seg = ~0;
        last_parsed_offset = offset;
        // last_parsed_addr = addr;
    } else {
        last_parsed_mode = APPEND_mode;
        last_parsed_seg = (seg == ~(uint)0) ? TPR.TSR : seg;
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

void fprint_addr(FILE *stream, DEVICE *dptr, t_addr simh_addr)
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
