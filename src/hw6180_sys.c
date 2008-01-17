#include "hw6180.h"

extern DEVICE cpu_dev;
extern DEVICE dsk_dev;
extern UNIT cpu_unit;
extern REG cpu_reg[];
extern DEVICE sio_dev;
extern DEVICE ptr_dev;
extern DEVICE ptp_dev;
extern DEVICE lpt_dev;
extern t_uint64 M[];

extern UNIT TR_clk_unit;
extern switches_t switches;
extern cpu_ports_t cpu_ports;
extern scu_t scu;

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
#if 0
    &sio_dev,
    &ptr_dev,
    &ptp_dev,
    &dsk_dev,
#endif
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


static void hw6180_init(void)
{
    debug_msg("SYS::init", "Once-only initialization running.\n");
    // todo: sim_brk_types = ...
    // todo: sim_brk_dflt = ...

    // Hardware config -- should be based on config cards!

    // CPU port 'a' connected to SCU port '7'
    memset(&cpu_ports, 0, sizeof(cpu_ports));
    cpu_ports.scu_port = 7; // arbitrary
    cpu_ports.ports[0] = cpu_ports.scu_port;

    memset(&switches, 0, sizeof(switches));
    // multics uses same vector for interrupts & faults?
    // OTOH, AN87, 1-41 claims faults are at 100o ((flt_base=1)<<5)
    switches.FLT_BASE = 0;  // multics uses same vector for interrupts & faults?

    // Only one SCU, connected as described above
    memset(&scu, 0, sizeof(scu));
    scu.ports[cpu_ports.scu_port] = 0;  // port '7' connected to CPU port 'a'
    
    // GB61, page 9-1
    scu.mask_assign[0] = 1 << cpu_ports.scu_port;
}

extern t_stat fprint_sym (FILE *ofile, t_addr addr, t_value *val, UNIT *uptr, int32 sw)
{
    abort();
}

extern t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
    abort();
}

activate_timer()
{
    uint32 t;
    debug_msg("SYS::clock", "TR is %Ld 0%Lo.\n", reg_TR, reg_TR);
    if (bit27_is_neg(reg_TR)) { // BUG: 27bit reg -- should check bit 26?
        if ((t = sim_is_active(&TR_clk_unit)) != 0)
            debug_msg("SYS::clock", "TR cancelled with %d time units left.\n", t);
        else
            debug_msg("SYS::clock", "TR loaded with negative value, but it was alread stopped.\n", t);
        sim_cancel(&TR_clk_unit);
        return;
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
}


t_stat clk_svc(UNIT *up)
{
    // only valid for TR
    (void) sim_rtcn_calb (CLK_TR_HZ, TR_CLK);   // calibrate clock
    uint32 t = sim_is_active(&TR_clk_unit);
    printf("SYS::clock::service", "TR has %d time units left\n");
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
