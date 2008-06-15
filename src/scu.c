/*
    scu.c -- System Controller
    
    See AN70, section 8.

    The term SCU is used throughout this code to match AL39, but the
    device emulated is closer to a Level 68 System Controller (SC) than
    to a Series 60 Level 66 Controller (SC).  The emulated device may
    be closer to a Level 68 4MW SCU than to an Level 68 6000 SCU.
*/

/*
SC and SCU -- System Controller Unit (GB61 and AN70)
    Versions
        SCU -- Series 60 Level66 Controller
        SC -- Level 68 System Controller
    SCUs control access to memory.
        Each SCU owns a certain range of absolute memory.
    SCUs contain clocks.
    SCUS also contain facilites which allow CPUS and IOMs to communicate.
        CPUs or IOMS request access to memory via the SCU.
        CPUs use the cioc instr to talk to IOMs and other CPUs via a SCU.
        IOMs use interrupts to ask a SCU to signal a CPU.
    Other Interesting instructions:
        read system controller reg and set system controller reg (rscr & sscr)
*/

/*
=== Initialization and Booting -- Part 1 -- Operator's view

Booting Instructions (GB61)
    First boot the BCE OS (Bootload command Environment).  See below.
    A config deck is used
    Bootload SCU is the one with a base addr of zero.
    BCE is on a BCE/Multics System tape
    Booted from tape into the system via bootload console

*/

/*
58009906
    When CPU needs to address the SCU (for a write/read data cycle,
    for example), the ETMCM board int the CU of the CPU issues a $INT
    to the SCU.  This signal is sent ... to the SCAMX active port
    control board in the SCU
*/

/*
    // How?  If one of the 32 interrupt cells is set in one of the SCs,
    // our processor will have the interrupt present (XIP) line active.
    // Perhaps faults are flagged in the same way via the SXC system
    // controller command.
*/

// =============================================================================
#if 0
TEMPORARY
        Each SCU owns a certain range of absolute memory.
        CPUs use the cioc instr to talk to IOMs and other CPUs via a SCU.
        IOMs use interrupts to ask a SCU to signal a CPU.
        read system controller reg and set system controller reg (rscr & sscr)
    Bootload SCU is the one with a base addr of zero.
58009906
    When CPU needs to address the SCU (for a write/read data cycle,
    for example), the ETMCM board int the CU of the CPU issues a $INT
    to the SCU.  This signal is sent ... to the SCAMX active port
    control board in the 
-----------------------
    // How?  If one of the 32 interrupt cells is set in one of the SCs,
    // our processor will have the interrupt present (XIP) line active.
    // Perhaps faults are flagged in the same way via the SXC system
    // controller command.
#endif

/*
    *** More (new) notes ***

instr rmcm -- read mem controller mask register
    ... for the selected controller, if the processor has a mask register
    assigned ..
instr smcm -- set  mem controller mask register
    ... for the selected controller, if the processor has a mask register
    assigned, set it to C(AQ)
instr smic
    turn on interrupt cells (any of 0..31)
instr cioc -- connect i/o channel, pg 173
    SC addressed by Y sends a connect signal to the port specified
    by C(Y)33,35
instr rscr & sscr -- Read/Store System Controller Register, pg 170

32 interrupt cells ... XIP
mask info
    8 mask registers
58009906
=============

AM81
Every active device (CPU, IOM) must be able to access all SCUs
Every SCU must have the same active device on the same SCU, so
all SCUs must have the same PORT ENABLE settings
Every active device must have the same SCU on the same port,
so all active devices will have the same config panel settings.
Ports must correspond -- port A on every CPU and IOM must either
be connected tothe same SCU or not connected to any SCU.
IOMs should be on lower-numbered SCU ports than CPUs.
Multics can have 16MW words of memory.
CPUs have 8 ports, a..h.
SCUs have 8 ports, 0..7.

config panel -- level 68 6000 SCU
    store A and store B
        3 position rotary switch: on line, maint, off line
        size: 32k, 64k, 128k, 256k
    exec interrupt mask assignment
        four 10-position rotary switches (a through d): off, 0, .. 7, M
        One switch for each program interrupt register
        Assign mask registers to system ports
        Normally, assing one mask reg to each CPU
        
    ...
config panel -- Level 68 System Controller UNIT (4MW SCU)
    PORT ENABLE
        Eight on/off switches
        Should be on for each port connected to a configured CPU
    LWR Store Size
    Store A, A1, B, B1 (online/offline)
    mask/port assignment
        Two rotary switchs (A & B); set to (off, 0..7)
        See EXEC INTERRUPR on the 6000 SCU
        When booting, one should be set to the port connected to
        the bootload CPU.   The other should be off.

*/

// ============================================================================

#include "hw6180.h"
#include <sys/time.h>

extern cpu_ports_t cpu_ports;
extern scu_t scu;   // BUG: we'll need more than one for max memory.  Unless we can abstract past the physical HW's capabilities

#if 0
void scu_dump()
{
    int pima;
    for (pima = 0; pima < 4; ++pima)
        debug_msg("SCU", "PIMA %d: mask assign = 0%o\n", pima, scu.mask_assign[pima]);
}
#endif

static int scu_hw_arg_check(const char *tag, t_uint64 addr, int port) {
    // Sanity check args
    // Verify that HW could have received signal

    if (port < 0 || port > 7) {
        complain_msg("SCU", "%s: Port %d from sscr is out of range 0..7\n", tag, port);
        cancel_run(STOP_BUG);
        return 1;
    }

#if 0
    return 0;
#else
    // Verify that HW could have received signal
    int cpu_no = cpu_ports.scu_port;    // port-no that rscr instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    // Verify that HW could have received signal
    if (cpu_port < 0) {
        complain_msg("SCU", "Port %d is disabled\n", cpu_no);
        cancel_run(STOP_WARN);
        return 1;
    }
    if (cpu_port > 7) {
        complain_msg("SCU", "Port %d is not enabled (%d).\n", cpu_no, cpu_port);
        cancel_run(STOP_WARN);
        return 1;
    }
    if (cpu_ports.ports[cpu_port] != cpu_no) {
        complain_msg("SCU", "Port %d on CPU is not connected to port %d of SCU.\n", cpu_port, cpu_no);
        cancel_run(STOP_WARN);
        return 1;
    }
    return 0;
#endif
}


int scu_set_mask(t_uint64 addr, int port)
{
    // BUG: addr should determine which SCU is selected
    // Implements part of the sscr instruction

    if (scu_hw_arg_check("setmask", addr, port) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    // find mask reg assigned to processer
    int pima;
    int found = 0;
    int err = 0;
    for (pima = 0; pima < 4; ++pima) {
        // todo: if only one pima per cpu, follwing can be simplified
        if (scu.mask_assign[pima] & (1 << cpu_no)) {
            ++found;
            // AN87, 2-6
            // we use the 32 low bits 
            int maskno = -1;
            int i;
            for (i = 0; i < 4; ++i) {
                // Find PIMA(s) assigned to requested port
                if (scu.mask_assign[i] & (1 << port)) {
                    maskno = i;
                    t_uint64 old = scu.masks[maskno];
                    scu.masks[maskno] = (getbits36(reg_A, 0, 16) << 16) | getbits36(reg_Q, 0, 16);
                    warn_msg("SCU", "PIMA %d has CPU %d assigned; Mask[%d] was 0%o, now 0%o\n",
                        pima, cpu_no, maskno, old, scu.masks[maskno]);
                    cancel_run(STOP_IBKPT);
                }
            }
            if (maskno == -1) {
                // OTOH, see bootload_tape_label.alm -- SSCR will do nothing for unassigned masks
                debug_msg("SCU", "PIMA %d has CPU %d assigned, but no PIMA has port %d assigned\n",
                    pima, cpu_no, port);
                err = 1;
            }
        }
    }
    if (!found) {
        warn_msg("SCU", "No masks assigned to cpu on port %d\n", cpu_no);
        fault_gen(store_fault);
        return 1;
    } else
        if (found > 1)
            warn_msg("SCU", "Multiple masks assigned to cpu on port %d\n", cpu_no);
    if (err) {
        return 1;
    }
    return 0;
}


int scu_set_cpu_mask(t_uint64 addr)
{
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("smcm", addr, 0) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    return scu_set_mask(addr, cpu_no);  // BUG: is this right?
}


int scu_get_mask(t_uint64 addr, int port)
{
    // BUG: addr should determine which SCU is selected
    // Implements part of the sscr instruction

    if (scu_hw_arg_check("getmask", addr, port) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    // Upper 16 bits of each register gets half of the flags
    reg_A = (scu.masks[port] >> 16) << 20;
    reg_Q = (scu.masks[port] & MASKBITS(16)) << 20;
    return 0;
}


int scu_get_calendar(t_uint64 addr)
{
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("getmask", addr, 0) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    // 52 bit clock
    // microseconds since 0000 GMT, Jan 1, 1901 // not 1900 which was a per century exception to leap years

    // sim_os_msec() gives elapsed time in microseconds; But does it return an OS agnositic value?

    // gettimeofday() is POSIX complient
    struct timeval tv;
    struct timezone tz;
    if (gettimeofday(&tv, &tz) != 0) {
        complain_msg("SCU::getcal", "Error from OS gettimeofday\n");
        reg_A = 0;
        reg_Q = 0;
        return 1;
    }
    // returned time is since epoch of 00:00:00 UTC, Jan 1, 1970
    t_uint64 seconds = tv.tv_sec;
    seconds += (t_uint64) 69 * 365 * 24 * 3600;
    t_uint64 now = seconds * 1000 + tv.tv_usec;
    reg_Q = now & MASK36;
    reg_A = (now >> 36) & MASK36;

    return 0;
}


int scu_cioc(t_uint64 addr)
{
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("cioc", addr, 0) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

    t_uint64 word;
    int ret;
    if ((ret = fetch_word(TPR.CA, &word)) != 0) {
        cancel_run(STOP_BUG);
        return ret;
    }
    int port = word & 7;
    debug_msg("SCU::cioc", "Contents of %Lo are: %Lo => port %d\n", addr, word, port);
    // OK ... what's a connect signal (as opposed to an interrupt?
    // A connect signal does the following (AN70, 8-7):
    //  IOM target: connect strobe to IOM
    //  CPU target: connect fault
    
    // todo: check if enabled & not masked
    warn_msg("SCU::cioc", "Partially implemented: Connect sent to port %d => %d\n", port, scu.ports[port]);

    // we only have one IOM, so signal it
    // todo: sanity check port connections
    iom_interrupt();

    return 0;
}

