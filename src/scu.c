/*
    scu.c -- System Controller
    
    See AN70, section 8 and GB61.

    There were a few variations of SCs and SCUs:
        SCU -- Series 60 Level66 Controller
        SC -- Level 68 System Controller
        4MW SCU -- A later version of the L68 SC

    SCUs control access to memory.
        Each SCU owns a certain range of absolute memory.
        This emulator allows the CPU to access memory directly however.
    SCUs contain clocks.
    SCUS also contain facilites which allow CPUS and IOMs to communicate.
        CPUs or IOMS request access to memory via the SCU.
        CPUs use the cioc instr to talk to IOMs and other CPUs via a SCU.
        IOMs use interrupts to ask a SCU to signal a CPU.
    Other Interesting instructions:
        read system controller reg and set system controller reg (rscr & sscr)
*/


/*
    The following comment is probably wrong:    
        The term SCU is used throughout this code to match AL39, but the
        device emulated is closer to a Level 68 System Controller (SC) than
        to a Series 60 Level 66 Controller (SC).  The emulated device may
        be closer to a Level 68 4MW SCU than to an Level 68 6000 SCU.

    BUG/TODO: The above is probably wrong; we explicitly report an
    ID code for SCU via rscr 000001x.  It wouldn't hurt to review
    all the code to make sure we never act like a SC instead of an
    SCU.
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
    for (pima = 0; pima < array_size(scu.mask_assign); ++pima)
        log_msg(DEBUG_MSG, "SCU", "PIMA %d: mask assign = 0%o\n", pima, scu.mask_assign[pima]);
}
#endif

static int scu_hw_arg_check(const char *tag, t_uint64 addr, int port)
{
    // Sanity check args
    // Verify that HW could have received signal

    if (port < 0 || port > 7) {
        log_msg(ERR_MSG, "SCU", "%s: Port %d from sscr is out of range 0..7\n", tag, port);
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
        log_msg(ERR_MSG, "SCU", "Port %d is disabled\n", cpu_no);
        cancel_run(STOP_WARN);
        return 1;
    }
    if (cpu_port > 7) {
        log_msg(ERR_MSG, "SCU", "Port %d is not enabled (%d).\n", cpu_no, cpu_port);
        cancel_run(STOP_WARN);
        return 1;
    }
    if (cpu_ports.ports[cpu_port] != cpu_no) {
        log_msg(ERR_MSG, "SCU", "Port %d on CPU is not connected to port %d of SCU.\n", cpu_port, cpu_no);
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

    // Find mask reg assigned to processer
    int pima = 0;
    int found = 0;
    for (int p = 0; p < ARRAY_SIZE(scu.eima_data); ++p) {
        if (! scu.eima_data[p].avail)
            continue;
        if (! scu.eima_data[p].assigned)
            continue;
        if (scu.eima_data[p].port == cpu_no) {
            pima = p;
            log_msg(NOTIFY_MSG, "SCU", "MASK %d is assigned to cpu %d\n", p, cpu_no);
            ++found;
        }
    }
    if (!found) {
        log_msg(WARN_MSG, "SCU", "No masks assigned to cpu on port %d\n", cpu_no);
        fault_gen(store_fault);
        return 1;
    } else
        if (found > 1) {
            log_msg(WARN_MSG, "SCU", "Multiple masks assigned to cpu on port %d\n", cpu_no);
            cancel_run(STOP_WARN);
        } else if (pima > 1) {
            log_msg(ERR_MSG, "SCU", "Cannot write to masks other than zero and one: %d\n", pima);
            cancel_run(STOP_BUG);
            return 1;
        } else {
            // See AN87
            for (int pima = 0; pima < 2; ++pima) {
                char name = (pima == 0) ? 'A' : 'B';
                scu.eima_data[pima].raw = (pima == 0) ? getbits36(reg_A, 0, 9) : getbits36(reg_Q, 0, 9);
                int found = 0;
                for (int p = 0; p < 8; ++p)
                    if (((1<<(8-p) & scu.eima_data[pima].raw)) != 0) {
                        ++ found;
                        scu.eima_data[pima].assigned = 1;
                        scu.eima_data[pima].port = p;
                        log_msg(NOTIFY_MSG, "SCU::set-mask", "Assigning port %d to MASK %c.\n", p, name);
                    }
                if ((scu.eima_data[pima].raw & 1) != 0) {
                    scu.eima_data[pima].assigned = 0;
                    log_msg(NOTIFY_MSG, "SCU::set-mask", "Unassigning MASK %c.\n", name);
                    ++ found;
                }
                if (found != 1) {
                    log_msg(WARN_MSG, "SCU::set-mask", "%d ports enabled for MASK %c: %#o\n", found, name, scu.eima_data[pima].raw);
                    if (found != 0 || scu.eima_data[pima].raw != 0)
                        cancel_run(STOP_WARN);
                }
            }
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


int scu_get_mode_register(t_uint64 addr)
{
    // Implements part of the rscr instruction
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("get-mode-register", addr, 0) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU


    // See scr.incl.pl1 and AN87 page 2-2

    // Note that rscr 00001X can only report an SC with a memory sizes of up
    // to 256 K-words, but can report an SCU with up to 4MW.  So, we identify
    // ourselves as an SCU.

    reg_A = 0;  // first 50 bits are padding
    reg_Q = 0;
    reg_Q |= setbits36(reg_Q, 50-36, 4, 2); // id for a 4MW SCU (level 66 SCU)
    /*
        remaining bits are only for T&D test and diagnostics
    */
        // reg_Q |= setbits36(reg_Q, 54-36, 2, 0);  // TS strobe normal timing
        // reg_Q |= setbits36(reg_Q, 64-36, 2, 0);  // both 00b and 10b mean normal voltage
        // reg_Q |= setbits36(reg_Q, 70-36, 1, 0);  // SGR accepted

    return 0;
}


int scu_get_config_switches(t_uint64 addr)
{
    // Implements part of the rscr instruction
    // Returns info appropriate to a 4MW SCU
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("get-mode-register", addr, 0) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU


    // See scr.incl.pl1 
    reg_A = 0;
    // interrupt mask A port assignment
#if 0
    if (!scu.eima_data[0].avail)
        reg_A |= setbits36(reg_A, 0, 9, 0);
    else if (scu.eima_data[0].assigned)
        reg_A |= setbits36(reg_A, 0, 9, 1);
    else
        reg_A |= setbits36(reg_A, 0, 9, 1<<(8 - scu.eima_data[0].port));
#else
    reg_A |= setbits36(reg_A, 0, 9, scu.eima_data[0].raw);
#endif
    //reg_A |= setbits36(reg_A, 9, 3, 7);   // size of lower store -- 2^(7+5) == 4096 K-words
    reg_A |= setbits36(reg_A, 9, 3, 5); // size of lower store -- 2^(5+5) == 1024 K-words
    reg_A |= setbits36(reg_A, 12, 4, 017);  // all four stores online
    reg_A |= setbits36(reg_A, 16, 4, cpu_no);   // requester's port #
    reg_A |= setbits36(reg_A, 21, 1, 1);    // programmable; BUG
    reg_A |= setbits36(reg_A, 22, 1, 0);    // non-existent address logic enabled
    reg_A |= setbits36(reg_A, 23, 7, 0);    // nea size
    reg_A |= setbits36(reg_A, 30, 1, 1);    // internally interlaced
    reg_A |= setbits36(reg_A, 31, 1, 0);    // store B is lower?
    for (int i = 0; i < 4; ++ i) {
        int enabled = scu.ports[i] >= 0;
        reg_A |= setbits36(reg_A, 32+i, 1, enabled);    // enable masks for ports 0-3
    }

    reg_Q = 0;
#if 0
    // interrupt mask B port assignment; BUG
    if (!scu.eima_data[0].avail)
        reg_Q |= setbits36(reg_Q, 0, 9, 0);
    else if (scu.eima_data[0].assigned)
        reg_Q |= setbits36(reg_Q, 0, 9, 1);
    else
        reg_Q |= setbits36(reg_Q, 0, 9, 1<<(8 - scu.eima_data[0].port));
#else
    reg_Q |= setbits36(reg_Q, 0, 9, scu.eima_data[0].raw);
#endif
    reg_Q |= setbits36(reg_Q, 57-36, 7, 0); // cyclic port priority switches; BUG
    for (int i = 0; i < 4; ++ i) {
        int enabled = scu.ports[i+4] >= 0;
        reg_Q |= setbits36(reg_Q, 68-36+i, 1, enabled); // enable masks for ports 4-7
    }

    return 0;
}


int scu_get_mask(t_uint64 addr, int port)
{
    // BUG: addr should determine which SCU is selected
    // Implements part of the rscr instruction

    if (scu_hw_arg_check("getmask", addr, port) != 0)
        return 1;
    int cpu_no = cpu_ports.scu_port;    // port-no that instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU


    // Find which of the 4 masks are assigned to the specified port
    int pima = 0;
    int found = 0;
    for (int p = 0; p < ARRAY_SIZE(scu.eima_data); ++p) {
        if (! scu.eima_data[pima].avail)
            continue;
        if (! scu.eima_data[pima].assigned)
            continue;
        if (scu.eima_data[pima].port == port) {
            pima = p;
            ++found;
        }
    }

    // BUG: we need to return two masks, each of which is assigned to some port
    if (pima % 2 == 1)
        -- pima;
    reg_A = setbits36(0, 0, 9, scu.eima_data[pima].raw);
    reg_Q = setbits36(0, 0, 9, scu.eima_data[pima+1].raw);
    return 0;
}


int scu_get_calendar(t_uint64 addr)
{
    // BUG: addr should determine which SCU is selected

    if (scu_hw_arg_check("get-calendar", addr, 0) != 0)
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
        log_msg(ERR_MSG, "SCU::getcal", "Error from OS gettimeofday\n");
        reg_A = 0;
        reg_Q = 0;
        return 1;
    }
    t_uint64 seconds = tv.tv_sec;
    seconds += (t_uint64) 69 * 365 * 24 * 3600; // returned time is since epoch of 00:00:00 UTC, Jan 1, 1970
    t_uint64 now = seconds * 1000000 + tv.tv_usec;
    reg_Q = now & MASK36;
    reg_A = (now >> 36) & MASK36;
    calendar_a = reg_A; // only for debugging
    calendar_q = reg_Q; // only for debugging
    // log_msg(INFO_MSG, "calendar", "UNIX time %ld; multics time {%012llo, %012llo}\n", tv.tv_sec, reg_A, reg_Q);

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
    log_msg(DEBUG_MSG, "SCU::cioc", "Contents of %llo are: %llo => port %d\n", addr, word, port);
    // OK ... what's a connect signal (as opposed to an interrupt?
    // A connect signal does the following (AN70, 8-7):
    //  IOM target: connect strobe to IOM
    //  CPU target: connect fault
    
    // todo: check if enabled & not masked

    static int n_cioc = 0;
    {
        //static int n_cioc = 0;
        log_msg(NOTIFY_MSG, "SCU::cioc", "CIOC # %d\n", ++ n_cioc);
        if (n_cioc >= 306) {        // BUG: temp hack to increase debug level
            extern DEVICE cpu_dev;
            ++ opt_debug; ++ cpu_dev.dctrl;
        }
    }
    log_msg(DEBUG_MSG, "SCU::cioc", "Connect sent to port %d => %d\n", port, scu.ports[port]);

    // we only have one IOM, so signal it
    // todo: sanity check port connections
    iom_interrupt();

    if (n_cioc >= 306) {
        extern DEVICE cpu_dev;
        -- opt_debug; -- cpu_dev.dctrl;
    }

    return 0;
}

