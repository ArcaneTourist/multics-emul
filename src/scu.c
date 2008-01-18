/*
    scu.c -- System Controller
    
    The term SCU is used throughout this code to match AL39, but the
    device emulated is closer to a Level 68 System Controller (SC) than
    to a Series 60 Level 66 Controller (SC).  The emulated device may
    be closer to a Level 68 4MW SCU than to an LEvel 68 6000 SCU.
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

extern cpu_ports_t cpu_ports;
extern scu_t scu;   // BUG: we'll need more than one for max memory.  Unless we can abstract past the physical HW's capabilities

int scu_set_mask(t_uint64 addr, int port)
{
    // BUG: addr should determine which SCU is selected
    // Implements part of the sscr instruction

    if (port < 0 || port > 7) {
        complain_msg("SCU", "Port %d from sscr is out of range 0..7\n", port);
        cancel_run(STOP_BUG);
        return 1;
    }

    int cpu_no = cpu_ports.scu_port;    // port-no that rscr instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

#if 1
    // Verify that HW could have received signal
    if (cpu_port < 0) {
        complain_msg("SCU", "Port %d is disabled\n", cpu_no);
        cancel_run(STOP_WARN);
        return 0;
    }
    if (cpu_port > 7) {
        complain_msg("SCU", "Port %d is not enabled (%d).\n", cpu_no, cpu_port);
        cancel_run(STOP_WARN);
        return 0;
    }
    if (cpu_ports.ports[cpu_port] != cpu_no) {
        complain_msg("SCU", "Port %d on CPU is not connected to port %d of SCU.\n", cpu_port, cpu_no);
        cancel_run(STOP_WARN);
        return 0;
    }
#endif

    // find mask reg assigned to processer
    int pima;
    int found = 0;
    for (pima = 0; pima < 4; ++pima) {
        if (scu.mask_assign[pima] & (1 << cpu_no)) {
            found = 1;
            scu.masks[port] = reg_Q;    // BUG: wrong; see AN87
            // todo: if only one interrput per cpu, we could break here
        }
    }
    if (!found) {
        debug_msg("SCU", "No masks assgined to cpu on port %d\n", cpu_no);
        fault_gen(store_fault);
        return 1;
    }
    return 0;
}

int scu_get_mask(t_uint64 addr, int port)
{
    // BUG: addr should determine which SCU is selected
    // Implements part of the sscr instruction

    if (port < 0 || port > 7) {
        complain_msg("SCU", "Port %d from sscr is out of range 0..7\n", port);
        cancel_run(STOP_BUG);
        return 1;
    }

    int cpu_no = cpu_ports.scu_port;    // port-no that rscr instr came in on
    int cpu_port = scu.ports[cpu_no];   // which port on the CPU connects to SCU

#if 1
    // Verify that HW could have received signal
    if (cpu_port < 0) {
        complain_msg("SCU", "Port %d is disabled\n", cpu_no);
        cancel_run(STOP_WARN);
        return 0;
    }
    if (cpu_port > 7) {
        complain_msg("SCU", "Port %d is not enabled (%d).\n", cpu_no, cpu_port);
        cancel_run(STOP_WARN);
        return 0;
    }
    if (cpu_ports.ports[cpu_port] != cpu_no) {
        complain_msg("SCU", "Port %d on CPU is not connected to port %d of SCU.\n", cpu_port, cpu_no);
        cancel_run(STOP_WARN);
        return 0;
    }
#endif

    // BUG: reg_AQ = ... masks[port];
    debug_msg("SCU", "get mask unimplmented\n", cpu_no);
    cancel_run(STOP_BUG);
    return 0;
}
