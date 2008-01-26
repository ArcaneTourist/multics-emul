/*
    mt.c -- mag tape
        See manual AN87
*/

#include "hw6180.h"
#include "sim_tape.h"

//extern t_uint64 M[];  /* memory */
//extern cpu_ports_t cpu_ports;
// extern scu_t scu;    // BUG: we'll need more than one for max memory.  Unless we can abstract past the physical HW's capabilities
extern iom_t iom;

int mt_iom_cmd(int chan, int dev_cmd, int dev_code, int* majorp, int* subp)
{
    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 055;
        *subp = 2;
        complain_msg("MT::iom_cmd", "Bad channel %d\n", chan);
        return 1;
    }
    DEVICE* devp = iom.devices[chan];
    UNIT* unitp = devp->units;
    if (dev_code < 0 || dev_code >= devp->numunits) {
        // *major = 042;
        // *subp = 2;
        *majorp = 045;
        *subp = 2;
        complain_msg("MT::iom_cmd", "Bad dev unit-num %d\n", dev_code);
        return 1;
    }

    switch(dev_cmd) {
        case 0: {
            *majorp = 040;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            if (sim_tape_bot(unitp)) *subp |= 2;
            if (sim_tape_eot(unitp)) {
                *majorp = 44;
                *subp = 23;
            }
            // todo: switch to having all cmds update status reg?  This would allow setting 047 bootload complete
            return 0;
        }
        default: {
            *majorp = 045;
            *subp = 1;
            complain_msg("MT::iom_cmd", "Unknown command %d\n", dev_cmd);
            return 1;
        }
    }
    return 1;   // not reached
}

t_stat mt_svc(UNIT *up)
{
    debug_msg("MT::service", "not doing anything!\n");
    cancel_run(STOP_WARN);
    return 0;
}
