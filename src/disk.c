/*
    disk.c -- disk drives

    See manual AN87

*/

#include "hw6180.h"
#include "bitstream.h"

extern iom_t iom;

/*
 * disk_init()
 *
 */

void disk_init()
{
}

/*
 * disk_iom_cmd()
 *
 */

int disk_iom_cmd(chan_devinfo* devinfop)
{
    const char* moi = "DISK::iom_cmd";

    int chan = devinfop->chan;
    int dev_cmd = devinfop->dev_cmd;
    int dev_code = devinfop->dev_code;
    int* majorp = &devinfop->major;
    int* subp = &devinfop->substatus;

    log_msg(DEBUG_MSG, moi, "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n",
        chan, dev_cmd, dev_code);

    devinfop->is_read = 1;  // FIXME
    devinfop->time = -1;

    // Major codes are 4 bits...

    if (chan < 0 || chan >= ARRAY_SIZE(iom.channels)) {
        devinfop->have_status = 1;
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        log_msg(ERR_MSG, moi, "Bad channel %d\n", chan);
        cancel_run(STOP_BUG);
        return 1;
    }

    DEVICE* devp = iom.channels[chan].dev;
    if (devp == NULL || devp->units == NULL) {
        devinfop->have_status = 1;
        *majorp = 05;
        *subp = 2;
        log_msg(ERR_MSG, moi, "Internal error, no device and/or unit for channel 0%o\n", chan);
        cancel_run(STOP_BUG);
        return 1;
    }
    if (dev_code < 0 || dev_code >= devp->numunits) {
        devinfop->have_status = 1;
        *majorp = 05;   // Command Reject
        *subp = 2;      // Invalid Device Code
        log_msg(ERR_MSG, moi, "Bad dev unit-num 0%o (%d decimal)\n", dev_code, dev_code);
        cancel_run(STOP_BUG);
        return 1;
    }
    UNIT* unitp = &devp->units[dev_code];

    // TODO: handle cmd etc for given unit

    log_msg(ERR_MSG, moi, "DISK devices not implemented.\n");
    switch(dev_cmd) {
        // disk_init: idcw.command values:
        //  042 restore access arm
        //  022 read status register
        //  000 request status
        case 040:       // CMD 40 -- Reset Status
            log_msg(NOTIFY_MSG, moi, "Reset Status.\n");
            devinfop->have_status = 1;
            *majorp = 0;
            *subp = 0;
            return 0;
        default: {
            devinfop->have_status = 1;
            *majorp = 05;       // Command reject
            *subp = 1;          // invalid opcode
            log_msg(ERR_MSG, moi, "Unknown command 0%o\n", dev_cmd);
            cancel_run(STOP_BUG);
            return 1;
        }
    }
    return 1;   // not reached
}

// ============================================================================


