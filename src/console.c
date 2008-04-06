/*
    console.c -- operator's console
        See manual AN87
*/

#include "hw6180.h"
#include <ctype.h>

extern iom_t iom;

int con_iom_cmd(int chan, int dev_cmd, int dev_code, int* majorp, int* subp)
{
    debug_msg("CON::iom_cmd", "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n", chan, dev_cmd, dev_code);

    // BUG: Should Major be added to 040? and left shifted 6? Ans: it's 4 bits

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 1;
        complain_msg("CON::iom_cmd", "Bad channel %d\n", chan);
        return 1;
    }

#if 0
    DEVICE* devp = iom.devices[chan];
    if (devp == NULL || devp->units == NULL) {
        *majorp = 05;
        *subp = 2;
        complain_msg("CON::iom_cmd", "Internal error, no device and/or unit for channel 0%o\n", chan);
        return 1;
    }
    UNIT* unitp = devp->units;
    if (dev_code < 0 || dev_code >= devp->numunits) {
        // *major = 042;
        // *subp = 2;
        *majorp = 05;
        *subp = 2;
        complain_msg("CON::iom_cmd", "Bad dev unit-num 0%o (%d decimal)\n", dev_code, dev_code);
        return 1;
    }
    // BUG: dev_code unused
#endif

    switch(dev_cmd) {
        case 0: {               // CMD 00 Request status
            *majorp = 0;
            *subp = 0;
            return 0;
        }
        case 023:               // Read ASCII
            complain_msg("CON::iom_cmd", "Read ASCII unimplemented\n");
            cancel_run(STOP_WARN);
            *majorp = 00;
            *subp = 0;
            return 0;
        case 033:               // Write ASCII
            complain_msg("CON::iom_cmd", "Write ASCII semi-implemented\n");
            cancel_run(STOP_WARN);
            *majorp = 00;
            *subp = 0;
            return 0;
        case 051:               // Write Alert -- Ring Bell
            out_msg("CONSOLE: ALERT\n");
            *majorp = 0;
            *subp = 0;
            return 0;
        case 040:               // Reset
            *majorp = 0;
            *subp = 0;
            return 0;
        default: {
            *majorp = 05;
            *subp = 1;
            complain_msg("CON::iom_cmd", "Unknown command 0%o\n", dev_cmd);
            return 1;
        }
    }
    return 1;   // not reached
}

int con_iom_io(int chan, t_uint64 *wordp, int* majorp, int* subp)
{
    debug_msg("CON::iom_io", "Chan 0%o\n", chan);
    debug_msg("CON::iom_io", "Word = %012Lo\n", *wordp);

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        complain_msg("CON::iom_io", "Bad channel %d\n", chan);
        return 1;
    }

    // BUG: Need to know whether to read or write

    char buf[80];
    *buf = 0;
    t_uint64 word = *wordp;
    if ((word >> 36) != 0) {
        complain_msg("CON::iom_io", "Word %012Lo has more than 36 bits.\n", word);
        cancel_run(STOP_BUG);
        word &= MASK36;
    }
    int err = 0;
    for (int i = 0; i < 4; ++i) {
        uint c = word >> 27;
        word = (word << 9) & MASKBITS(36);
        if (c <= 0177 && isprint(c)) {
            sprintf(buf+strlen(buf), "%c", c);
            err |= sim_putchar(c);
        } else {
            sprintf(buf+strlen(buf), "\\%03o", c);
            if (c != 0)
                err |= sim_putchar(c);
        }
    }
    if (err)
        warn_msg("CON::iom_io", "Error writing to CONSOLE\n");
    out_msg("CONSOLE: %s\n", buf);

    *majorp = 0;
    *subp = 0;

    return 0;
}
