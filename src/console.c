/*
    console.c -- operator's console
        See manual AN87
*/

#include "hw6180.h"
#include <ctype.h>

extern iom_t iom;

typedef struct s_console_state {
    // Hangs off the device structure
    enum { no_mode, read_mode, write_mode } io_mode;
} con_state_t;


void console_init()
{
}


static int  con_check_args(const char* moi, int chan, int dev_code, int* majorp, int* subp, DEVICE **devpp, con_state_t **statepp)
{

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 1;
        log_msg(ERR_MSG, moi, "Bad channel %d\n", chan);
        return 1;
    }

    *devpp = iom.devices[chan];
    DEVICE *devp = *devpp;
    if (devpp == NULL) {
        *majorp = 05;
        *subp = 1;
        log_msg(ERR_MSG, moi, "Internal error, no device for channel 0%o\n", chan);
        return 1;
    }
    if (dev_code != 0) {
        // Consoles don't have units
        *majorp = 05;
        *subp = 1;
        log_msg(ERR_MSG, moi, "Bad dev unit-num 0%o (%d decimal)\n", dev_code, dev_code);
        return 1;
    }

    struct s_console_state *con_statep = devp->ctxt;
    if (con_statep == NULL) {
        if ((devp->ctxt = malloc(sizeof(struct s_console_state))) == NULL) {
            log_msg(ERR_MSG, moi, "Internal error, malloc failed.\n");
            return 1;
        }
        con_statep = devp->ctxt;
        con_statep->io_mode = no_mode;
    }
    *statepp = con_statep;
    return 0;
}


int con_iom_cmd(int chan, int dev_cmd, int dev_code, int* majorp, int* subp)
{
    log_msg(DEBUG_MSG, "CON::iom_cmd", "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n", chan, dev_cmd, dev_code);

    // BUG: Should Major be added to 040? and left shifted 6? Ans: it's 4 bits

    DEVICE* devp;
    con_state_t* con_statep;
    if (con_check_args("CON::iom_cmd", chan, dev_code, majorp, subp, &devp, &con_statep) != 0)
        return 1;

    switch(dev_cmd) {
        case 0: {               // CMD 00 Request status
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Status request cmd received");
            *majorp = 0;
            *subp = 0;
            return 0;
        }
        case 023:               // Read ASCII
            con_statep->io_mode = read_mode;
            log_msg(ERR_MSG, "CON::iom_cmd", "Read ASCII unimplemented\n");
            cancel_run(STOP_BUG);
            *majorp = 00;
            *subp = 0;
            return 0;
        case 033:               // Write ASCII
            con_statep->io_mode = write_mode;
            log_msg(ERR_MSG, "CON::iom_cmd", "Write ASCII semi-implemented\n");
            cancel_run(STOP_WARN);
            *majorp = 00;
            *subp = 0;
            return 0;
        case 040:               // Reset
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Reset cmd received");
            con_statep->io_mode = no_mode;
            *majorp = 0;
            *subp = 0;
            return 0;
        case 051:               // Write Alert -- Ring Bell
            // AN70-1 says only console channels respond to this command
            out_msg("CONSOLE: ALERT\n");
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Write Alert cmd received\n");
            *majorp = 0;
            *subp = 0;
            return 0;
        case 057:               // Read ID (according to AN70-1)
            log_msg(ERR_MSG, "CON::iom_cmd", "Read ID unimplemented\n");
            *majorp = 05;
            *subp = 1;
            return 1;
        default: {
            *majorp = 05;   // command reject
            *subp = 1;      // invalid instruction code
            log_msg(ERR_MSG, "CON::iom_cmd", "Unknown command 0%o\n", dev_cmd);
            cancel_run(STOP_BUG);
            return 1;
        }
    }
    return 1;   // not reached
}


int con_iom_io(int chan, t_uint64 *wordp, int* majorp, int* subp)
{
    log_msg(DEBUG_MSG, "CON::iom_io", "Chan 0%o\n", chan);

    DEVICE* devp;
    con_state_t* con_statep;
    const int dev_code = 0;
    if (con_check_args("CON::iom_cmd", chan, dev_code, majorp, subp, &devp, &con_statep) != 0)
        return 1;

    switch (con_statep->io_mode) {
        case no_mode:
            log_msg(ERR_MSG, "CON::iom_io", "Console is uninitialized\n");
            *majorp = 05;
            *subp = 1;
            return 1;

        case read_mode:
            // int c = sim_poll_char();
            log_msg(ERR_MSG, "CON::iom_io", "Read mode unimplemented\n");
            cancel_run(STOP_BUG);
            *majorp = 05;
            *subp = 1;
            return 1;

        case write_mode: {

            // log_msg(DEBUG_MSG, "CON::iom_io", "Write: word = %012llo\n", *wordp);

            char buf[80];
            *buf = 0;
            t_uint64 word = *wordp;
            if ((word >> 36) != 0) {
                log_msg(ERR_MSG, "CON::iom_io", "Word %012llo has more than 36 bits.\n", word);
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
#if 1
    // BUG: not sending control junk
                    if (c != 0)
                        err |= sim_putchar(c);
#endif
                }
            }
            if (err)
                log_msg(WARN_MSG, "CON::iom_io", "Error writing to CONSOLE\n");
            out_msg("CONSOLE: %s\n", buf);

            *majorp = 0;
            *subp = 0;

            return 0;
        }

        default:
            log_msg(ERR_MSG, "CON::iom_io", "Console is in unknown mode %d\n", con_statep->io_mode);
            *majorp = 05;
            *subp = 1;
            return 1;
    }
}
