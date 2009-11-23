/*
    console.c -- operator's console

    See manual AN87.  See also mtb628.

*/

#include <ctype.h>
#include <time.h>
#include "hw6180.h"

extern iom_t iom;

typedef struct s_console_state {
    // Hangs off the device structure
    enum { no_mode, read_mode, write_mode } io_mode;
    // SIMH console library has only putc and getc; the SIMH terminal
    // library has more features including line buffering.
    char buf[81];
    char *tailp;
    char *readp;
    flag_t have_eol;
} con_state_t;

static void check_keyboard(int chan);

// ============================================================================

void console_init()
{
}


// ============================================================================

/*
 * con_check_args()
 *
 * Internal function to do sanity checks
 */

static int con_check_args(const char* moi, int chan, int dev_code, int* majorp, int* subp, DEVICE **devpp, con_state_t **statepp)
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
        con_statep->tailp = con_statep->buf;
        con_statep->readp = con_statep->buf;
        con_statep->have_eol = 0;
    }
    *statepp = con_statep;
    return 0;
}


// ============================================================================


/*
 * con_iom_cmd()
 *
 * Handle a device command.  Invoked by the IOM while processing a PCW
 * or IDCW.
 */

int con_iom_cmd(int chan, int dev_cmd, int dev_code, int* majorp, int* subp)
{
    log_msg(DEBUG_MSG, "CON::iom_cmd", "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n", chan, dev_cmd, dev_code);

    // BUG: Should Major be added to 040? and left shifted 6? Ans: it's 4 bits

    DEVICE* devp;
    con_state_t* con_statep;
    if (con_check_args("CON::iom_cmd", chan, dev_code, majorp, subp, &devp, &con_statep) != 0)
        return 1;

    check_keyboard(chan);

    switch(dev_cmd) {
        case 0: {               // CMD 00 Request status
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Status request cmd received");
            *majorp = 0;
            *subp = 0;
            return 0;
        }
        case 023:               // Read ASCII
            con_statep->io_mode = read_mode;
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Read ASCII command received\n");
            // TODO: discard any buffered chars from SIMH
            con_statep->tailp = con_statep->buf;
            con_statep->readp = con_statep->buf;
            con_statep->have_eol = 0;
            *majorp = 00;
            *subp = 0;
            return 0;
        case 033:               // Write ASCII
            con_statep->io_mode = write_mode;
            log_msg(NOTIFY_MSG, "CON::iom_cmd", "Write ASCII may not be fully functional.\n");
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
            // BUG: No support for Read ID
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

// ============================================================================

/*
 * con_iom_io()
 *
 * Handle an I/O request.  Invoked by the IOM while processing a DDCW.
 */

int con_iom_io(int chan, t_uint64 *wordp, int* majorp, int* subp)
{
    const char *moi = "CON::iom_io";
    log_msg(DEBUG_MSG, "CON::iom_io", "Chan 0%o\n", chan);

    DEVICE* devp;
    con_state_t* con_statep;
    const int dev_code = 0;
    if (con_check_args("CON::iom_cmd", chan, dev_code, majorp, subp, &devp, &con_statep) != 0)
        return 1;

    check_keyboard(chan);

    switch (con_statep->io_mode) {
        case no_mode:
            log_msg(ERR_MSG, "CON::iom_io", "Console is uninitialized\n");
            *majorp = 05;       // 05 -- Command Reject
            *subp = 1;          // 01 Invalid Instruction Code
            return 1;

        case read_mode: {
            // Read keyboard if we don't have an EOL from the operator
            // yet
            if (! con_statep->have_eol) {
                // We won't return anything to the IOM until the operator
                // has finished entering a full line and pressed ENTER.
                log_msg(NOTIFY_MSG, moi, "Starting input loop for channel %d (%#o)\n", chan, chan);
                time_t now = time(NULL);
                while (time(NULL) < now + 30 && ! con_statep->have_eol) {
                    check_keyboard(chan);
                    sleep(1);       // BUG: blocking
                }
                // Impossible to both have EOL and have buffer overflow
                if (con_statep->tailp >= con_statep->buf + sizeof(con_statep->buf)) {
                    *majorp = 03;       // 03 -- Data Alert
                    *subp = 040;        // 10 -- Message length alert
                    log_msg(NOTIFY_MSG, "CON::iom_io", "buffer overflow\n");
                    cancel_run(STOP_IBKPT);
                    return 1;
                }
                if (! con_statep->have_eol) {
                    *majorp = 03;       // 03 -- Data Alert
                    *subp = 010;        // 10 -- Operator distracted (30 sec timeout)
                    log_msg(NOTIFY_MSG, "CON::iom_io", "Operator distracted (30 second timeout\n");
                    cancel_run(STOP_IBKPT);
                }
            }
            // We have an EOL from the operator
            log_msg(NOTIFY_MSG, moi, "Transfer for channel %d (%#o)\n", chan, chan);
            unsigned char c;
            if (con_statep->readp < con_statep->tailp) {
                c = *con_statep->readp++;
            } else
                c = 0;
            *wordp = (t_uint64) c << 27;
            if (c <= 0177 && isprint(c))
                log_msg(NOTIFY_MSG, moi, "Returning word %012llo: %c\n", *wordp, c);
            else
                log_msg(NOTIFY_MSG, moi, "Returning word %012llo: \\%03o\n", *wordp, c);
            int ret;
            if (con_statep->readp == con_statep->tailp) {
                con_statep->readp = con_statep->buf;
                con_statep->tailp = con_statep->buf;
                // con_statep->have_eol = 0;
                log_msg(WARN_MSG, moi, "Entire line now transferred.\n");
                ret = 1;    // BUG: out of band request to return
            } else {
                log_msg(WARN_MSG, moi, "%d chars remain to be transfered.\n", con_statep->tailp - con_statep->readp);
                ret = 0;
            }
            *majorp = 0;
            *subp = 0;
            log_msg(WARN_MSG, moi, "Auto breakpoint.\n");
            cancel_run(STOP_IBKPT);
            return ret;
        }

        case write_mode: {

            char buf[40];   // max four "\###" sequences
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
                    // WARNING: may send junk to the console.
                    // Char 0177 is used by Multics as non-printing padding
                    // (typically after a CRNL as a delay; see syserr_real.pl1).
                    if (c != 0 && c != 0177)
                        err |= sim_putchar(c);
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

// ============================================================================

/*
 * check_keyboard()
 *
 * Check simulated keyboard and transfer input to buffer.
 *
 */

static void check_keyboard(int chan)
{
    const char* moi = "CON::input";

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        log_msg(WARN_MSG, moi, "Bad channel\n");
        return;
    }
    DEVICE* devp = iom.devices[chan];
    if (devp == NULL) {
        log_msg(WARN_MSG, moi, "No device\n");
        return;
    }
    struct s_console_state *con_statep = devp->ctxt;
    if (con_statep == NULL) {
        log_msg(WARN_MSG, moi, "No state\n");
        return;
    }

    for (;;) {
        if (con_statep->tailp >= con_statep->buf + sizeof(con_statep->buf)) {
            log_msg(WARN_MSG, moi, "Buffer full; ignoring keyboard.\n");
            return;
        }
        if (con_statep->have_eol)
            return;
        int c = sim_poll_kbd();
        if (c == SCPE_OK)
            return; // no input
        if (c == SCPE_STOP) {
            log_msg(NOTIFY_MSG, moi, "Got <sim stop>\n");
            return; // User typed ^E to stop simulation
        }
        if (c < SCPE_KFLAG) {
            log_msg(NOTIFY_MSG, moi, "Bad char\n");
            return; // Should be impossible
        }

        c -= SCPE_KFLAG;    // translate to ascii
        if (isprint(c))
            log_msg(NOTIFY_MSG, moi, "Got char '%c'\n", c);
        else
            log_msg(NOTIFY_MSG, moi, "Got char '\\%03o'\n", c);

        // BUG: We don't allow user to set editing characters
        if (c == '\177' || c == '\010') {
            if (con_statep->tailp > con_statep->buf)
                -- con_statep->tailp;
        } else if (c == '\014') {
            sim_putchar('\r');
            sim_putchar('\n');
            for (const char *p = con_statep->buf; p < con_statep->tailp; ++p)
                sim_putchar(*p);
        } else if (c == '\015') {
            con_statep->have_eol = 1;
            log_msg(NOTIFY_MSG, moi, "Got EOL for channel %d (%#o); con_statep is %p\n", chan, chan, con_statep);
            return;
        } else
            *con_statep->tailp++ = c;
    }
}
