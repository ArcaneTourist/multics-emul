/*
    mt.c -- mag tape
        See manual AN87
*/

#include "hw6180.h"
#include "sim_tape.h"
#include "bitstream.h"

extern iom_t iom;

static const char *simh_tape_msg(int code); // hack
static const size_t bufsz = 4096 * 1024;
static struct s_tape_state {
    // BUG: this should hang off of UNIT structure (not that the UNIT
    // structure contains a pointer...)
    // BUG: An array index by channel doesn't allow multiple tapes per channel
    enum { no_mode, read_mode, write_mode } io_mode;
    uint8 *bufp;
    bitstream_t *bitsp;
} tape_state[ARRAY_SIZE(iom.channels)];

void mt_init()
{
    memset(tape_state, 0, sizeof(tape_state));
}

/*
 * mt_iom_cmd()
 *
 */

int mt_iom_cmd(chan_devinfo* devinfop)
{
    int chan = devinfop->chan;
    int dev_cmd = devinfop->dev_cmd;
    int dev_code = devinfop->dev_code;
    int* majorp = &devinfop->major;
    int* subp = &devinfop->substatus;

    log_msg(DEBUG_MSG, "MT::iom_cmd", "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n", chan, dev_cmd, dev_code);

    devinfop->is_read = 1;
    devinfop->time = -1;

    // Major codes are 4 bits...

    if (chan < 0 || chan >= ARRAY_SIZE(iom.channels)) {
        devinfop->have_status = 1;
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_cmd", "Bad channel %d\n", chan);
        cancel_run(STOP_BUG);
        return 1;
    }

    DEVICE* devp = iom.channels[chan].dev;
    if (devp == NULL || devp->units == NULL) {
        devinfop->have_status = 1;
        *majorp = 05;
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_cmd", "Internal error, no device and/or unit for channel 0%o\n", chan);
        cancel_run(STOP_BUG);
        return 1;
    }
    if (dev_code < 0 || dev_code >= devp->numunits) {
        devinfop->have_status = 1;
        *majorp = 05;   // Command Reject
        *subp = 2;      // Invalid Device Code
        log_msg(ERR_MSG, "MT::iom_cmd", "Bad dev unit-num 0%o (%d decimal)\n", dev_code, dev_code);
        cancel_run(STOP_BUG);
        return 1;
    }
    UNIT* unitp = &devp->units[dev_code];

    struct s_tape_state *tape_statep = &tape_state[chan];

    switch(dev_cmd) {
        case 0: {               // CMD 00 Request status
            devinfop->have_status = 1;
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            if (sim_tape_bot(unitp)) *subp |= 2;
            if (sim_tape_eot(unitp)) {
                *majorp = 044;  // BUG? should be 3?
                *subp = 023;    // BUG: should be 040?
            }
            // todo: switch to having all cmds update status reg?  This would allow setting 047 bootload complete
            log_msg(INFO_MSG, "MT::iom_cmd", "Request status is %02o,%02o.\n",
                *majorp, *subp);
            return 0;
        }
        case 5: {               // CMD 05 -- Read Binary Record
            // We read the record into the tape controllers memory;
            // IOM can then retrieve the data via DCWs
            if (devp->numunits <= 1) {
                devinfop->chan_data = 0;
            } else {
                // T&D tape seems to want to see non-zero residue from tape
                // read.
                // Bootload_tape_label.alm indicates that after a read binary
                // operation, the field is interpreted as the device number
                // and that a device number of zero is legal.
                // AN70, page 2-1 says that when the tape is read in
                // native mode via an IOM or IOCC, the tape drive number
                // in the IDCW will be zero.  A non-zero tape drive number
                // in the IDCW indicates that BOS is being used to simulate
                // an IOM.
                // Perhaps the T&D tape requires BOS (or the BCE replacement
                // for BOS) ?
                // Yes, the below results in zero if we only have one unit,
                // but provides a place for this long comment. :-)
                devinfop->chan_data = unitp - devp->units;
            }
            if (tape_statep->bufp == NULL)
                if ((tape_statep->bufp = malloc(bufsz)) == NULL) {
                    log_msg(ERR_MSG, "MT::iom_cmd", "Malloc error\n");
                    devinfop->have_status = 1;
                    *majorp = 012;  // BUG: arbitrary error code; config switch
                    *subp = 1;
                    return 1;
                }
            t_mtrlnt tbc = 0;
            int ret;
            if ((ret = sim_tape_rdrecf(unitp, tape_statep->bufp, &tbc, bufsz)) != 0) {
                if (ret == MTSE_TMK || ret == MTSE_EOM) {
                    log_msg(NOTIFY_MSG, "MT::iom_cmd", "EOF: %s\n", simh_tape_msg(ret));
                    devinfop->have_status = 1;
                    *majorp = 044;  // EOF category
                    *subp = 023;    // EOF file mark
                    if (tbc != 0) {
                        log_msg(ERR_MSG, "MT::iom_cmd", "Read %d bytes with EOF.\n", tbc);
                        cancel_run(STOP_WARN);
                    }
                    // cancel_run(STOP_IBKPT);
                    return 0;
                } else {
                    devinfop->have_status = 1;
                    log_msg(ERR_MSG, "MT::iom_cmd", "Cannot read tape: %d - %s\n", ret, simh_tape_msg(ret));
                    log_msg(ERR_MSG, "MT::iom_cmd", "Returning arbitrary error code\n");
                    *majorp = 010;  // BUG: arbitrary error code; config switch
                    *subp = 1;
                    return 1;
                }
            }
            tape_statep->bitsp = bitstm_new(tape_statep->bufp, tbc);
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            tape_statep->io_mode = read_mode;
            devinfop->time = sys_opts.mt_times.read;
            if (devinfop->time < 0) {
                log_msg(INFO_MSG, "MT::iom_cmd", "Read %d bytes from simulated tape\n", (int) tbc);
                devinfop->have_status = 1;
            } else
                log_msg(INFO_MSG, "MT::iom_cmd", "Queued read of %d bytes from tape.\n", (int) tbc);
            return 0;
        }
        case 040:               // CMD 040 -- Reset Status
            devinfop->have_status = 1;
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            log_msg(INFO_MSG, "MT::iom_cmd", "Reset status is %02o,%02o.\n",
                *majorp, *subp);
            return 0;
        case 046: {             // BSR
            // BUG: Do we need to clear the buffer?
            // BUG: We don't check the channel data for a count
            t_mtrlnt tbc;
            int ret;
            if ((ret = sim_tape_sprecr(unitp, &tbc)) == 0) {
                log_msg(NOTIFY_MSG, "MT::iom_cmd", "Backspace one record\n");
                devinfop->have_status = 1;  // TODO: queue
                *majorp = 0;
                *subp = 0;
                if (sim_tape_wrp(unitp)) *subp |= 1;
            } else {
                log_msg(ERR_MSG, "MT::iom_cmd", "Cannot backspace record: %d - %s\n", ret, simh_tape_msg(ret));
                devinfop->have_status = 1;
                if (ret == MTSE_BOT) {
                    *majorp = 05;
                    *subp = 010;
                } else {
                    log_msg(ERR_MSG, "MT::iom_cmd", "Returning arbitrary error code\n");
                    *majorp = 010;  // BUG: arbitrary error code; config switch
                    *subp = 1;
                }
                return 1;
            }
            return 0;
        }
        case 051:               // CMD 051 -- Reset Device Status
            devinfop->have_status = 1;
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            log_msg(INFO_MSG, "MT::iom_cmd", "Reset device status is %02o,%02o.\n",
                *majorp, *subp);
            return 0;
        default: {
            *majorp = 05;
            *subp = 1;
            log_msg(ERR_MSG, "MT::iom_cmd", "Unknown command 0%o\n", dev_cmd);
            return 1;
        }
    }
    return 1;   // not reached
}

// ============================================================================


int mt_iom_io(int chan, t_uint64 *wordp, int* majorp, int* subp)
{
    // log_msg(DEBUG_MSG, "MT::iom_io", "Chan 0%o\n", chan);

    if (chan < 0 || chan >= ARRAY_SIZE(iom.channels)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_io", "Bad channel %d\n", chan);
        return 1;
    }

    DEVICE* devp = iom.channels[chan].dev;
    if (devp == NULL || devp->units == NULL) {
        *majorp = 05;
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_io", "Internal error, no device and/or unit for channel 0%o\n", chan);
        return 1;
    }
    UNIT* unitp = devp->units;
    // BUG: no dev_code

    struct s_tape_state *tape_statep = &tape_state[chan];

    if (tape_statep->io_mode == no_mode) {
        // no prior read or write command
        *majorp = 013;  // MPC Device Data Alert
        *subp = 02;     // Inconsistent command
        log_msg(ERR_MSG, "MT::iom_io", "Bad channel %d\n", chan);
        return 1;
    } else if (tape_statep->io_mode == read_mode) {
        // read
        if (bitstm_get(tape_statep->bitsp, 36, wordp) != 0) {
            // BUG: There isn't another word to be read from the tape buffer, but the IOM wants
            // another word.
            // BUG: How did this tape hardare handle an attempt to read more data than was present?
            // One answer is in bootload_tape_label.alm which seems to assume a 4000 all-clear status.
            // Boot_tape_io.pl1 seems to assume that "short reads" into an over-large buffer
            // should not yield any error return.
            // So we'll set the flags to all-ok, but return an out-of-band non-zero status to
            // make the iom stop.
            // BUG: The IOM should be updated to return its DCW tally residue to the caller.
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            log_msg(ERR_MSG, "MT::iom_io", "Read buffer exhausted on channel %d\n", chan);
            return 1;
        }
        *majorp = 0;
        *subp = 0;      // BUG: do we need to detect end-of-record?
        if (sim_tape_wrp(unitp)) *subp |= 1;
        //if (opt_debug > 2)
        //  log_msg(DEBUG_MSG, "MT::iom_io", "Data moved from tape controller buffer to IOM\n");
        return 0;
    } else {
        // write
        log_msg(ERR_MSG, "MT::iom_io", "Write I/O Unimplemented\n");
        *majorp = 043;  // DATA ALERT   
        *subp = 040;        // Reflective end of tape mark found while trying to write
        return 1;
    }

    /*notreached*/
    *majorp = 0;
    *subp = 0;
    log_msg(ERR_MSG, "MT::iom_io", "Internal error.\n");
    cancel_run(STOP_BUG);
    return 1;
}

t_stat mt_svc(UNIT *up)
{
    const char* moi = "MT::service";
    log_msg(INFO_MSG, moi, "Calling channel service.\n");
    return channel_svc(up);
}

static const char *simh_tape_msg(int code)
{
    // WARNING: Only selected SIMH tape routines return private tape codes
    // WARNING: returns static buf
    // static char msg[80];
    if (code == MTSE_OK)
        return "OK";
    else if (code == MTSE_UNATT)
        return "Unit not attached to a file";
    else if (code == MTSE_FMT)
        return "Unit specifies an unsupported tape file format";
    else if (code == MTSE_IOERR)
        return "Host OS I/O error";
    else if (code == MTSE_INVRL)
        return "Invalid record length (exceeds maximum allowed)";
    else if (code == MTSE_RECE)
        return "Record header contains error flag";
    else if (code == MTSE_TMK)
        return "Tape mark encountered";
    else if (code == MTSE_BOT)
        return "BOT encountered during reverse operation";
    else if (code == MTSE_EOM)
        return "End of Medium encountered";
    else if (code == MTSE_WRP)
        return "Write protected unit during write operation";
    else
        return "Unknown SIMH tape error";
}
