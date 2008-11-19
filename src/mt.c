/*
    mt.c -- mag tape
        See manual AN87
*/

#include "hw6180.h"
#include "sim_tape.h"
#include "bits.h"

extern iom_t iom;

static const char *simh_tape_msg(int code); // hack
static const size_t bufsz = 4096 * 1024;
static struct s_tape_state {
    // BUG: this should hang off of UNIT structure
    enum { no_mode, read_mode, write_mode } io_mode;
    uint8 *bufp;
    bitstream_t *bitsp;
} tape_state[ARRAY_SIZE(iom.devices)];

void mt_init()
{
    memset(tape_state, 0, sizeof(tape_state));
}

int mt_iom_cmd(int chan, int dev_cmd, int dev_code, int* majorp, int* subp)
{
    log_msg(DEBUG_MSG, "MT::iom_cmd", "Chan 0%o, dev-cmd 0%o, dev-code 0%o\n", chan, dev_cmd, dev_code);

    // BUG: Should Major be added to 040? and left shifted 6? Ans: it's 4 bits

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_cmd", "Bad channel %d\n", chan);
        return 1;
    }

    DEVICE* devp = iom.devices[chan];
    if (devp == NULL || devp->units == NULL) {
        *majorp = 05;
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_cmd", "Internal error, no device and/or unit for channel 0%o\n", chan);
        return 1;
    }
    UNIT* unitp = devp->units;
    if (dev_code < 0 || dev_code >= devp->numunits) {
        // *major = 042;
        // *subp = 2;
        *majorp = 05;
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_cmd", "Bad dev unit-num 0%o (%d decimal)\n", dev_code, dev_code);
        return 1;
    }
    // BUG: dev_code unused

    struct s_tape_state *tape_statep = &tape_state[chan];

    switch(dev_cmd) {
        case 0: {               // CMD 00 Request status
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            if (sim_tape_bot(unitp)) *subp |= 2;
            if (sim_tape_eot(unitp)) {
                *majorp = 044;  // BUG? should be 3?
                *subp = 023;    // BUG: should be 040?
            }
            // todo: switch to having all cmds update status reg?  This would allow setting 047 bootload complete
            return 0;
        }
        case 5: {               // CMD 05 -- Read Binary Record
            // We read the record into the tape controllers memory; IOM can retrieve via PCW 
            if (tape_statep->bufp == NULL)
                if ((tape_statep->bufp = malloc(bufsz)) == NULL) {
                    log_msg(ERR_MSG, "MT::iom_cmd", "Malloc error\n");
                    *majorp = 012;  // BUG: arbitrary error code; config switch
                    *subp = 1;
                    return 1;
                }
            t_mtrlnt tbc;
            int ret;
            if ((ret = sim_tape_rdrecf(unitp, tape_statep->bufp, &tbc, bufsz)) != 0) {
                if (ret == MTSE_TMK || ret == MTSE_EOM) {
                    log_msg(WARN_MSG, "MT::iom_cmd", "EOF: %s\n", simh_tape_msg(ret));
                    *majorp = 044;  // EOF category
                    *subp = 023;    // EOF file mark
                    cancel_run(STOP_WARN);
                    return 0;
                } else {
                    log_msg(ERR_MSG, "MT::iom_cmd", "Cannot read tape: %d - %s\n", ret, simh_tape_msg(ret));
                    log_msg(ERR_MSG, "MT::iom_cmd", "Returning arbitrary error code\n");
                    *majorp = 010;  // BUG: arbitrary error code; config switch
                    *subp = 1;
                    return 1;
                }
            }
            log_msg(WARN_MSG, "MT::iom_cmd", "Read %d bytes from simulated tape\n", (int) tbc);
            tape_statep->bitsp = bitstm_new(tape_statep->bufp, tbc);
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            tape_statep->io_mode = read_mode;
            return 0;
        }
        case 040:               // CMD 040 -- Reset Status
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            return 0;
        case 046: {             // BSR
            // BUG: Do we need to clear the buffer?
            t_mtrlnt tbc;
            int ret;
            if ((ret = sim_tape_sprecr(unitp, &tbc)) == 0) {
                log_msg(WARN_MSG, "MT::iom_cmd", "Backspace one record\n");
                *majorp = 0;
                *subp = 0;
                if (sim_tape_wrp(unitp)) *subp |= 1;
            } else {
                log_msg(ERR_MSG, "MT::iom_cmd", "Cannot backspace record: %d - %s\n", ret, simh_tape_msg(ret));
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
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
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


int mt_iom_io(int chan, t_uint64 *wordp, int* majorp, int* subp)
{
    // log_msg(DEBUG_MSG, "MT::iom_io", "Chan 0%o\n", chan);

    if (chan < 0 || chan >= ARRAY_SIZE(iom.devices)) {
        *majorp = 05;   // Real HW could not be on bad channel
        *subp = 2;
        log_msg(ERR_MSG, "MT::iom_io", "Bad channel %d\n", chan);
        return 1;
    }

    DEVICE* devp = iom.devices[chan];
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
            // BUG: How did this tape hardare handle an attempt to read more data than was present?
            // For "long" reads, bootload_tape_label.alm seems to assume a 4000 all-clear status,
            // so we'll set the flags to all-ok, but return an out-of-band non-zero status to
            // make the iom stop
            //*majorp = 013;    // MPC Device Data Alert
            //*subp = 02;       // Inconsistent command
            *majorp = 0;
            *subp = 0;
            if (sim_tape_wrp(unitp)) *subp |= 1;
            log_msg(ERR_MSG, "MT::iom_io", "Read buffer exhausted on channel %d\n", chan);
            return 1;
        }
        // log_msg(DEBUG_MSG, "MT::iom_io", "Data moved from tape controller buffer to IOM\n");
        *majorp = 0;
        *subp = 0;      // BUG: do we need to detect end-of-record?
        if (sim_tape_wrp(unitp)) *subp |= 1;
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
    log_msg(ERR_MSG, "MT::iom_io", "Internel error.\n");
    cancel_run(STOP_BUG);
    return 1;
}

t_stat mt_svc(UNIT *up)
{
    log_msg(DEBUG_MSG, "MT::service", "not doing anything!\n");
    cancel_run(STOP_WARN);
    return 0;
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
