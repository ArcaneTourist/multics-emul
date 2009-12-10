/*
    iom.c -- emulation of an I/O Multiplexer

    See: Document 43A239854 -- 6000B I/O Multiplexer
    (43A239854_600B_IOM_Spec_Jul75.pdf)

    See AN87 which specifies some details of portions of PCWs that are
    interpreted by the channel boards and not the IOM itself.

    See also: http://www.multicians.org/fjcc5.html -- Communications
    and Input/Output Switching in a Multiplex Computing System

    See also: Patents: 4092715, 4173783, 1593312
*/

/*

    3.10 some more physical switches

    Config switch: 3 positions -- Standard GCOS, Extended GCOS, Multics

    Note that all M[addr] references are absolute (The IOM has no access to
    the CPU's appending hardware.)
*/

#include "hw6180.h"
#include <sys/time.h>

extern t_uint64 M[];    /* memory */
extern cpu_ports_t cpu_ports;
extern scu_t scu;
extern iom_t iom;

// ============================================================================
// === Typedefs

enum iom_sys_faults {
    // List from 4.5.1; descr from AN87, 3-9
    iom_no_fault = 0,
    iom_ill_chan = 01,      // PCW to chan with chan number >= 40
    // iom_ill_ser_req=02,  // chan requested svc with a service code of zero or bad chan #
    iom_256K_of=04,         // 256K overflow -- address decremented to zero, but not tally
    iom_lpw_tro_conn = 05,  // tally was zero for an update LPW (LPW bit 21==0) when the LPW was fetched for the connect channel
    iom_not_pcw_conn=06,    // DCW for conn channel had bits 18..20 != 111b
    // iom_cp1_data=07,     // DCW was TDCW or had bits 18..20 == 111b
    iom_ill_tly_cont = 013, // LPW bits 21-22 == 00 when LPW was fetched for the connect channel
    // 14 LPW had bit 23 on in Multics mode
};
enum iom_user_faults {  // aka central status
    // from 4.5.2
    iom_lpw_tro = 1,    //  tally was zero for an update LPW (LPW bit 21==0) when the LPW was fetched  and TRO-signal (bit 22) is on
    iom_bndy_vio = 03,
};

typedef struct {
    uint32 dcw; // bits 0..17
    flag_t ires;    // bit 18; IDCW restrict
    flag_t hrel;    // bit 19; hardware relative addressing
    flag_t ae;      // bit 20; address extension
    flag_t nc;      // bit 21; no tally; zero means update tally
    flag_t trunout; // bit 22; signal tally runout?
    flag_t srel;    // bit 23; software relative addressing; not for Multics!
    uint32 tally;   // bits 24..35
    // following not valid for paged mode; see B15; but maybe IOM-B non existant
    uint32 lbnd;
    uint32 size;
    uint32 idcw;    // ptr to most recent dcw, idcw, ...
} lpw_t;

// Much of this is from AN87 as 43A23985 lacked details of 0..11 and 22..36
typedef struct pcw_s {
    int dev_cmd;    // 6 bits; 0..5
    int dev_code;   // 6 bits; 6..11
    int ext;        // 6 bits; 12..17
    int cp;         // 3 bits; 18..20, must be all ones
    flag_t mask;    // extension control or mask; 1 bit; bit 21
    int control;    // 2 bits; bit 22..23
    int chan_cmd;   // 6 bits; bit 24..29; AN87 says: 00 single record xfer, 02 non data xfer, 06 multi-record xfer, 10 single char record xfer
    int chan_data;  // 6 bits; bit 30..35
    //
    int chan;       // 6 bits; bits 3..8 of word 2
} pcw_t;

typedef struct dcw_s {
    enum { ddcw, tdcw, idcw } type;
    union {
        pcw_t instr;
        struct {
            uint daddr; // data address; 18 bits at 0..17);
            uint cp;    // char position; 3 bits 18..20
            uint tctl;  // tally control; 1 bit at 21
            uint type;  // 2 bits at 22..23
            uint tally; // 12 bits at 24..35
        } ddcw;
        struct {
            uint addr;
            flag_t ec;  // extension control
            flag_t i;   // IDCW control
            flag_t r;   // relative addressing control
        } xfer;
    } fields;
} dcw_t;


#if 0
// from AN87, 3-8
typedef struct {
    int channel;    // 9 bits
    int serv_req;   // 5 bits; see AN87, 3-9
    int ctlr_fault; // 4 bits; SC ill action codes, AN87 sect II
    int io_fault;   // 6 bits; see enum iom_sys_faults
} sys_fault_t;
#endif

// from AN87, 3-11
typedef struct {
    int chan;       // not part of the status word; simulator only
    int major;
    int substatus;
    // even/odd bit
    // status marker bit
    // soft, 2 bits set to zero by hw
    // initiate bit
    // chan_stat; 3 bits; 1=busy, 2=invalid chan, 3=incorrect dcw, 4=incomplete
    // iom_stat; 3 bits; 1=tro, 2=2tdcw, 3=bndry, 4=addr ext, 5=idcw,
    int addr_ext;   // BUG: not maintained
    // rcound;  // residue in PCW or last IDCW
    // addr;    // addr of *next* data word to be transmitted
    // char_pos
    // read;    // flag
    // type;    // 1 bit
    // dcw_residue; // residue in tally of last dcw
    flag_t power_off;
} chan_status_t;


// ============================================================================
// === Static globals

// #define MAXCHAN 64

#define IOM_A_MBX 01400     /* location of mailboxes for IOM A */
#define IOM_CONNECT_CHAN 2

static chan_status_t chan_status;


// ============================================================================
// === Internal functions

//static void dump_iom(void);
//static void dump_iom_mbx(int base, int i);
static void iom_fault(int chan, int src_line, int is_sys, int signal);
static int list_service(int chan, int first_list, int *ptro, int *addr);
static int handle_pcw(int chan, int addr);
static int do_channel(int chan, pcw_t *p);
static int do_dcw(int chan, int addr, int *control, int *need_indir_svc);
static int do_ddcw(int chan, int addr, dcw_t *dcwp, int *control);
static int lpw_write(int chan, int chanloc, const lpw_t* lpw);
static int do_connect_chan(void);
static char* lpw2text(const lpw_t *p, int conn);
static char* pcw2text(const pcw_t *p);
static char* dcw2text(const dcw_t *p);
static void parse_lpw(lpw_t *p, int addr, int is_conn);
static void parse_pcw(pcw_t *p, int addr, int ext);
static void parse_dcw(dcw_t *p, int addr);
static int dev_send_pcw(int chan, pcw_t *p);
static int status_service(int chan);
static int send_chan_flags();
// static int send_general_interrupt(int chan, int pic);
// static int send_marker_interrupt(int chan);
// static int list_service_whatif(int chan, int first_list, int *ptro, int *addrp);

// ============================================================================

/*
 * iom_interrupt()
 *
 * Top level interface to the IOM for the SCU.   Simulates receipt of a $CON
 * signal.  The $CON signal is sent from an SCU when a CPU executes a CIOC
 * instruction asking the SCU to signal the IOM.
 */

void iom_interrupt()
{
    // Simulate receipt of a $CON signal (caused by CPU using a CIOC
    // instruction to signal SCU)
    // Actually, the BUS would give us more than just the channel:
    //      for program interrupt service
    //          interrupt level
    //          addr ext (0-2)
    //          channel number
    //          servicer request code
    //          addr ext (3-5)
    //      for status service signals
    //          ...
    //      for data or list service
    //          direct data addr
    //          addr ext (0-2)
    //          channel number
    //          service request code
    //          mode
    //          DP
    //          chan size
    //          addr ext (3-5)

    extern DEVICE cpu_dev;
    ++ opt_debug; ++ cpu_dev.dctrl;
    log_msg(DEBUG_MSG, "IOM::CIOC::intr", "Starting\n");
    do_connect_chan();
    log_msg(DEBUG_MSG, "IOM::CIOC::intr", "Finished\n");
    -- opt_debug; -- cpu_dev.dctrl;
    log_forget_ic();    // we log a lot of msgs, so a blank line between CIOCs in an I/O loop is good...
}

// ============================================================================

/*
 * do_connect_chan()
 *
 * Process the "connect channel".  This is what the IOM does when it
 * receives a $CON signal.
 *
 * Only called by iom_interrupt() just above.
 *
 * The connect channel requests one or more "list services" and processes the
 * resulting PCW control words.
 */

static int do_connect_chan()
{
    const char *moi = "IOM::conn-chan";
    
    // BUG: don't detect channel status #1 "unexpected PCW (connect while busy)"

    int ptro = 0;   // pre-tally-run-out, e.g. end of list
    int addr;
    int ret = 0;
    while (ptro == 0) {
        log_msg(DEBUG_MSG, moi, "Doing list service for Connect Channel\n");
        ret = list_service(IOM_CONNECT_CHAN, 1, &ptro, &addr);
        if (ret == 0) {
            log_msg(DEBUG_MSG, moi, "Return code zero from Connect Channel list service, doing dcw\n");
            // Do next PCW  (0720201 at loc zero for bootload_tape_label.alm)
            ret = handle_pcw(IOM_CONNECT_CHAN, addr);
        } else {
            log_msg(DEBUG_MSG, moi, "Return code non-zero from Connect Channel list service, skipping dcw\n");
            break;
        }
        // BUG: update LPW for chan 2 in core -- unless list service does it
        // BUG: Stop if tro system fault occured
    }
    return ret;
}

// ============================================================================

/*
 * handle_pcw()
 *
 * Process a PCW (Peripheral Control Word)
 *
 * The PCW indicates a physical channel and usually specifies a command to
 * be sent to the peripheral on that channel.
 *
 * Only the connect channel has lists that contain PCWs. (Other channels
 * use IDCWs (Instruction Data Control Words) to send commands to devices).
 *
 * Only called by do_connect_chan() just above.
 */

static int handle_pcw(int chan, int addr)
{
    const char *moi = "IOM::conn-pcw";

    log_msg(DEBUG_MSG, moi, "PCW for chan %d, addr 0%o\n", chan, addr);
    pcw_t pcw;
    parse_pcw(&pcw, addr, 1);
    log_msg(INFO_MSG, moi, "PCW is: %s\n", pcw2text(&pcw));

    if (pcw.chan < 0 || pcw.chan >= 040) {  // 040 == 32 decimal
        iom_fault(chan, __LINE__, 1, iom_ill_chan); // BUG: what about ill-ser-req? -- is iom issuing a pcw or is channel requesting svc?
        return 1;
    }
    if (pcw.cp != 07) {
        iom_fault(chan, __LINE__, 1, iom_not_pcw_conn);
        return 1;
    }

    if (pcw.mask) {
        // BUG: set mask flags for channel?
        log_msg(ERR_MSG, moi, "PCW Mask not implemented\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    return do_channel(pcw.chan, &pcw);
}

// ============================================================================

/*
 * do_channel()
 *
 * Simulates the operation of a channel.  Channels run asynchrounsly from
 * the IOM central.  Calling this function represents the channel receiving
 * a PCW.
 *
 * Only called by handle_pcw() just above, which in turn, is only
 * called by the connect channel.   However, note that the channel
 * being processed is the one specified in the PCW, not the connect
 * channel.
 *
 * Send a PCW to the device and perform any other operations specified
 * by the PCW.  This can include requesting one or more list services,
 * sending interrupts, dispatching DCWs, and lastly performing a status
 * service.
 *
 * This code is probably not quite correct; the nuances around the looping
 * controls may be wrong...
 */

static int do_channel(int chan, pcw_t *p)
{
    const char* moi = "IOM::do-chan";

    int ret = 0;
    chan_status.chan = chan;

    log_msg(DEBUG_MSG, moi, "Starting for channel 0%o (%d)\n", chan, chan);

    /*
     * First of four phases -- send any PCW command to the device
     */

    ret = dev_send_pcw(chan, p);
    if (ret != 0) {
        log_msg(WARN_MSG, moi, "Device on channel 0%o(%d) did not like our PCW -- non zero return.\n", chan, chan);
    }


    /*
     * Second of three phases
     *     Loop requesting list service(s) and data service(s)
     *
     * Check to see the PCW requires us to request a list service,
     * or requires us to send an interrupt, etc
     */

    // So, try to guess if we need a list service and/or data service.  If
    // so, force the control be two, the code for a list service.

    int first = 1;
    int control = p->control;
    int first_list = 1;
    if (control == 0 && chan_status.major == 0) {
        // BUG: It's not clear when a list service is needed
        // The diag tape seems to want one unless (p->chan_cmd == 02)
        // The boot tape seems to care whether or not (p->chan_data != 0)
        if (p->chan_cmd != 02 || p->chan_data != 0) {
            log_msg(WARN_MSG, moi, "Forcing at least one list service (this PCW implies data service).\n");
            control = 2;
        } else
            log_msg(WARN_MSG, moi, "Not forcing at least one list service (this PCW does no data xfer).\n");
    }

    // Now loop doing list services and other actions

    if (ret != 0 || control == 0)
        log_msg(NOTIFY_MSG, moi, "Not doing a single DCW loop.\n");

    int nlist = 0;  // counter for debugging
    int need_indir_svc = 0; // control info from most recent DCW
    while (ret == 0 && (control != 0 || need_indir_svc)) {
        log_msg(DEBUG_MSG, moi, "In channel loop.\n");
        if (chan_status.major != 0) {
            log_msg(DEBUG_MSG, moi, "Channel has non-zero major status; terminating DCW loop.\n");
            break;
        }
        if (control == 2 || need_indir_svc) {
            need_indir_svc = 0;
            int addr;
            ++ nlist;
            log_msg(DEBUG_MSG, moi, "Asking for %s list service (svc # %d).\n", first_list ? "first" : "another", nlist);
            if (list_service(chan, first_list, NULL, &addr) != 0) {
                ret = 1;
                log_msg(WARN_MSG, moi, "List service indicates failure\n");
            } else {
                log_msg(DEBUG_MSG, moi, "List service yields DCW at addr 0%o\n", addr);
                control = -1;
                ret = do_dcw(chan, addr, &control, &need_indir_svc);
                log_msg(DEBUG_MSG, moi, "Back from latest do_dcw (at %0o); control = %d\n", addr, control);
                if (chan_status.major == 0) {
                    if (need_indir_svc)
                        log_msg(WARN_MSG, moi, "DCW requests indirect data service; will use a list-service to find the next DCW.\n");
                } else {
                    log_msg(DEBUG_MSG, moi, "do_dcw returns with non-zero channel status; terminating DCW loop\n");
                    if (need_indir_svc) {
                        log_msg(DEBUG_MSG, moi, "ignoring request for indirect data service due to non-zero channel status.\n");
                        need_indir_svc = 0;
                    }
                    control = 0;
                }
            }
            first_list = 0;
        } else if (control == 0) {
            // impossible
            // do nothing, terminate
            log_msg(DEBUG_MSG, moi, "DCW has control of zero; terminating DCW loop.\n");
            ret = 1;
        } else if (control == 3) {
            // BUG: set marker interrupt and proceed (list service)
            // Marker interrupts indicate normal completion of
            // a PCW or IDCW
            // PCW control == 3
            // See also: 3.2.7, 3.5.2, 4.3.6
            // See also 3.1.3
#if 1
            log_msg(ERR_MSG, moi, "Set marker not implemented\n");
            ret = 1;
#else
            // Boot tape never requests marker interrupts...
            ret = send_marker_interrupt(chan);
            if (ret == 0) {
                log_msg(NOTIFY_MSG, moi, "Asking for a list service due to set-marker-interrupt-and-proceed.\n");
                control = 2;
            }
#endif
        } else {
            log_msg(ERR_MSG, moi, "Bad PCW/DCW control, %d\n", control);
            cancel_run(STOP_BUG);
            ret = 1;
        }
    }

    /*
     * Third of four phases -- request a status service
     */

    // BUG: skip status service if system fault exists
    log_msg(DEBUG_MSG, moi, "Requesting Status service\n");
    status_service(chan);

    /*
     * Fourth of four phases
     *
     * 3.0 -- Following the status service, the channel will request the
     * IOM to do a multiplex interrupt service.
     */

    log_msg(DEBUG_MSG, moi, "Finished\n");
    return ret;
}

// ============================================================================

#if 0
static int list_service_orig(int chan, int first_list, int *ptro)
{

    // Flowchart 256K overflow checks relate to 18bit offset & tally incr
    // TRO -- tally runout, a fault (not matching CPU fault names)
    //      user fault: lpw tr0 sent to channel (sys fault lpw tr0 for conn chan)
    // PTRO -- pre-tally runout -- only used internally?
    // page 82 etc for status

    // addr upper bits -- from: PCW in extended GCOS mode, or IDCW for list service (2.1.2)
    // paging mode given by PTP (page table ptr) in second word of PCW (for IOM-B)
    // 3 modes: std gcos; ext gcos, paged
    // paged: all acc either paged or extneded gcos, not relative moe

}
#endif

 
#if 0
static int list_service_whatif(int chan, int first_list, int *ptro, int *addrp)
{
    lpw_t lpw;
    int chanloc = IOM_A_MBX + chan * 4;
    const char* moi = "IOM::list-svc-whatif";

    parse_lpw(&lpw, chanloc, chan == IOM_CONNECT_CHAN);
    log_msg(DEBUG_MSG, moi, "LPW: %s\n", lpw2text(&lpw, chan == IOM_CONNECT_CHAN));
    return 0;
}
#endif

// ============================================================================

/*
 * list_service()
 *
 * Perform a list service for a channel.
 *
 * Examines the LPW (list pointer word).  Returns the 24-bit core address
 * of the next PCW or DCW.
 *
 * Called by do_connect_chan() for the connect channel or by do_channel()
 * for other channels.
 * 
 * This code is probably not quite correct.  In particular, there appear
 * to be cases where the LPW will specify the next action to be taken,
 * but only a copy of the LPW in the IOM should be updated and not the
 * LPW in main core.
 *
 */

static int list_service(int chan, int first_list, int *ptro, int *addrp)
{
    // Core address of next PCW or DCW is returned in *addrp.  Pre-tally-runout
    // is returned in *ptro.

    lpw_t lpw;
    int chanloc = IOM_A_MBX + chan * 4;
    const char* moi = "IOM::list-service";

    *addrp = -1;
    parse_lpw(&lpw, chanloc, chan == IOM_CONNECT_CHAN);
    log_msg(DEBUG_MSG, moi, "Starting for LPW for channel %0o(%d dec) at addr %0o\n", chan, chan, chanloc);
    log_msg(DEBUG_MSG, moi, "LPW: %s\n", lpw2text(&lpw, chan == IOM_CONNECT_CHAN));

    if (lpw.srel) {
        log_msg(ERR_MSG, moi, "LPW with bit 23 on is invalid for Multics mode\n");
        iom_fault(chan, __LINE__, 1, 014);  // TODO: want enum
        cancel_run(STOP_BUG);
        return 1;
    }
    if (first_list) {
        lpw.hrel = lpw.srel;
    }
    if (lpw.ae != lpw.hrel) {
        log_msg(WARN_MSG, moi, "AE does not match HREL\n");
        cancel_run(STOP_BUG);
    }

    // Check for TRO or PTRO at time that LPW is fetched -- not later

    if (ptro != NULL)
        *ptro = 0;
    int addr = lpw.dcw;
    if (chan == IOM_CONNECT_CHAN) {
        if (lpw.nc == 0 && lpw.trunout == 0) {
            log_msg(WARN_MSG, moi, "Illegal tally connect channel\n");
            iom_fault(chan, __LINE__, 1, iom_ill_tly_cont);
            cancel_run(STOP_WARN);
            return 1;
        }
        if (lpw.nc == 0 && lpw.trunout == 1)
            if (lpw.tally == 0) {
                log_msg(WARN_MSG, moi, "TRO on connect channel\n");
                iom_fault(chan, __LINE__, 1, iom_lpw_tro_conn);
                cancel_run(STOP_WARN);
                return 1;
            }
        if (lpw.nc == 1) {
            // we're not updating tally, so pretend it's at zero
            if (ptro != NULL)
                *ptro = 1;  // forced, see pg 23
        }
        *addrp = addr;  // BUG: force y-pair
        log_msg(DEBUG_MSG, moi, "Expecting that connect channel will pull DCW from core\n");
    } else {
        // non connect channel
        // first, do an addr check for overflow
        int overflow = 0;
        if (lpw.ae) {
            int sz = lpw.size;
            if (lpw.size == 0) {
                log_msg(INFO_MSG, "IOM::list-sevice", "LPW size is zero; interpreting as 4096\n");
                sz = 010000;    // 4096
            }
            if (addr >= sz)     // BUG: was >
                overflow = 1; // signal or record below
            else
                addr = lpw.lbnd + addr ;
        }
        // see flowchart 4.3.1b
        if (lpw.nc == 0 && lpw.trunout == 0) {
            if (overflow) {
                iom_fault(chan, __LINE__, 1, iom_256K_of);
                return 1;
            }
        }
        if (lpw.nc == 0 && lpw.trunout == 1) {
            // BUG: Chart not fully handled (nothing after (C) except T-DCW detect)
            for (;;) {
                if (lpw.tally == 0) {
                    log_msg(WARN_MSG, moi, "TRO on channel 0%o\n", chan);
                    iom_fault(chan, __LINE__, 0, iom_lpw_tro);
                    cancel_run(STOP_WARN);
                    // user fault, no return
                    break;
                }
                if (lpw.tally > 1) {
                    if (overflow) {
                        iom_fault(chan, __LINE__, 1, iom_256K_of);
                        return 1;
                    }
                }
                // Check for T-DCW
                int t = getbits36(M[addr], 18, 3);
                if (t == 2) {
                    uint next_addr = M[addr] >> 18;
                    log_msg(ERR_MSG, moi, "Transfer-DCW not implemented; addr would be %06o; E,I,R = 0%o\n", next_addr, M[addr] & 07);
                    return 1;
                } else
                    break;
            }
        }
        *addrp = addr;
        // if in GCOS mode && lpw.ae) fault;    // bit 20
        // next: channel should pull DCW from core
        log_msg(DEBUG_MSG, moi, "Expecting that channel 0%o will pull DCW from core\n", chan);
    }

    int cp = getbits36(M[addr], 18, 3);
    if (cp == 7) {
        // BUG: update idcw fld of lpw
    }

    // int ret;

    //-------------------------------------------------------------------------
    // ALL THE FOLLOWING HANDLED BY PART "D" of figure 4.3.1b and 4.3.1c
    //          if (pcw.chan == IOM_CONNECT_CHAN) {
    //              log_msg(DEBUG_MSG, "IOM::pcw", "Connect channel does not return status.\n");
    //              return ret;
    //          }
    //          // BUG: need to write status to channel (temp: todo chan==036)
    // update LPW for chan (not 2)
    // update DCWs as used
    // SCW in mbx
    // last, send an interrupt (still 3.0)
    // However .. conn chan does not interrupt, store status, use dcw, or use
    // scw
    //-------------------------------------------------------------------------

    // Part "D" of 4.3.1c

    // BUG BUG ALL THE FOLLOWING IS BOTH CORRECT AND INCORRECT!!! Section 3.0
    // BUG BUG states that LPW for CONN chan is updated in core after each
    // BUG BUG chan is given PCW
    // BUG BUG Worse, below is prob for channels listed in dcw/pcw, not conn

    if (chan_status.chan != IOM_CONNECT_CHAN)
        send_chan_flags();

    int write_lpw = 0;
    int write_lpw_ext = 0;
    int write_any = 1;
    if (lpw.nc == 0) {
        if (lpw.trunout == 1) {
            if (lpw.tally == 1) {
                if (ptro != NULL)
                    *ptro = 1;
            } else if (lpw.tally == 0) {
                write_any = 0;
                if (chan == IOM_CONNECT_CHAN)
                    iom_fault(chan, __LINE__, 1, iom_lpw_tro_conn);
                else
                    iom_fault(chan, __LINE__, 0, iom_bndy_vio); // BUG: might be wrong
            }
        }
        if (write_any) {
            -- lpw.tally;
            if (chan == IOM_CONNECT_CHAN)
                lpw.dcw += 2;   // pcw is two words
            else
                ++ lpw.dcw;     // dcw is one word
        }
    } else  {
        // note: ptro forced earlier
        write_any = 0;
    }

int did_idcw = 0;   // BUG
int did_tdcw = 0;   // BUG
    if (lpw.nc == 0) {
        write_lpw = 1;
        if (did_idcw || first_list)
            write_lpw_ext = 1;
    } else {
        // no update
        if (did_idcw || first_list) {
            write_lpw = 1;
            write_lpw_ext = 1;
        } else if (did_tdcw) 
            write_lpw = 1;
    }
    //if (pcw.chan != IOM_CONNECT_CHAN) {
    //  ; // BUG: write lpw
    //}
    lpw_write(chan, chanloc, &lpw);     // BUG: we always write LPW

    log_msg(DEBUG_MSG, "IOM::list-sevice", "returning\n");
    return 0;   // BUG: unfinished
}

// ============================================================================

//static int do_pcw(int chan, pcw_t *p)
//{
//  log_msg(DEBUG_MSG, "IOM::do-pcw", "Using dev-send-pcw\n");
//  return dev_send_pcw(chan, p);
//}

// ============================================================================

/*
 * do_dcw()
 *
 * Called by do_channel() when a DCW (Data Control Word) is seen.
 *
 * DCWs may specify a command to be sent to a device or may specify
 * I/O transfer(s).
 *
 * *controlp will be set to 0, 2, or 3 -- indicates terminate, 
 * proceed (request another list services), or send marker interrupt
 * and proceed
 */

static int do_dcw(int chan, int addr, int *controlp, int *need_indir_svc)
{
    extern DEVICE cpu_dev;
    if (iom.channels[chan] == DEV_CON) {
        ++ opt_debug; ++ cpu_dev.dctrl;
    }

    dcw_t dcw;
    log_msg(DEBUG_MSG, "IOM::dcw", "chan %d, addr 0%o\n", chan, addr);
    if (M[addr] == 0) {
        log_msg(ERR_MSG, "IOM::dcw", "DCW of all zeros is legal but useless (unless you want to dump first 4K of memory).\n");
        log_msg(ERR_MSG, "IOM::dcw", "Disallowing legal but useless all zeros DCW at address %012llo.\n", addr);
        cancel_run(STOP_BUG);
        if (iom.channels[chan] == DEV_CON) { -- opt_debug; -- cpu_dev.dctrl; }
        return 1;
    }
        
    parse_dcw(&dcw, addr);
    if (dcw.type == idcw) {
        // instr dcw
        dcw.fields.instr.chan = chan;   // Real HW would not populate
        log_msg(DEBUG_MSG, "IOM::DCW", "%s\n", dcw2text(&dcw));
        *controlp = dcw.fields.instr.control;
        int ret = dev_send_pcw(chan, &dcw.fields.instr);
        if (ret != 0)
            log_msg(DEBUG_MSG, "IOM::dcw", "dev-send-pcw returns %d.\n", ret);
        if (dcw.fields.instr.chan_cmd != 02) {
#if 1
            *need_indir_svc = 1;
#else
            if (dcw.fields.instr.control != 2) {
                log_msg(DEBUG_MSG, "IOM::dcw", "Changing control from %d to 2 to force list service after I-DCW with i/o transfer\n", dcw.fields.instr.control);
                *controlp = 2;
            }
#endif
        }
        if (iom.channels[chan] == DEV_CON) { -- opt_debug; -- cpu_dev.dctrl; }
        return ret;
    } else if (dcw.type == tdcw) {
        if (iom.channels[chan] == DEV_CON) { -- opt_debug; -- cpu_dev.dctrl; }
        uint next_addr = M[addr] >> 18;
        log_msg(ERR_MSG, "IOW::DCW", "Transfer-DCW not implemented; addr would be %06o; E,I,R = 0%o\n", next_addr, M[addr] & 07);
        return 1;
    } else  if (dcw.type == ddcw) {
        // IOTD, IOTP, or IONTP -- i/o (non) transfer
        int ret = do_ddcw(chan, addr, &dcw, controlp);
        if (iom.channels[chan] == DEV_CON) { -- opt_debug; -- cpu_dev.dctrl; }
        return ret;
    } else {
        log_msg(ERR_MSG, "IOW::DCW", "Unknown DCW type\n");
        if (iom.channels[chan] == DEV_CON) { -- opt_debug; -- cpu_dev.dctrl; }
        return 1;
    }
}

// ============================================================================

/*
 * dev_send_pcw()
 *
 * Send a PCW (Peripheral Control Word) or an IDCW Instruction Data Control
 * Word) to a device.   PCWs and IDCWs are typically used for sending 
 * commands to devices but not for requesting I/O transfers.  PCWs are only
 * used by the connect channel; the other channels use IDCWs.  IDCWs are
 * essentially PCWs.
 *
 * Note that we don't generate marker interrupts here; instead we
 * expect the caller to handle.
 *
 * See dev_io() below for handling of Data DCWS and I/O transfers.
 *
 * The various devices are implemented in other source files.  The IOM
 * expects two routines for each device: one to handle commands and
 * one to handle I/O transfers.
 *
 */

static int dev_send_pcw(int chan, pcw_t *p)
{
    log_msg(NOTIFY_MSG, "IOM::dev-send-pcw", "Starting for channel 0%o(%d).  PCW: %s\n", chan, chan, pcw2text(p));

    DEVICE* devp = iom.devices[chan];
    // if (devp == NULL || devp->units == NULL)
    if (devp == NULL) {
        // BUG: no device connected, what's the fault code(s) ?
        chan_status.power_off = 1;
        log_msg(WARN_MSG, "IOM::dev-send-pcw", "No device connected to channel %#o(%d); Auto breakpoint.\n", chan, chan);
        iom_fault(chan, __LINE__, 0, 0);
        cancel_run(STOP_IBKPT);
        return 1;
    }
    chan_status.power_off = 0;

    switch(iom.channels[chan]) {
        case DEV_NONE:
            // BUG: no device connected, what's the fault code(s) ?
            chan_status.power_off = 1;
            log_msg(WARN_MSG, "IOM::dev-send-pcw", "Device on channel %#o (%d) is missing.\n", chan, chan);
            iom_fault(chan, __LINE__, 0, 0);
            cancel_run(STOP_WARN);
            return 1;
        case DEV_TAPE: {
            int ret = mt_iom_cmd(p->chan, p->dev_cmd, p->dev_code, &chan_status.major, &chan_status.substatus);
            log_msg(DEBUG_MSG, "IOM::dev-send-pcw", "MT returns major code 0%o substatus 0%o\n", chan_status.major, chan_status.substatus);
            //return 0; // ignore ret in favor of chan_status.{major,substatus}
            return ret; // caller must choose between our return and the chan_status.{major,substatus}
        }
        case DEV_CON: {
            int ret = con_iom_cmd(p->chan, p->dev_cmd, p->dev_code, &chan_status.major, &chan_status.substatus);
            log_msg(DEBUG_MSG, "IOM::dev-send-pcw", "CON returns major code 0%o substatus 0%o\n", chan_status.major, chan_status.substatus);
            // log_msg(NOTIFY_MSG, "IOM::dev-send-pcw", "CON: Auto breakpoint\n");
            // cancel_run(STOP_IBKPT);
            // return 0;    // ignore ret in favor of chan_status.{major,substatus}
            return ret; // caller must choose between our return and the chan_status.{major,substatus}
        }
        default:
            log_msg(ERR_MSG, "IOM::dev-send-pcw", "Unknown device type 0%o\n", iom.channels[chan]);
            iom_fault(chan, __LINE__, 1, 0);    // BUG: need to pick a fault code
            cancel_run(STOP_BUG);
            return 1;
    }
    return -1;  // not reached
}

// ============================================================================


/*
 * dev_io()
 *
 * Send an I/O transfer request to a device.
 *
 * Called only by do_ddcw() as part of handling data DCWs (data control words).
 * This function sends or receives a single word.  See do_ddcw() for full
 * details of the handling of data DCWs including looping over multiple words.
 *
 * See dev_send_pcw() above for handling of PCWS and non I/O command requests.
 *
 * The various devices are implemented in other source files.  The IOM
 * expects two routines for each device: one to handle commands and
 * one to handle I/O transfers.
 * 
 * BUG: BUG: We return zero to do_ddcw() even after a failed transfer.   This
 * causes addresses and tallys to become incorrect.   For example, we
 * return zero when the console operator is "distracted". -- fixed
 */

static int dev_io(int chan, t_uint64 *wordp)
{
    DEVICE* devp = iom.devices[chan];
    // if (devp == NULL || devp->units == NULL)
    if (devp == NULL) {
        // BUG: no device connected, what's the fault code(s) ?
        log_msg(WARN_MSG, "IOM::dev-io", "No device connected to chan 0%o\n", chan);
        chan_status.power_off = 1;
        iom_fault(chan, __LINE__, 0, 0);
        cancel_run(STOP_WARN);
        return 1;
    }
    chan_status.power_off = 0;

    switch(iom.channels[chan]) {
        case DEV_NONE:
            // BUG: no device connected, what's the fault code(s) ?
            chan_status.power_off = 1;
            iom_fault(chan, __LINE__, 0, 0);
            log_msg(WARN_MSG, "IOM::dev-io", "Device on channel %#o (%d) is missing.\n", chan, chan);
            cancel_run(STOP_WARN);
            return 1;
        case DEV_TAPE: {
            int ret = mt_iom_io(chan, wordp, &chan_status.major, &chan_status.substatus);
            if (ret != 0 || chan_status.major != 0)
                log_msg(DEBUG_MSG, "IOM::dev-io", "MT returns major code 0%o substatus 0%o\n", chan_status.major, chan_status.substatus);
            //return 0; // ignore ret in favor of chan_status.{major,substatus}
            return ret; // caller must choose between our return and the chan_status.{major,substatus}
        }
        case DEV_CON: {
            int ret = con_iom_io(chan, wordp, &chan_status.major, &chan_status.substatus);
            if (ret != 0 || chan_status.major != 0)
                log_msg(DEBUG_MSG, "IOM::dev-io", "CON returns major code 0%o substatus 0%o\n", chan_status.major, chan_status.substatus);
#if 0
            if (ret != 0)
                log_msg(INFO_MSG, "IOM::dev-io", "BUG: Returning zero even though console has indicated an error.  Major code is %#o, substatus is %#o.\n", chan_status.major, chan_status.substatus);
            return 0;   // ignore ret in favor of chan_status.{major,substatus}
#endif
            return ret; // caller must choose between our return and the chan_status.{major,substatus}
        }
        default:
            log_msg(ERR_MSG, "IOM::dev-io", "Unknown device type 0%o\n", iom.channels[chan]);
            iom_fault(chan, __LINE__, 1, 0);    // BUG: need to pick a fault code
            cancel_run(STOP_BUG);
            return 1;
    }
    return -1;  // not reached
}

// ============================================================================

/*
 * do_ddcw()
 *
 * Process "data" DCWs (Data Control Words).   This function handles DCWs
 * relating to I/O transfers: IOTD, IOTP, and IONTP.
 *
 * Called only by do_dcw() which handles all types of DCWs.
 */

static int do_ddcw(int chan, int addr, dcw_t *dcwp, int *control)
{

    log_msg(DEBUG_MSG, "IOW::DO-DDCW", "%012llo: %s\n", M[addr], dcw2text(dcwp));

    // impossible for (cp == 7); see do_dcw

    // AE, the upper 6 bits of data addr (bits 0..5) from LPW
    // if rel == 0; abs; else absolutize & boundry check
    // BUG: Need to munge daddr

    uint type = dcwp->fields.ddcw.type;
    uint daddr = dcwp->fields.ddcw.daddr;
    uint tally = dcwp->fields.ddcw.tally;
    t_uint64 word = 0;
    t_uint64 *wordp = (type == 3) ? &word : M + daddr;  // 2 impossible; see do_dcw
    if (type == 3 && tally != 1)
        log_msg(ERR_MSG, "IOM::DDCW", "Type is 3, but tally is %d\n", tally);
    int ret;
    if (tally == 0) {
        log_msg(DEBUG_MSG, "IOM::DDCW", "Tally of zero interpreted as 010000(4096)\n");
        tally = 4096;
        log_msg(DEBUG_MSG, "IOM::DDCW", "I/O Request(s) starting at addr 0%o; tally = zero->%d\n", daddr, tally);
    } else
        log_msg(DEBUG_MSG, "IOM::DDCW", "I/O Request(s) starting at addr 0%o; tally = %d\n", daddr, tally);
    for (;;) {
        ret = dev_io(chan, wordp);
        if (ret != 0)
            log_msg(DEBUG_MSG, "IOM::DDCW", "Device for chan 0%o(%d) returns non zero (out of band return)\n", chan, chan);
        if (ret != 0 || chan_status.major != 0)
            break;
        // BUG: BUG: We increment daddr & tally even if device didn't do the
        // transfer, e.g. when the console operator is "distracted".  This
        // is because dev_io() returns zero on failed transfers
        // -- fixed in dev_io()
        ++daddr;    // todo: remove from loop
        if (type != 3)
            ++wordp;
        if (--tally <= 0)
            break;
    }
    log_msg(DEBUG_MSG, "IOM::DDCW", "Last I/O Request was to/from addr 0%o; tally now %d\n", daddr, tally);
    // set control ala PCW as method to indicate terminate or proceed
    if (type == 0) {
        // This DCW is an IOTD -- do I/O and disconnect.  So, we'll 
        // return a code zero --  don't ask for another list service
        *control = 0;
    } else {
        // Tell caller to ask for another list service.
        // Guessing '2' for proceed -- only PCWs and IDCWS should generate
        // marker interrupts?
#if 0
        *control = 3;
#else
        *control = 2;
#endif
    }
    // update dcw
#if 0
    // Assume that DCW is only in scratchpad (bootload_tape_label.alm rd_tape reuses same DCW on each call)
    M[addr] = setbits36(M[addr], 0, 18, daddr);
    M[addr] = setbits36(M[addr], 24, 12, tally);
    log_msg(DEBUG_MSG, "IOM::DDCW", "Data DCW update: %012llo: addr=%0o, tally=%d\n", M[addr], daddr, tally);
#endif
    return ret;
}

// ============================================================================

/*
 * parse_pcw()
 *
 * Parse the words at "addr" into a pcw_t.
 */

static void parse_pcw(pcw_t *p, int addr, int ext)
{
    p->dev_cmd = getbits36(M[addr], 0, 6);
    p->dev_code = getbits36(M[addr], 6, 6);
    p->ext = getbits36(M[addr], 12, 6);
    p->cp = getbits36(M[addr], 18, 3);
    p->mask = getbits36(M[addr], 21, 1);
    p->control = getbits36(M[addr], 22, 2);
    p->chan_cmd = getbits36(M[addr], 24, 6);
    p->chan_data = getbits36(M[addr], 30, 6);
    if (ext) {
        p->chan = getbits36(M[addr+1], 3, 6);
        uint x = getbits36(M[addr+1], 9, 27);
        if (x != 0) {
            // BUG: Should only check if in GCOS or EXT GCOS Mode
            log_msg(ERR_MSG, "IOM::pcw", "Page Table Pointer for model IOM-B detected\n");
            cancel_run(STOP_BUG);
        }
    } else {
        p->chan = -1;
    }
}

// ============================================================================

/*
 * pcw2text()
 *
 * Display pcw_t
 */

static char* pcw2text(const pcw_t *p)
{
    // WARNING: returns single static buffer
    static char buf[80];
    sprintf(buf, "[dev-cmd=0%o, dev-code=0%o, ext=0%o, mask=%d, ctrl=0%o, chan-cmd=0%o, chan-data=0%o, chan=0%o]",
        p->dev_cmd, p->dev_code, p->ext, p->mask, p->control, p->chan_cmd, p->chan_data, p->chan);
    return buf;
}

// ============================================================================

/*
 * parse_dcw()
 *
 * Parse word at "addr" into a dcw_t.
 */

static void parse_dcw(dcw_t *p, int addr)
{
    int cp = getbits36(M[addr], 18, 3);
    const char* moi = "IOM::DCW-parse";

    if (cp == 7) {
        p->type = idcw;
        parse_pcw(&p->fields.instr, addr, 0);
        // p->fields.instr.chan = chan; // Real HW would not populate
        p->fields.instr.chan = -1;
        if (p->fields.instr.mask) {
            // Bit 21 is extension control (EC), not a mask
            // BUG: Check LPW bit 23
            // M[addr] = setbits36(M[addr], 12, 6, present_addr_extension);
            log_msg(ERR_MSG, "IOW::DCW", "I-DCW bit EC not implemented\n");
            cancel_run(STOP_BUG);
            // return 1;
        }
    } else {
        int type = getbits36(M[addr], 22, 2);
        if (type == 2) {
            // transfer
            p->type = tdcw;
            p->fields.xfer.addr = M[addr] >> 18;
            p->fields.xfer.ec = (M[addr] >> 2) & 1;
            p->fields.xfer.i = (M[addr] >> 1) & 1;
            p->fields.xfer.r = M[addr]  & 1;
        } else {
            p->type = ddcw;
            p->fields.ddcw.daddr = getbits36(M[addr], 0, 18);
            p->fields.ddcw.cp = cp;
            p->fields.ddcw.tctl = getbits36(M[addr], 21, 1);
            p->fields.ddcw.type = type;
            p->fields.ddcw.tally = getbits36(M[addr], 24, 12);
        }
    }
    // return 0;
}

// ============================================================================

/*
 * dcw2text()
 *
 * Display a dcw_t
 *
 */

static char* dcw2text(const dcw_t *p)
{
    // WARNING: returns single static buffer
    static char buf[80];
    if (p->type == ddcw) {
        int dtype = p->fields.ddcw.type;
        const char* type =
            (dtype == 0) ? "IOTD" :
            (dtype == 1) ? "IOTP" :
            (dtype == 2) ? "transfer" :
            (dtype == 3) ? "IONTP" :
            "<illegal>";
        sprintf(buf, "D-DCW: type=%d(%s), addr=0%o, cp=0%o, tally=0%o(%d) tally-ctl=%d",
            dtype, type, p->fields.ddcw.daddr, p->fields.ddcw.cp, p->fields.ddcw.tally,
            p->fields.ddcw.tally, p->fields.ddcw.tctl);
    }
    else if (p->type == tdcw)
        sprintf(buf, "T-DCW: ...");
    else if (p->type == idcw)
        sprintf(buf, "I-DCW: %s", pcw2text(&p->fields.instr));
    else
        strcpy(buf, "<not a dcw>");
    return buf;
}

// ============================================================================

/*
 * lpw2text()
 *
 * Display an LPW
 */

static char* lpw2text(const lpw_t *p, int conn)
{
    // WARNING: returns single static buffer
    static char buf[80];
    sprintf(buf, "[dcw=0%o ires=%d hrel=%d ae=%d nc=%d trun=%d srel=%d tally=0%o]",
        p->dcw, p->ires, p->hrel, p->ae, p->nc, p->trunout, p->srel, p->tally);
    if (!conn)
        sprintf(buf+strlen(buf), " [lbnd=0%o size=0%o(%d) idcw=0%o]",
            p->lbnd, p->size, p->size, p->idcw);
    return buf;
}

// ============================================================================

/*
 * parse_lpw()
 *
 * Parse the words at "addr" into a lpw_t.
 */

static void parse_lpw(lpw_t *p, int addr, int is_conn)
{
    p->dcw = M[addr] >> 18;
    p->ires = getbits36(M[addr], 18, 1);
    p->hrel = getbits36(M[addr], 19, 1);
    p->ae = getbits36(M[addr], 20, 1);
    p->nc = getbits36(M[addr], 21, 1);
    p->trunout = getbits36(M[addr], 22, 1);
    p->srel = getbits36(M[addr], 23, 1);
    p->tally = getbits36(M[addr], 24, 12);

    if (!is_conn) {
        // Ignore 2nd word on connect channel
        // following not valid for paged mode; see B15; but maybe IOM-B non existant
        // BUG: look at what bootload does & figure out if they expect 6000-B
        p->lbnd = getbits36(M[addr+1], 0, 9);
        p->size = getbits36(M[addr+1], 9, 9);
        p->idcw = getbits36(M[addr+1], 18, 18);
    } else {
        p->lbnd = -1;
        p->size = -1;
        p->idcw = -1;
    }
}

// ============================================================================

/*
 * print_lpw()
 *
 * Display a LPW along with its channel number
 *
 */

char* print_lpw(t_addr addr)
{
    lpw_t temp;
    int chan = (addr - IOM_A_MBX) / 4;
    parse_lpw(&temp, addr, chan == IOM_CONNECT_CHAN);
    static char buf[80];
    sprintf(buf, "Chan 0%o -- %s", chan, lpw2text(&temp, chan == IOM_CONNECT_CHAN));
    return buf;
}

// ============================================================================

/*
 * lpw_write()
 *
 * Write an LPW into main memory
 */

int lpw_write(int chan, int chanloc, const lpw_t* p)
{
    log_msg(DEBUG_MSG, "IOM::lpw_write", "Chan 0%o: Addr 0%o had %012llo %012llo\n", chan, chanloc, M[chanloc], M[chanloc+1]);
    lpw_t temp;
    parse_lpw(&temp, chanloc, chan == IOM_CONNECT_CHAN);
    //log_msg(DEBUG_MSG, "IOM::lpw_write", "Chan 0%o: Addr 0%o had: %s\n", chan, chanloc, lpw2text(&temp, chan == IOM_CONNECT_CHAN));
    //log_msg(DEBUG_MSG, "IOM::lpw_write", "Chan 0%o: Addr 0%o new: %s\n", chan, chanloc, lpw2text(p, chan == IOM_CONNECT_CHAN));
    t_uint64 word = 0;
    //word = setbits36(0, 0, 18, p->dcw & MASK18);
    word = setbits36(0, 0, 18, p->dcw);
    word = setbits36(word, 18, 1, p->ires);
    word = setbits36(word, 19, 1, p->hrel);
    word = setbits36(word, 20, 1, p->ae);
    word = setbits36(word, 21, 1, p->nc);
    word = setbits36(word, 22, 1, p->trunout);
    word = setbits36(word, 23, 1, p->srel);
    //word = setbits36(word, 24, 12, p->tally & MASKBITS(12));
    word = setbits36(word, 24, 12, p->tally);
    M[chanloc] = word;

    int is_conn = chan == 2;    // BUG: HACK
    if (!is_conn) {
        t_uint64 word2 = setbits36(0, 0, 9, p->lbnd);
        word2 = setbits36(word2, 9, 9, p->size);
        word2 = setbits36(word2, 18, 18, p->idcw);
        M[chanloc+1] = word2;
    }
    log_msg(DEBUG_MSG, "IOM::lpw_write", "Chan 0%o: Addr 0%o now %012llo %012llo\n", chan, chanloc, M[chanloc], M[chanloc+1]);
    return 0;
}

// ============================================================================

/*
 * send_chan_flags(0
 *
 * Stub
 */

static int send_chan_flags()
{
    log_msg(NOTIFY_MSG, "IOM", "send_chan_flags() unimplemented\n");
    return 0;
}

// ============================================================================

/*
 * status_service()
 *
 * Write status info into a status mailbox.
 *
 * BUG: Only partially implemented.
 *
 */

static int status_service(int chan)
{
    // See page 33 and AN87 for format of y-pair of status info

    // BUG: much of the following is not tracked
    
    t_uint64 word1, word2;
    word1 = 0;
    word1 = setbits36(word1, 0, 1, 1);
    word1 = setbits36(word1, 1, 1, chan_status.power_off);
    word1 = setbits36(word1, 2, 4, chan_status.major);
    word1 = setbits36(word1, 6, 6, chan_status.substatus);
    word1 = setbits36(word1, 12, 1, 1); // BUG: even/odd
    word1 = setbits36(word1, 13, 1, 1); // BUG: marker int
    // word1 = setbits36(word1, 14, 2, 0);
    word1 = setbits36(word1, 16, 1, 0); // BUG: initiate flag
    // word1 = setbits36(word1, 17, 1, 0);
    word2 = 0;
#if 0
    word1 = setbits36(word1, 18, 3, chan_status.chan_stat);
    word1 = setbits36(word1, 21, 3, chan_status.iom_stat);
    word1 = setbits36(word1, 24, 6, chan_status.addr_ext);
    word1 = setbits36(word1, 30, 6, chan_status.dcw_residue);

    word2 = setbits36(word2, 0, 18, chan_status.addr);
    word2 = setbits36(word2, 18, 3, chan_status.char_pos);
    word2 = setbits36(word2, 21, 1, chan_status.is_read);
    word2 = setbits36(word2, 22, 2, chan_status.type);
    word2 = setbits36(word2, 24, 12, chan_status.dcw_residue);
#endif

    // BUG: need to write to mailbox queue

    int chanloc = IOM_A_MBX + chan * 4;
    int scw = chanloc + 2;
    if (scw % 2 == 1) { // 3.2.4
        log_msg(WARN_MSG, "IOM::status", "SCW address 0%o is not even\n", scw);
        -- scw;         // force y-pair behavior
    }
    int addr = getbits36(M[scw], 0, 18);    // absolute
    // log_msg(DEBUG_MSG, "IOM::status", "Writing status for chan %d to 0%o\n", chan, addr);
    log_msg(DEBUG_MSG, "IOM::status", "Writing status for chan %d to 0%o=>0%o\n", chan, scw, addr);
    log_msg(DEBUG_MSG, "IOM::status", "Status: 0%012llo 0%012llo\n", word1, word2);
    log_msg(DEBUG_MSG, "IOM::status", "Status: (0)t=Y, (1)pow=%d, (2..5)major=0%02o, (6..11)substatus=0%02o, (12)e/o=Z, (13)marker=Y, (14..15)Z, 16(Z?), 17(Z)\n",
        chan_status.power_off, chan_status.major, chan_status.substatus);
    int lq = getbits36(M[scw], 18, 2);
    int tally = getbits36(M[scw], 24, 12);
#if 1
    if (lq == 3) {
        log_msg(WARN_MSG, "IOM::status", "SCW address 0%o has illegal LQ\n", scw);
        lq = 0;
    }
#endif
    M[addr] = word1;
    M[addr+1] = word2;
    switch(lq) {
        case 0:
            // list
            if (tally != 0) {
                addr += 2;
                -- tally;
            }
            break;
        case 1:
            // 4 entry (8 word) queue
            if (tally % 8 == 1 || tally % 8 == -1)
                addr -= 8;
            else
                addr += 2;
            -- tally;
            break;
        case 2:
            // 16 entry (32 word) queue
            if (tally % 32 == 1 || tally % 32 == -1)
                addr -= 32;
            else
                addr += 2;
            -- tally;
            break;
    }
    if (tally < 0 && tally == - (1 << 11) - 1) {    // 12bits => -2048 .. 2047
        log_msg(WARN_MSG, "IOM::status", "Tally SCW address 0%o wraps to zero\n", tally);
        tally = 0;
    }
    // BUG: update SCW in core
    return 0;
}

// ============================================================================

/*
 * iom_fault()
 *
 * Handle errors internal to the IOM.
 *
 * BUG: Not implemented.
 *
 */

static void iom_fault(int chan, int src_line, int is_sys, int signal)
{
    // Store the indicated fault into a system fault word (3.2.6) in
    // the system fault channel -- use a list service to get a DCW to do so

    // sys fault masks channel

    // signal gets put in bits 30..35, but we need fault code for 26..29

    // BUG: unimplemented

    log_msg(WARN_MSG, "IOM", "Fault for channel %d at line %d: is_sys=%d, signal=%d\n", chan, src_line, is_sys, signal);
    log_msg(ERR_MSG, "IOM", "Not setting status word.\n");
    // cancel_run(STOP_WARN);
}

// ============================================================================

enum iom_imw_pics {
    // These are for bits 19..21 of an IMW address.  This is normally
    // the Interrupt Level from a Channel's Program Interrupt Service
    // Request.  We can deduce from the map in 3.2.7 that certain
    // special case PICs must exist:
    imw_overhead_pic = 1,   // IMW address ...001xx
    imw_terminate_pic = 3,  // IMW address ...011xx
    imw_marker_pic = 5,     // IMW address ...101xx
    imw_special_pic = 7     // IMW address ...111xx
};

/*
 * send_marker_interrupt()
 *
 * Send a "marker" interrupt to the CPU.
 *
 * Channels send marker interrupts to indicate normal completion of
 * a PCW or IDCW if the control field of the PCW/IDCW has a value
 * of three.
 */

#if 0
static int send_marker_interrupt(int chan)
{
    return send_general_interrupt(chan, imw_marker_pic);
}
#endif

// ============================================================================

/*
 * send_general_interrupt()
 *
 * Send an interrupt from the IOM to the CPU.
 *
 */

#if 0
static int send_general_interrupt(int chan, int pic)
{
    const char* moi = "IOM::send-interrupt";

    int imw_addr;
    imw_addr = iom.iom_num; // 2 bits
    imw_addr |=  pic << 2;  // 3 bits
    int interrupt_num = imw_addr;
    // Section 3.2.7 defines the upper bits of the IMW address as
    // being defined by the mailbox base address switches and the
    // multiplex base address switches.
    // However, AN-70 reports that the IMW starts at 01200.  If AN-70 is
    // correct, the bits defined by the mailbox base address switches would
    // have to always be zero.  We'll go with AN-70.  This is equivalent to
    // using bit value 0010100 for the bits defined by the  multiplex base
    // address switches and zeros for the bits defined by the mailbox base
    // address switches.
    imw_addr += 01200;  // all remaining bits

    log_msg(NOTIFY_MSG, moi, "Channel %d (%#o), level %d; Interrupt %d (%#o).\n", chan, chan, pic, interrupt_num, interrupt_num);
    t_uint64 imw = M[imw_addr]; 
    // The 5 least significant bits of the channel determine a bit to be
    // turned on.
    log_msg(INFO_MSG, moi, "IMW at %#o was %012llo\n", imw_addr, imw);
    imw = setbits36(imw, 1 << (chan & 037), 1, 1);
    log_msg(INFO_MSG, moi, "IMW at %#o now %012llo\n", imw_addr, imw);
    M[imw_addr] = imw;

    return scu_set_interrupt(interrupt_num);

#if 1
    log_msg(ERR_MSG, moi, "Not implemented\n");
    // cancel_run(STOP_WARN);
    return 1;
#endif
}
#endif
