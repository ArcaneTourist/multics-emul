/*
    iom.c
        See: Document 43A239854 -- 6000B I/O Multiplexer --  43A239854_600B_IOM_Spec_Jul75.pdf

    3.10 some more physical switches
*/

#include "hw6180.h"
#include <sys/time.h>

#define IOM_A_MBX 01400     /* location of mailboxes for IOM A */
#define IOM_CONNECT_CHAN 2

extern t_uint64 M[];    /* memory */
extern cpu_ports_t cpu_ports;
extern scu_t scu;   // BUG: we'll need more than one for max memory.  Unless we can abstract past the physical HW's capabilities
extern iom_t iom;

enum iom_sys_faults {
    // from 4.5.1; descr from AN87, 3-9
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
    t_bool ires;    // bit 18; IDCW restrict
    t_bool hrel;    // bit 19; hardware relative addressing
    t_bool ae;      // bit 20; address extension
    t_bool nc;      // bit 21; no tally; zero means update tally
    t_bool trunout; // bit 22; signal tally runout?
    t_bool srel;    // bit 23; software relative addressing; not for Multics!
    uint32 tally;   // bits 24..35
    // following not valid for paged mode; see B15; but maybe IOM-B non existant
    uint32 lbnd;
    uint32 size;
    uint32 idcw;    // ptr to most recent dcw, idcw, ...
} lpw_t;

// Much of this is from, AN87 as 43A23985 lacked details of 0..18 and 22..36
typedef struct pcw_s {
    int dev_cmd;    // 6 bits; 0..5
    int dev_code;   // 6 bits; 6..11
    int ext;        // 6 bits; 12..17
    int cp;         // 3 bits; 18..20, must be all ones
    t_bool mask;    // 1 bit; bit 21
    int control;    // 2 bits; bit 22..23
    int chan_cmd;   // 6 bits; bit 24..29
    int chan_data;  // 6 bits; bit 30..35
    //
    int chan;       // 6 bits; bits 3..8 of word 2
} pcw_t;

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
    // chan_stat; 3 bits; 1=busy, 2=invalid chan, 3=incorrect dcw, 5=incomplete
    // iom_stat; 3 bits; 1=tro, 2=2tdcw, 3=bndry, 4=addr ext, 5=idcw,
    // addrext
    // rcound;  // residue in PCW or last IDCW
    // addr;    // addr of *next* data word to be transmitted
    // char_pos
    // read;    // flag
    // type;    // 1 bit
    // dcw_residue; // residue in tally of last dcw
} chan_status_t;

static chan_status_t chan_status;


static void dump_iom(void);
static void iom_fault(int chan, int src_line, int is_sys, int signal);
static int list_service(int chan, int first_list, int *ptro);
static void dump_iom_mbx(int base, int i);
static int handle_pcw(int chan, int addr);
static int do_pcw(int chan, pcw_t *p);
static int do_dcw(int chan, int addr);

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

    // dump_iom();

    debug_msg("IOM::CIOC", "Starting\n");
    int ptro = 0;   // pre-tally-run-out, e.g. end of list
    while (ptro == 0) {
        debug_msg("IOM::CIOC", "Doing list service for Connect Channel\n");
        list_service(IOM_CONNECT_CHAN, 1, &ptro);
    }

    debug_msg("IOM::CIOC", "Unfinished\n");
    cancel_run(STOP_BUG);
}

static void dump_iom()
{
    int currp = IOM_MBX_LOW;
    out_msg("DUMP of IOM Mailbox at %0o\n", currp);
    out_msg("IMW Array at %0o:\n", currp);
    for (int i = 0; i < 32; i += 2) {
        out_msg("    IMW interrupt %2d: %-012Lo           IMW interrupt 0%02d: %-012Lo\n",
            i, M[currp], i+1, M[currp+1]);
        currp += 2;
    }
    out_msg("System Fault Word Circular Queues are at 0%o...\n", currp);
    currp = IOM_A_MBX;
    out_msg("IOM A Channel Mailboxes at %0o\n", currp);
    for (int i = 0; i < 64; ++i, currp += 4) {
        dump_iom_mbx(IOM_A_MBX, i);
    }
}

static void dump_iom_mbx(int base, int i)
{
    int currp = base + i * 4;
        //if (M[currp] == 0 && M[currp+1] == 0 && M[currp+2] == 0 && M[currp+3] == 0)
        //  continue;
        // out_msg("Something at %0o\n", currp);
        //out_msg("    MBX %2d: %12Lo %12Lo %12Lo %12Lo\n",
        //  i, M[currp], M[currp+1], M[currp+2], M[currp+3]);
        uint32 lpw_dcw = M[currp] >> 18;
        uint32 lpw_flags = getbits36(M[currp], 18, 6);
        uint32 lpw_tally = getbits36(M[currp], 24, 12);
        uint32 lpwe_lbnd = getbits36(M[currp+1], 0, 9);
        uint32 lpwe_size = getbits36(M[currp+1], 9, 9);
        uint32 lpwe_idcw = getbits36(M[currp+1], 18, 18);
        uint32 scw_addr = getbits36(M[currp+2], 0, 18);
        uint32 scw_q = getbits36(M[currp+2], 18, 2);
        uint32 scw_tally = getbits36(M[currp+2], 24, 12);
        t_uint64 dcw = M[currp+3];
        out_msg("    MBX %2d: LPW: %6o %2o %4o; LPWE: %3o %3o %6o; SCW: %6o %o %4o; DCW: %Lo\n",
            i,
            lpw_dcw, lpw_flags, lpw_tally,
            lpwe_lbnd, lpwe_size, lpwe_idcw,
            scw_addr, scw_q, scw_tally,
            dcw);
        if (dcw != 0) {
            uint32 dcw_dt_addr = getbits36(M[currp+3], 0, 18);
            uint32 dcw_dt_cp = getbits36(M[currp+3], 18, 3);
            uint32 dcw_dt_c = getbits36(M[currp+3], 21, 1);
            uint32 dcw_dt_t = getbits36(M[currp+3], 22, 2);
            uint32 dcw_dt_tally = getbits36(M[currp+3], 24, 12);
            //
            uint32 dcw_i_dcmd = getbits36(M[currp+3], 0, 6);
            uint32 dcw_i_dcode = getbits36(M[currp+3], 6, 6);
            uint32 dcw_i_addrxtn = getbits36(M[currp+3], 12, 6);
            uint32 dcw_i_m = getbits36(M[currp+3], 21, 1);
            uint32 dcw_i_ccmd = getbits36(M[currp+3], 24, 6);
            uint32 dcw_i_cdata = getbits36(M[currp+3], 30, 6);
            //
            uint32 dcw_t_addr = getbits36(M[currp+3], 0, 18);
            uint32 dcw_t_e = getbits36(M[currp+3], 33, 1);
            uint32 dcw_t_i = getbits36(M[currp+3], 34, 1);
            uint32 dcw_t_r = getbits36(M[currp+3], 35, 1);
            out_msg("                 DCW: DT:[%6o %o %o %o %4o] I:[%2o %2o %2o %o %2o %2o] T:[%6o %o %o %o]\n",
                dcw_dt_addr, dcw_dt_cp, dcw_dt_c, dcw_dt_t, dcw_dt_tally,
                dcw_i_dcmd, dcw_i_dcode, dcw_i_addrxtn, dcw_i_m, dcw_i_ccmd, dcw_i_cdata, 
                dcw_t_addr, dcw_t_e, dcw_t_i, dcw_t_r);
        }
}

char* lpw2text(const lpw_t *p)
{
    // WARNING: returns single static buffer
    static char buf[80];
    sprintf(buf, "[dcw=0%o ires=%d hrel=%d ae=%d nc=%d trun=%d srel=%d tally=0%o] [lbnd=0%o size=0%o(%d) idcw=0%o]",
        p->dcw, p->ires, p->hrel, p->ae, p->nc, p->trunout, p->srel, p->tally,
        p->lbnd, p->size, p->size, p->idcw);
    return buf;
}

int parse_lpw(lpw_t *p, int addr)
{
    // NOTE: We always look at 2nd word even if connect channel shouldn't
    p->dcw = M[addr] >> 18;
    p->ires = getbits36(M[addr], 18, 1);
    p->hrel = getbits36(M[addr], 19, 1);
    p->ae = getbits36(M[addr], 20, 1);
    p->nc = getbits36(M[addr], 21, 1);
    p->trunout = getbits36(M[addr], 22, 1);
    p->srel = getbits36(M[addr], 23, 1);
    p->tally = getbits36(M[addr], 24, 12);

    // following not valid for paged mode; see B15; but maybe IOM-B non existant
    // BUG: look at what bootload does & figure out if they expect 6000-B
    p->lbnd = getbits36(M[addr+1], 0, 9);
    p->size = getbits36(M[addr+1], 9, 9);
    p->idcw = getbits36(M[addr+1], 18, 18);
}


static int list_service_orig(int chan, int first_list, int *ptro)
{

    // Flowchare 256K overflow checks relate to 18bit offset & tally incr
    // TRO -- tally runout, a fault (not matching CPU fault names)
    //      user fault: lpw tr0 sent to channel (sys fault lpw tr0 for conn chan)
    // PTRO -- pre-tally runout -- only used internally?
    // page 82 etc for status

    // addr upper bits -- from: PCW in extended GCOS mode, or IDCW for list service (2.1.2)
    // paging mode given by PTP (page table ptr) in second word of PCW (for IOM-B)
    // 3 modes: std gcos; ext gcos, paged
    // paged: all acc either paged or extneded gcos, not relative moe

}


static int list_service(int chan, int first_list, int *ptro)
{
    lpw_t lpw;
    int chanloc = IOM_A_MBX + chan * 4;

    debug_msg("IOM::list-service", "Checking LPW for channel %0o(%d dec) at addr %0o\n", chan, chan, chanloc);
    // dump_iom_mbx(IOM_A_MBX, chan);

    // Pull LPW -- NOTE: should ignore 2nd word on connect channel
    parse_lpw(&lpw, chanloc);
    debug_msg("IOM::list-service", "LPW: %s\n", lpw2text(&lpw));

    if (lpw.srel) {
        complain_msg("IOM::list-service", "LPW with bit 23 on is invalid for Multics mode\n");
        iom_fault(chan, __LINE__, 1, 014);  // BUG, want enum
        cancel_run(STOP_BUG);
        return 1;
    }
    if (first_list) {
        lpw.hrel = lpw.srel;
    }
    if (lpw.ae != lpw.hrel) {
        warn_msg("IOM::list-service", "AE does not match HREL\n");
        cancel_run(STOP_BUG);
    }

    // Check for TRO or PTRO at time that LPW is fetched -- not later

    if (ptro != NULL)
        *ptro = 0;
    int addr = lpw.dcw;
    if (chan == IOM_CONNECT_CHAN) {
        if (lpw.nc == 0 && lpw.trunout == 0) {
            iom_fault(chan, __LINE__, 1, iom_ill_tly_cont);
            cancel_run(STOP_WARN);
            return 1;
        }
        if (lpw.nc == 0 && lpw.trunout == 1)
            if (lpw.tally == 0) {
                iom_fault(chan, __LINE__, 1, iom_lpw_tro_conn);
                cancel_run(STOP_WARN);
                return 1;
            }
        if (lpw.nc == 1) {
            // we're not updating tally, so pretend it's at zero
            if (ptro != NULL)
                *ptro = 1;  // forced, see pg 23
        }
        // next: pull PCW from core
    } else {
        // non connect channel
        // first, do an addr check for overflow
        int overflow = 0;
        if (lpw.ae) {
            if (addr > lpw.size)
                overflow = 1; // signal or record below
            else
                addr = lpw.lbnd + addr ;
        }
        // see flowchart 4.3.1b
        if (lpw.nc == 0 && lpw.trunout == 0) {
            if (overflow)
                iom_fault(chan, __LINE__, 1, iom_256K_of);
        }
        if (lpw.nc == 0 && lpw.trunout == 1) {
            // BUG: need looping or goto here
            if (lpw.tally == 0) {
                iom_fault(chan, __LINE__, 0, iom_lpw_tro);
                cancel_run(STOP_WARN);
                // no return
            }
            if (lpw.tally > 1) {
                if (overflow) {
                    iom_fault(chan, __LINE__, 1, iom_256K_of);
                    cancel_run(STOP_WARN);
                    return 1;
                }
            }
        }
        //if (lpw.ae)   {   // bit 20
        //  // fault if in GCOS mode
        //}
        // next: Pull DCW using addr calculted above
    }

    int ret;

    if (chan == IOM_CONNECT_CHAN) {
        // Do next PCW  (0720201 at loc zero for bootload_tape_label.alm)
        ret = handle_pcw(chan, addr);
    } else {
        // flowchar allows looping on tdcw until idcw or dcw sent to channel
        // BUG: need looping: (3.x) -- see Notes.iom
        ret = do_dcw(chan, addr);
    }

//-------------------------------------------------------------------------
// ALL THE FOLLOWING HANDLED BY PART "D" of figure 4.3.1b and 4.3.1c
//          if (pcw.chan == IOM_CONNECT_CHAN) {
//              debug_msg("IOM::pcw", "Connect channel does not return status.\n");
//              return ret;
//          }
//          // BUG: need to write status to channel (temp: todo chan==036)
// update LPW for chan (not 2)
// update DCWs as used
// SCW in mbx
// last, send an interrupt (still 3.0)
// However .. conn chan does not interrupt, store status, use dcw, or use scw
//-------------------------------------------------------------------------

    // Part "D" of 4.3.1c

    // BUG BUG ALL THE FOLLOWING IS BOTH CORRECT AND INCORRECT!!! Section 3.0 states
    // BUG BUG that LPW for CONN chan is updated in core after each chan is given PCW
    // BUG BUG Worse, below is prob for channels listed in dcw/pcw, not conn

    if (chan_status.chan != IOM_CONNECT_CHAN)
        send_chan_flags();

    int write_lpw = 0;
    int write_lpw_ext = 0;
    int write_any = 1;
    if (lpw.nc == 0) {
        if (lpw.trunout == 1)
            if (lpw.tally == 1)
                if (ptro != NULL)
                    *ptro = 1;
            else if (lpw.tally == 0) {
                write_any = 0;
                if (chan == IOM_CONNECT_CHAN)
                    iom_fault(chan, __LINE__, 1, iom_lpw_tro_conn);
                else
                    iom_fault(chan, __LINE__, 0, iom_bndy_vio); // BUG: might be wrong
            }
        if (write_any)
            if (chan == IOM_CONNECT_CHAN) {
                lpw.tally -= 2; // maybe should be one?
                lpw.dcw += 2;   // pcw is two words
            }
            else {
                -- lpw.tally;
                ++ lpw.dcw;     // dcw is one word
            }
    } else  {
        // note: ptro forced earlier
        write_any = 0;
    }

int idcw = 0;   // BUG
int tdcw = 0;   // BUG
    if (lpw.nc == 0) {
        write_lpw = 1;
        if (idcw || first_list)
            write_lpw_ext = 1;
    } else {
        // no update
        if (idcw || first_list) {
            write_lpw = 1;
            write_lpw_ext = 1;
        } else if (tdcw) 
            write_lpw = 1;
    }
    //if (pcw.chan != IOM_CONNECT_CHAN) {
    //  ; // BUG: write lpw
    //}
    // BUG: how do we get status for other channels?


    // BUG: need to loop over tally?
    warn_msg("IOM::list-sevice", "returning\n");
    return 1;   // internal error
}

static void iom_fault(int chan, int src_line, int is_sys, int signal)
{
    // signal gets put in bits 30..35, but we need fault code for 26..29
    complain_msg("IOM", "Fault for channel %d at line %d: is_sys=%d, signal=%d\n", chan, src_line, is_sys, signal);
    cancel_run(STOP_WARN);
}


int parse_pcw(pcw_t *p, int addr, int ext)
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
        uint x = getbits36(M[addr+1], 9, 28);
        if (x != 0) {
            // BUG: Should only check if in GCOS or EXT GCOS Mode
            complain_msg("IOM::pcw", "Page Table Pointer for model IOM-B detected\n");
            cancel_run(STOP_BUG);
        }
    } else {
        p->chan = -1;
    }
}

char* pcw2text(const pcw_t *p)
{
    // WARNING: returns single static buffer
    static char buf[80];
    sprintf(buf, "[dev-cmd=0%o, dev-code=0%o, ext=0%o, mask=%d, ctrl=0%o, chan-cmd=0%o, chan-data=0%o, chan=0%o]",
        p->dev_cmd, p->dev_code, p->ext, p->mask, p->control, p->chan_cmd, p->chan_data, p->chan);
    return buf;
}


static int handle_pcw(int chan, int addr)
{
    debug_msg("IOM::pcw", "chan %d, addr 0%o\n", chan, addr);
    pcw_t pcw;
    parse_pcw(&pcw, addr, 1);
    debug_msg("IOM::pcw", "%s\n", pcw2text(&pcw));
    if (pcw.chan < 0 || pcw.chan >= 040) {  // 040 == 32 decimal
        iom_fault(chan, __LINE__, 1, iom_ill_chan); // BUG: what about ill-ser-req? -- is iom issuing a pcw or is channel requesting svc?
        return 1;
    }
    if (pcw.cp != 07 && chan == IOM_CONNECT_CHAN) {     // BUG: is 111 OK in IDCW?
        iom_fault(chan, __LINE__, 1, iom_not_pcw_conn);
        return 1;
    }

    if (pcw.mask) {
        // BUG: set mask flags for channel?
        complain_msg("IOM::pwc", "Mask not implemented\n");
        cancel_run(STOP_BUG);
        return 1;
    }
    return do_pcw(pcw.chan, &pcw);
}


static int do_pcw(int chan, pcw_t *p)
{
    int ret = 0;
    chan_status.chan = chan;
    switch(iom.channels[chan]) {
        case DEV_NONE:
            // BUG: no device connected, what's the fault code(s) ?
            iom_fault(chan, __LINE__, 0, 0);
            cancel_run(STOP_WARN);
            return 1;
        case DEV_TAPE: {
            ret = mt_iom_cmd(p->chan, p->dev_cmd, p->dev_code, &chan_status.major, &chan_status.substatus);
            debug_msg("IOM::pcw", "MT returns major code 0%o substatus 0%o\n", chan_status.major, chan_status.substatus);
            break;
        }
        default:
            iom_fault(chan, __LINE__, 1, 0);    // BUG: need to pick a fault code
            cancel_run(STOP_BUG);
            return 1;
    }

    switch (p->control) {
        case 0:
            // do nothing, terminate
            break;
        case 2:
            debug_msg("IOM::pcw", "Asking for list service\n");
            list_service(chan, 1, NULL);
            debug_msg("IOM::pcw", "Return from list service\n");
            break;
        case 3:
            // BUG: set marker interrupt and proceed (list service)
            complain_msg("IOM::pwc", "Set marker not implemented\n");
            cancel_run(STOP_BUG);
            break;
        default:
            complain_msg("IOM::pwc", "Bad PCW control == %d\n", p->control);
            cancel_run(STOP_BUG);
    }

    // BUG: may need to request status service

    /* -- from orangesquid iom.c -- seems somewhat bogus
    read mbx at iom plus 4*pcw.chan
    take lpw from that loc; extract tally
    read scw
    extract addr
    for i = 0 .. tally; do-dcw(dcw+i)
    */

    return ret;
}

static int do_dcw(int chan, int addr)
{
    debug_msg("IOM::dwc", "chan %d, addr 0%o\n", chan, addr);
    int cp = getbits36(M[addr], 18, 3);
    if (cp == 7) {
        // instr dcw
        int ec = getbits36(M[addr], 21, 1);
        if (ec) {
            // BUG: Check LPW bit 23
            // M[addr] = setbits36(M[addr], 12, 6, present_addr_extension);
            complain_msg("IOW::DCW", "I-DCW bit EC not implemented\n");
            cancel_run(STOP_BUG);
        }
        debug_msg("IOW::DCW", "I-DCW found\n");
        pcw_t pcw;
        parse_pcw(&pcw, addr, 0);
        pcw.mask = 0;   // EC not mask
        pcw.chan = chan;    // Real HW would not populate
        debug_msg("IOM::dcw", "I-DCW: %s\n", pcw2text(&pcw));
        return do_pcw(chan, &pcw);
    } else {
        int type = getbits36(M[addr], 22, 2);
        if (type == 2) {
            // transfer
            complain_msg("IOW::DCW", "Transfer-DCW not implemented\n");
            return 1;
        } else {
            // IOTD, IOTP, or IONTP -- i/o (non) transfer
            uint daddr = getbits36(M[addr], 0, 18);
            uint cp = getbits36(M[addr], 18, 3);
            uint tally = getbits36(M[addr], 24, 12);
            debug_msg("IOW::DCW", "Data DCW: addr=0%o, cp=0%o, tally=%d\n", daddr, cp, tally);
            return 1;
        }
    }
}

int send_chan_flags()
{
}
