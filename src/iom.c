/*
    iom.c
*/

#include "hw6180.h"
#include <sys/time.h>

extern t_uint64 M[];    /* memory */
extern cpu_ports_t cpu_ports;
extern scu_t scu;   // BUG: we'll need more than one for max memory.  Unless we can abstract past the physical HW's capabilities

void dump_iom_mbx(void);

void iom_interrupt()
{
    debug_msg("IOM", "Unimplemented\n");
    dump_iom_mbx();
    cancel_run(STOP_BUG);
}

void dump_iom_mbx()
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
    currp = 01400;
    out_msg("IOM A Channel Mailboxes at %0o\n", currp);
    for (int i = 0; i < 64; ++i, currp += 4) {
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
}
