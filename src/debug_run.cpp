/*
    debug_run.cpp

    Support for runtime debugging -- tracking and displaying changes
    to registers, the current pl1 source line, automatic (stack based)
    variables, etc.
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

using namespace std;
#include <iostream>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <sstream>
#include <algorithm>

//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>

#include "hw6180.h"
#include "seginfo.hpp"

// BUG: The following externs are hacks
extern DEVICE cpu_dev;

//=============================================================================

// This typedef is a temporary hack for partial history.   Will
// probably turn into the per-cpu state structure.
typedef struct {
    t_uint64 reg_A;
    t_uint64 reg_Q;
    uint8 reg_E;
    uint32 reg_X[8];
    IR_t IR;
    AR_PR_t AR_PR[8];
    PPR_t PPR;
    TPR_t TPR;
    struct {
        flag_t SD_ON;
        flag_t PT_ON;
    } cu;
    cpu_t cpu;
} hist_t;

static hist_t hist;
static int seg_debug[n_segments];
static int seg_debug_init_done = 0;

static void print_src_loc(const char *prefix, addr_modes_t addr_mode, int segno, int ic, const instr_t* instrp);
static void check_autos(int segno, int ic);
static void dump_autos(void);
static void out_auto(ostringstream& obuf, const var_info& v, int addr, int is_initialized);
static void frame_trace(void);
static int walk_stack(int output, list<seg_addr_t>* frame_listp);
// static int push_frame(const seg_addr_t& framep, const linkage_info* lip);
static int is_stack_frame(int segno, int offset);
static int have_stack(void);
static int stack_depth(void);

//=============================================================================

static void init_seg_debug()
{
    memset(seg_debug, 0, sizeof(seg_debug));
    seg_debug_init_done = 1;
}


//=============================================================================

void check_seg_debug()
{
    // Set debug flags, but check for per-segment override to global debug setting

    if (!seg_debug_init_done)
        init_seg_debug();

    opt_debug = (cpu_dev.dctrl != 0);   // todo: should CPU control all debug settings?
    if (get_addr_mode() == APPEND_mode)
        if (PPR.PSR >= 0 && PPR.PSR < ARRAY_SIZE(seg_debug)) {
            if (seg_debug[PPR.PSR] == -1)
                opt_debug = 0;
            else if (seg_debug[PPR.PSR] == 1)
                opt_debug = 1;
        }
}

//=============================================================================

void state_save()
{
    // Save CPU state so that state_dump_changes() can report on any interesting changes.

    hist_t *histp = &hist;

    histp->reg_A = reg_A;
    histp->reg_Q = reg_Q;
    histp->reg_E = reg_E;
    memcpy(histp->reg_X, reg_X, sizeof(histp->reg_X));
    memcpy(&histp->IR, &IR, sizeof(histp->IR));
    memcpy(histp->AR_PR, AR_PR, sizeof(histp->AR_PR));
    memcpy(&histp->PPR, &PPR, sizeof(histp->PPR));
    memcpy(&histp->TPR, &TPR, sizeof(histp->TPR));
    memcpy(&histp->cpu.DSBR, &cpup->DSBR, sizeof(histp->cpu.DSBR));
    histp->cu.SD_ON = cu.SD_ON;
    histp->cu.PT_ON = cu.PT_ON;
    memcpy(histp->cpu.SDWAM, cpup->SDWAM, sizeof(histp->cpu.SDWAM));
    memcpy(histp->cpu.PTWAM, cpup->PTWAM, sizeof(histp->cpu.PTWAM));

    if (memcmp(hist.AR_PR + 6, AR_PR + 6, sizeof(*hist.AR_PR)) != 0 || ! have_stack())
        state_invalidate_cache();
}

//=============================================================================

void state_dump_changes()
{
    // Track and dump any changes to any registers.  Called after each instruction
    // is executed.
    // TODO: Track some of the control-unit data

    log_msg(DEBUG_MSG, NULL, "\n", NULL);
    if (reg_A != hist.reg_A)
        log_msg(DEBUG_MSG, "HIST", "Reg A: %012llo\n", reg_A);
    if (reg_Q != hist.reg_Q) {
        if (reg_Q == calendar_q)
            log_msg(DEBUG_MSG, "HIST", "Reg Q: <calendar>\n");
        else
            log_msg(DEBUG_MSG, "HIST", "Reg Q: %012llo\n", reg_Q);
    }
    if (reg_E != hist.reg_E)
        log_msg(DEBUG_MSG, "HIST", "Reg E: %03o (%d)\n", reg_E, ((reg_E & (1<<7)) == 0) ? reg_E : (int) reg_E - 128);
    for (int i = 0; i < (int) ARRAY_SIZE(reg_X); ++i)
        if (reg_X[i] != hist.reg_X[i])
            log_msg(DEBUG_MSG, "HIST", "Reg X[%d]: %06o\n", i, reg_X[i]);
    if (memcmp(&hist.IR, &IR, sizeof(hist.IR)) != 0) {
        t_uint64 ir;
        save_IR(&ir);
        log_msg(DEBUG_MSG, "HIST", "IR: %s %s\n", bin2text(ir, 18), ir2text(&IR));
    }
    if (memcmp(hist.AR_PR, AR_PR, sizeof(hist.AR_PR)) != 0) {
        for (int i = 0; i < (int) ARRAY_SIZE(AR_PR); ++i)
            if (memcmp(hist.AR_PR + i, AR_PR + i, sizeof(*hist.AR_PR)) != 0) {
                log_msg(DEBUG_MSG, "HIST", "PR[%d]: rnr=%o, snr=%o, wordno=%0o, bitno=%#o; AR: bitno=%#o, charno=%#o\n",
                    i, AR_PR[i].PR.rnr, AR_PR[i].PR.snr, AR_PR[i].wordno, AR_PR[i].PR.bitno,
                    AR_PR[i].AR.bitno, AR_PR[i].AR.charno);
            }
    }
    if (memcmp(&hist.PPR, &PPR, sizeof(hist.PPR)) != 0)
        log_msg(DEBUG_MSG, "HIST", "PPR: PRR=%#o, PSR=%#o, P=%#o, IC=%#o\n", PPR.PRR, PPR.PSR, PPR.P, PPR.IC);
    if (memcmp(&hist.TPR, &TPR, sizeof(hist.TPR)) != 0) {
        if (TPR.is_value)
            log_msg(DEBUG_MSG, "HIST", "TPR: TRR=%#o, TSR=%#o, TBR=%#o, CA=%#o, is_value=Y, value=%#llo\n",
                TPR.TRR, TPR.TSR, TPR.TBR, TPR.CA, TPR.value);
        else
            log_msg(DEBUG_MSG, "HIST", "TPR: TRR=%#o, TSR=%#o, TBR=%#o, CA=%#o, is_value=N\n",
                TPR.TRR, TPR.TSR, TPR.TBR, TPR.CA);
    }
    if (memcmp(&hist.cpu.DSBR, &cpup->DSBR, sizeof(hist.cpu.DSBR)) != 0)
        log_msg(DEBUG_MSG, "HIST", "DSBR: addr=%#o, bound=%#o(%d), unpaged=%c, stack=%#o\n",
            cpup->DSBR.addr, cpup->DSBR.bound, cpup->DSBR.bound, cpup->DSBR.u ? 'Y' : 'N', cpup->DSBR.stack);
    if (hist.cu.PT_ON != cu.PT_ON)
        log_msg(DEBUG_MSG, "HIST", "PTWAM %s enabled\n", cu.PT_ON ? "is" : "is NOT");
    if (memcmp(hist.cpu.PTWAM, cpup->PTWAM, sizeof(hist.cpu.PTWAM)) != 0) {
        for (int i = 0; i < (int) ARRAY_SIZE(cpup->PTWAM); ++i) {
            uint tmp = hist.cpu.PTWAM[i].assoc.use;     // compare all members except "use" counter
            hist.cpu.PTWAM[i].assoc.use = cpup->PTWAM[i].assoc.use;
            if (memcmp(hist.cpu.PTWAM + i, cpup->PTWAM + i, sizeof(*hist.cpu.PTWAM)) != 0) {
                log_msg(DEBUG_MSG, "HIST", "PTWAM[%d]: ptr/seg = %#o, pageno=%#o, is_full/used=%c, use=%02o\n",
                    i, cpup->PTWAM[i].assoc.ptr, cpup->PTWAM[i].assoc.pageno, cpup->PTWAM[i].assoc.is_full ? 'Y' : 'N', cpup->PTWAM[i].assoc.use);
                log_msg(DEBUG_MSG, "HIST", "PTWAM[%d]: PTW: addr=%06oxx, used=%c, mod=%c, fault=%c, fc=%#o\n",
                    i, cpup->PTWAM[i].ptw.addr, cpup->PTWAM[i].ptw.u ? 'Y' : 'N', cpup->PTWAM[i].ptw.m ? 'Y' : 'N',
                    cpup->PTWAM[i].ptw.f ? 'Y' : 'N', cpup->PTWAM[i].ptw.fc);
            }
            hist.cpu.PTWAM[i].assoc.use = tmp;
        }
    }
    if (hist.cu.SD_ON != cu.SD_ON)
        log_msg(DEBUG_MSG, "HIST", "SDWAM %s enabled\n", cu.SD_ON ? "is" : "is NOT");
    if (memcmp(hist.cpu.SDWAM, cpup->SDWAM, sizeof(hist.cpu.SDWAM)) != 0) {
        for (int i = 0; i < (int) ARRAY_SIZE(cpup->SDWAM); ++i) {
            uint tmp = hist.cpu.SDWAM[i].assoc.use;     // compare all members except "use" counter
            hist.cpu.SDWAM[i].assoc.use = cpup->SDWAM[i].assoc.use;
            if (memcmp(hist.cpu.SDWAM + i, cpup->SDWAM + i, sizeof(*hist.cpu.SDWAM)) != 0) {
                log_msg(DEBUG_MSG, "HIST", "SDWAM[%d]: ptr(segno)=0%05o, is-full=%c, use=0%02o(%02d).\n",
                    i, cpup->SDWAM[i].assoc.ptr, cpup->SDWAM[i].assoc.is_full ? 'Y' : 'N',
                    cpup->SDWAM[i].assoc.use, cpup->SDWAM[i].assoc.use);
                SDW_t *sdwp = &cpup->SDWAM[i].sdw;
                log_msg(DEBUG_MSG, "HIST", "\tSDW for seg %d: addr = %#08o, r1=%o r2=%o r3=%o, f=%c, fc=%#o.\n",
                    cpup->SDWAM[i].assoc.ptr, sdwp->addr, sdwp->r1, sdwp->r2, sdwp->r3, sdwp->f ? 'Y' : 'N', sdwp->fc);
                log_msg(DEBUG_MSG, "HIST", "\tbound = %05o(%d), r=%c e=%c w=%c, priv=%c, unpaged=%c, g=%c, c=%c, cl=%05o\n",
                    sdwp->bound, sdwp->bound, sdwp->r ? 'Y' : 'N', sdwp->e ? 'Y' : 'N', sdwp->w ? 'Y' : 'N',
                    sdwp->priv ? 'Y' : 'N', sdwp->u ? 'Y' : 'N', sdwp->g ? 'Y' : 'N',
                    sdwp->c ? 'Y' : 'N', sdwp->cl);
            }
            hist.cpu.SDWAM[i].assoc.use = tmp;
        }
    }
}

//=============================================================================

void ic2text(char *icbuf, addr_modes_t addr_mode, uint seg, uint ic)
{
    // Format the IC according to the address mode

    if (addr_mode == ABSOLUTE_mode)
        sprintf(icbuf, "%06o", ic);
    else if (addr_mode == BAR_mode)
        sprintf(icbuf, "BAR %o|%06o", seg, ic);
    else
        sprintf(icbuf, "%o|%06o", seg, ic);
}

//=============================================================================

/*
    ic_hist - Circular queue of instruction history
    Used for display via cpu_show_history()
*/

// FIXME: Now that this is C++, we could switch to a STL container

static int ic_hist_max = 0;
static int ic_hist_ptr;
static int ic_hist_wrapped;
struct ic_hist_t {
    addr_modes_t addr_mode;
    uint seg;
    uint ic;
    enum hist_enum { instruction, fault, intr } htype;
    union {
        int intr;
        int fault;
        instr_t instr;
    } detail;
};
static ic_hist_t *ic_hist;

void ic_history_init()
{
    ic_hist_wrapped = 0;
    ic_hist_ptr = 0;
    if (ic_hist != NULL) 
        free(ic_hist);
    if (ic_hist_max < 60)
        ic_hist_max = 60;
    ic_hist = (ic_hist_t*) malloc(sizeof(*ic_hist) * ic_hist_max);
}

//=============================================================================

static ic_hist_t& ic_history_append()
{

    ic_hist_t& ret =  ic_hist[ic_hist_ptr];

    if (++ic_hist_ptr == ic_hist_max) {
        ic_hist_wrapped = 1;
        ic_hist_ptr = 0;
    }
    return ret;
}

//=============================================================================

// Maintain a queue of recently executed instructions for later display via cmd_dump_history()
// Caller should make sure IR and PPR are set to the recently executed instruction.

void ic_history_add()
{
    if (ic_hist_max == 0)
        return;
    ic_hist_t& hist = ic_history_append();
    hist.htype = ic_hist_t::instruction;
    hist.addr_mode = get_addr_mode();
    hist.seg = PPR.PSR;
    hist.ic = PPR.IC;
    memcpy(&hist.detail.instr, &cu.IR, sizeof(hist.detail.instr));
}

//=============================================================================

void ic_history_add_fault(int fault)
{
    if (ic_hist_max == 0)
        return;
    ic_hist_t& hist = ic_history_append();
    hist.htype = ic_hist_t::fault;
    hist.detail.fault = fault;
}

//=============================================================================

void ic_history_add_intr(int intr)
{
    if (ic_hist_max == 0)
        return;
    ic_hist_t& hist = ic_history_append();
    hist.htype = ic_hist_t::intr;
    hist.detail.intr = intr;
}

//=============================================================================

int cmd_dump_history(int32 arg, char *buf, int nshow)
    // Dumps the queue of instruction history
{
    if (ic_hist_max == 0) {
        out_msg("History is disabled.\n");
        return SCPE_NOFNC;
    }
    // The queue is implemented via an array and is circular,
    // so we make up to two passes through the array.
    int n = (ic_hist_wrapped) ? ic_hist_max : ic_hist_ptr;
    int n_ignore = (nshow < n) ? n - nshow : 0;
    for (int wrapped = ic_hist_wrapped; wrapped >= 0; --wrapped) {
        int start, end;
        if (wrapped) {
            start = ic_hist_ptr;
            end = ic_hist_max;
        } else {
            start = 0;
            end = ic_hist_ptr;
        }
        for (int i = start; i < end; ++i) {
            if (n_ignore-- > 0)
                continue;
            switch(ic_hist[i].htype) {
                case ic_hist_t::instruction: {
                    int segno = (ic_hist[i].addr_mode == APPEND_mode) ? (int) ic_hist[i].seg: -1;
                    print_src_loc("", ic_hist[i].addr_mode, segno, ic_hist[i].ic, &ic_hist[i].detail.instr);
                    break;
                }
                case ic_hist_t::fault:
                    out_msg("    Fault %#o (%d)\n", ic_hist[i].detail.fault, ic_hist[i].detail.fault);
                    break;
                case ic_hist_t::intr:
                    out_msg("    Interrupt %#o (%d)\n", ic_hist[i].detail.intr, ic_hist[i].detail.intr);
                    break;
            }
        }
    }
    return 0;
}

//=============================================================================

int cpu_show_history(FILE *st, UNIT *uptr, int val, void *desc)
{
    // FIXME: use FILE *st

    if (ic_hist_max == 0) {
        out_msg("History is disabled.\n");
        return SCPE_NOFNC;
    }

    char* cptr = (char *) desc;
    int n;
    if (cptr == NULL)
        n = ic_hist_max;
    else {
        char c;
        if (sscanf(cptr, "%d %c", &n, &c) != 1) {
            out_msg("Error, expecting a number.\n");
            return SCPE_ARG;
        }
    }

    return cmd_dump_history(0, NULL, n);
}

//=============================================================================

int cpu_set_history (UNIT *uptr, int32 val, char *cptr, void *desc)
{
    if (cptr == NULL) {
        out_msg("Error, usage is set cpu history=<n>\n");
        return SCPE_ARG;
    }
    char c;
    int n;
    if (sscanf(cptr, "%d %c", &n, &c) != 1) {
        out_msg("Error, expecting a number.\n");
        return SCPE_ARG;
    }

    if (n <= 0) {
        if (ic_hist != NULL)
            free(ic_hist);
        ic_hist = NULL;
        ic_hist_wrapped = 0;
        ic_hist_ptr = 0;
        ic_hist_max = 0;
        out_msg("History disabled\n");
        return 0;
    }

    ic_hist_t* new_hist = (ic_hist_t*) malloc(sizeof(*ic_hist) * n);

    int old_n;
    int old_head_loc;
    int old_nhead;  // amount at ptr..end
    int old_ntail;  // amount at 0..ptr (or all if never wrapped)
    int old_tail_loc = 0;
    if (ic_hist_wrapped)  {
        // data order is ptr..(max-1), 0..ptr-1
        old_n = ic_hist_max;
        old_head_loc = ic_hist_ptr;
        old_nhead = ic_hist_max - ic_hist_ptr;
        old_ntail = ic_hist_ptr;
    } else {
        // data is 0..(ptr-1)
        old_n = ic_hist_ptr;
        // old_head_loc = "N/A";
        old_head_loc = -123;
        old_nhead = 0;
        old_ntail = ic_hist_ptr;
    }
    int nhead = old_nhead;
    int ntail = old_ntail;
    if (old_n > n) {
        nhead -= old_n - n; // lose some of the earlier stuff
        if (nhead < 0) {
            // under flow, use none of ptr..end and lose some of 0..ptr
            ntail += nhead;
            old_tail_loc -= nhead;
            nhead = 0;
        } else
            old_head_loc += old_n - n;
    }
    if (nhead != 0)
        memcpy(new_hist, ic_hist + old_head_loc, sizeof(*new_hist) * nhead);
    if (ntail != 0)
        memcpy(new_hist + nhead, ic_hist, sizeof(*new_hist) * ntail);
    if (ic_hist != 0)
        free(ic_hist);
    ic_hist = new_hist;

    if (n <= old_n)  {
        ic_hist_ptr = 0;
        ic_hist_wrapped = 1;
    } else {
        ic_hist_ptr = old_n;
        ic_hist_wrapped = 0;
    }

    if (n >= ic_hist_max)
        if (ic_hist_max == 0)
            out_msg("History enabled.\n");
        else
            out_msg("History increased from %d entries to %d.\n", ic_hist_max, n);
    else
        out_msg("History reduced from %d entries to %d.\n", ic_hist_max, n);

    ic_hist_max = n;

    return 0;
}

//=============================================================================

static void print_src_loc(const char *prefix, addr_modes_t addr_mode, int segno, int ic, const instr_t* instrp)
{
    char icbuf[80];
    ic2text(icbuf, addr_mode, segno, ic);

    where_t where;
    if (seginfo_find_all(segno, ic, &where) != 0)
        out_msg("%sIC: %s: %-60s\n", prefix, icbuf, instr2text(instrp));
    else {
        const char *name = where.entry ? where.entry : where.file_name ? where.file_name : "unknown";
        if (where.line_no >= 0) {
            out_msg("%sIC: %s: %-60s %s, line %d\n", prefix, icbuf, instr2text(instrp), name, where.line_no);
            out_msg("%s\tline %d: %s\n", prefix, where.line_no, where.line);
        } else {
            if (where.entry_offset < 0)
                out_msg("%sIC: %s: %-60s %s\n", prefix, icbuf, instr2text(instrp), name);
            else {
                int offset = (int) ic - where.entry_offset;
                char sign = (offset < 0) ? '-' : '+';
                if (sign == '-')
                    offset = - offset;
                out_msg("%sIC: %s: %-60s %s %c%#o\n", prefix, icbuf, instr2text(instrp), name, sign, offset);
            }
        }
    }
}

//=============================================================================


int show_location(int show_source_lines)
    // Called by the CPU.   Decides if we need to display the current IC and/or
    // source filenames, entry point names, etc.   If so, displays them.
    // Returns zero if any entrypoint of source line info was displayed
{

    // WARNING: re-init the following two varibles if (re)init() is ever implemented for symtab pkg
    static int have_source = 0;
    static where_t where;
    static int prev_segno = -1;

    static const char *old;
    static int old_line_no = -1;

    // Segment 0400 is the first used segment and shows a lot of bouncing between source
    // files that's no longer of interest   
    const int show_source_changes = PPR.PSR != 0400 || prev_segno != 0400;
    //const int show_source_changes = 0;

    static int seg_scanned[512];
    if (! opt_debug && ! show_source_changes && ! show_source_lines)
        return 1;

    // Scan segments for entry points to procedures
    addr_modes_t amode = get_addr_mode();
    int segno = (amode == APPEND_mode) ? (int) PPR.PSR : -1;
    if (segno >= 0 && segno < (int) ARRAY_SIZE(seg_scanned) && ! seg_scanned[segno]) {
        scan_seg(segno, 0);
        seg_scanned[segno] = 1;
    }

    if (opt_debug)
        log_msg(DEBUG_MSG, NULL, "\n", NULL);

    int display_file = 0;

    // Did we just change from one source file or procedure to another?
    int source_changed;
    if (have_source) {
        source_changed = prev_segno != segno || where.entry_offset > PPR.IC || where.entry_hi < 0 || PPR.IC > where.entry_hi;
        if (source_changed) {
            where_t owhere = where;
            have_source = seginfo_find_all(segno, PPR.IC, &where) == 0;
            if (have_source) {
                display_file = where.file_name != owhere.file_name;
                source_changed = where.file_name != owhere.file_name || where.entry != owhere.entry;
            } else {
                source_changed = 1;
                // log_msg(INFO_MSG, "MAIN", "src changed: lost source on %#o|%#o\n", segno, PPR.IC);
            }
        } else {
            have_source = seginfo_find_all(segno, PPR.IC, &where) == 0;
            if (! have_source) {
                source_changed = 1;
                // log_msg(INFO_MSG, "MAIN", "src changed: lost source (but within prior range?)\n");
            }
        }
    } else {
        source_changed = have_source = seginfo_find_all(segno, PPR.IC, &where) == 0;
        display_file = source_changed;
    }

    prev_segno = segno;

    // Display source line
    int display_entry = 0;
    int display_line = 0;
    if (show_source_lines && (opt_debug || cpu.cycle != FETCH_cycle)) {
        display_entry = source_changed;
        if (where.line_no >= 0)
            display_line = source_changed || old_line_no != where.line_no;
        old_line_no = where.line_no;
    }
    
    int ret = 1;
    if (display_entry) {
        if (where.entry) {
            if (display_file && display_line && where.file_name)
                log_msg(INFO_MSG, NULL, "Source: %s\n", where.file_name);
            log_msg(DEBUG_MSG, "MAIN", "%s: %s\n", "Procedure", where.entry);
            ret = 0;
        } else if (where.file_name)
            log_msg(DEBUG_MSG, "MAIN", "%s: %s\n", "Source file", where.file_name);
        else
            log_msg(DEBUG_MSG, "MAIN", "Source unknown\n");
    }

    if (display_line && where.line_no >= 0) {
        // Note that if we have a source line, we also expect to have a "proc" entry
        const char *name = where.entry ? where.entry : where.file_name;
        // log_msg(INFO_MSG, NULL, "Source:  %3o|%06o, line %5d: < %s\n", segno, PPR.IC, where.line_no, where.line);
        int depth = stack_depth();
        log_msg(INFO_MSG, NULL, "Source:  %3o|%06o, line %5d(%d): < %s\n", segno, PPR.IC, where.line_no, depth, where.line);
        ret = 0;
    }

    // Display IC
    if (opt_debug || (source_changed && show_source_changes)) {
        const char *name = where.entry ? where.entry : where.file_name;
        char icbuf[80];
        ic2text(icbuf, amode, PPR.PSR, PPR.IC);
        if (source_changed) {
            if (have_source) {
                if (! where.entry || where.entry_offset < 0 || where.entry_offset == (int) PPR.IC) {
                    if (name == NULL)
                        log_msg(WARN_MSG, "MAIN", "name is null; offset = %#o; e-name = %s, f-name = %s.\n", where.entry_offset, where.entry, where.file_name); // impossible
                    log_msg(INFO_MSG, "MAIN", "IC: %s\tSource: %s\n", icbuf, name);
                } else {
                    int offset = (int) PPR.IC - where.entry_offset;
                    char sign = (offset < 0) ? '-' : '+';
                    if (sign == '-')
                        offset = - offset;
                    log_msg(INFO_MSG, "MAIN", "IC: %s\tSource: %s %c%#o\n", icbuf, name, sign, offset);
                }
            } else if (old != NULL)
                log_msg(INFO_MSG, "MAIN", "IC: %s\tSource: Unknown (leaving %s)\n", icbuf, old);
            old = name;
        } else
            if (opt_debug)
                log_msg(DEBUG_MSG, "MAIN", "IC: %s\n", icbuf);  // source unchanged
    }

#if 0
    cout << flush;
    clog << flush;
    // cdebug << flush;
    cerr << flush;
#endif
    log_any_io(0);      // Output of source/location info doesn't count towards requiring re-display of source

    return ret;
}

//=============================================================================

int cmd_xdebug(int32 arg, char *buf)
{
    if (!seg_debug_init_done)
        init_seg_debug();

    char *s = buf;
    s += strspn(s, " \t");
    if (*s == 0) {
        out_msg("USAGE xdebug seg <segment number> { on | off }\n");
        return 1;
    }

    if (strncmp(s, "seg", strlen("seg")) == 0)
        s += strlen("seg");
    else if (strncmp(s, "segment", strlen("segment")) == 0)
        s += strlen("segment");
    else {
        out_msg("xdebug: expecting the word 'seg' or 'segment'\n");
        return 1;
    }
    s += strspn(s, " \t");
    unsigned segno;
    char c;
    int n;
    if (sscanf(s, "%o", &segno) != 1) {
        out_msg("xdebug: Expecting a octal segment number.\n");
        return 1;
    }
    s += strspn(s, "01234567 \t");
    int state;
    if (strcmp(s, "on") == 0)
        state = 1;
    else if (strcmp(s, "cpu") == 0 || strcmp(s, "default") == 0 || strcmp(s, "def") == 0)
        state = 0;
    else if (strcmp(s, "off") == 0)
        state = -1;
    else {
        out_msg("xdebug: Expecting 'on', 'off', or 'cpu', not: %s\n", s);
        return 1;
    }

    if (segno >= n_segments) {
        out_msg("xdebug: Maximum segment number is %#o\n", n_segments - 1);
        return 1;
    }

    seg_debug[segno] = state;
    return 0;
}

//=============================================================================

char *ir2text(const IR_t *irp)
{
    static char buf[256];

    char *s = buf;
    *s++ = '[';

    if (irp->zero) { strcpy(s, " zero"); s += strlen(s); }
    if (irp->neg) { strcpy(s, " neg"); s += strlen(s); }
    if (irp->carry) { strcpy(s, " carry"); s += strlen(s); }
    if (irp->overflow) { strcpy(s, " overflow"); s += strlen(s); }
    if (irp->exp_overflow) { strcpy(s, " exp-overflow"); s += strlen(s); }
    if (irp->exp_underflow) { strcpy(s, " exp-underflow"); s += strlen(s); }
    if (irp->overflow_mask) { strcpy(s, " overflow-mask"); s += strlen(s); }
    if (irp->tally_runout) { strcpy(s, " tally-run-out"); s += strlen(s); }
    if (irp->parity_error) { strcpy(s, " parity-error"); s += strlen(s); }
    if (irp->parity_mask) { strcpy(s, " parity-mask"); s += strlen(s); }
    if (irp->not_bar_mode) { strcpy(s, " not-bar-mode"); s += strlen(s); }
    if (irp->truncation) { strcpy(s, " truncation"); s += strlen(s); }
    if (irp->mid_instr_intr_fault) { strcpy(s, " mid-instr-intr-fault"); s += strlen(s); }
    if (irp->abs_mode) { strcpy(s, " abs-mode"); s += strlen(s); }
    if (irp->hex_mode) { strcpy(s, " hex-mode"); s += strlen(s); }
    strcpy(s, " ]");
    return buf;
}

//=============================================================================

static int stack_to_entry(unsigned abs_addr, AR_PR_t* prp)
{
    // Looks into the stack frame maintained by Multics and
    // returns the "current" entry point address that's
    // recorded in the stack frame.  Abs_addr should be a 24-bit
    // absolute memory location.
    return words2its(Mem[abs_addr+026], Mem[abs_addr+027], prp);
}

#if 0
static int stack_to_entry(int segno, unsigned offset, AR_PR_t* prp)
{
    // See description above

    unsigned framep;
    if (convert_address(&framep, segno, offset, 0) != 0) {
        log_msg(NOTIFY_MSG, "STACK", "Cannot convert stack pointer address %03o|%06o to an absolute memory address.\n", segno, offset);
        return -1;
    }
    return stack_to_entry(framep, prp);
}
#endif

#if 0
static int stack_to_entry(const seg_addr_t& addr, AR_PR_t* prp)
{
    return stack_to_entry(addr.segno, addr.offset, prp);
}
#endif

//=============================================================================

static void print_frame(
    int seg,        // Segment portion of frame pointer address.
    int offset,     // Offset portion of frame pointer address.
    int addr)       // 24-bit address corresponding to above seg|offset
{
    // Print a single stack frame for walk_stack()
    // Frame pointers can be found in PR[6] or by walking a process's stack segment

    AR_PR_t entry_pr;
    out_msg("stack trace: ");
    if (stack_to_entry(addr, &entry_pr) == 0) {
        const seginfo& seg = segments(entry_pr.PR.snr);
        map<int,linkage_info>::const_iterator li_it = seg.find_entry(entry_pr.wordno);
        if (li_it != seg.linkage.end()) {
            const linkage_info& li = (*li_it).second;
            out_msg("\t%s  ", li.name.c_str());
        } else
            out_msg("\tUnknown entry %o|%o  ", entry_pr.PR.snr, entry_pr.wordno);
    } else
        out_msg("\tUnknowable entry {%llo,%llo}  ", Mem[addr+026], Mem[addr+027]);
    out_msg("(stack frame at %03o|%06o)\n", seg, offset);

#if 0
    char buf[80];
    out_msg("prev_sp: %s; ",    its2text(buf, addr+020));
    out_msg("next_sp: %s; ",    its2text(buf, addr+022));
    out_msg("return_ptr: %s; ", its2text(buf, addr+024));
    out_msg("entry_ptr: %s\n",  its2text(buf, addr+026));
#endif
}

//=============================================================================

int cmd_stack_trace(int32 arg, char *buf)
{
    walk_stack(1, NULL);
    frame_trace();
    out_msg("stack trace:\n");
    dump_autos();
    out_msg("\n");

    float secs = (float) sys_stats.total_msec / 1000;
    out_msg("Stats: %.1f seconds: %lld cycles at %.0f cycles/sec, %lld instructions at %.0f instr/sec\n",
        secs, sys_stats.total_cycles, sys_stats.total_cycles/secs, sys_stats.total_instr, sys_stats.total_instr/secs);

    return 0;
}

//=============================================================================

int cpu_show_stack(FILE *st, UNIT *uptr, int val, void *desc)
{
    // FIXME: use FILE *st
    return cmd_stack_trace(0, NULL);
}

//=============================================================================

static int walk_stack(int output, list<seg_addr_t>* frame_listp)
    // Trace through the Multics stack frames
    // See stack_header.incl.pl1 and http://www.multicians.org/exec-env.html
{
    const char* moi = "STACK::walk";

    if (AR_PR[6].PR.snr == 077777 || (AR_PR[6].PR.snr == 0 && AR_PR[6].wordno == 0)) {
        log_msg(INFO_MSG, moi, "Null PR[6]\n");
        return 1;
    }

    // PR6 should point to the current stack frame.  That stack frame
    // should be within the stack segment.
    int seg = AR_PR[6].PR.snr;

    uint curr_frame;
    if (convert_address(&curr_frame, seg, AR_PR[6].wordno, 0) != 0) {
        log_msg(INFO_MSG, moi, "Cannot convert PR[6] == %#o|%#o to absolute memory address.\n",
            AR_PR[6].PR.snr, AR_PR[6].wordno);
        return 1;
    }

    // The stack header will be at offset 0 within the stack segment.
    int offset = 0;
    uint hdr_addr;  // 24bit main memory address
    if (convert_address(&hdr_addr, seg, offset, 0) != 0) {
        log_msg(INFO_MSG, moi, "Cannot convert %03o|0 to absolute memory address.\n", seg);
        return 1;
    }
    AR_PR_t stack_begin_pr;
    if (words2its(Mem[hdr_addr+022], Mem[hdr_addr+023], &stack_begin_pr) != 0) {
        log_msg(INFO_MSG, moi, "Stack header seems invalid; no stack_begin_ptr at %03o|22\n", seg);
        if (output)
            out_msg("Stack Trace: Stack header seems invalid; no stack_begin_ptr at %03o|22\n", seg);
        return 1;
    }
    AR_PR_t stack_end_pr;
    if (words2its(Mem[hdr_addr+024], Mem[hdr_addr+025], &stack_end_pr) != 0) {
        //if (output)
            out_msg("Stack Trace: Stack header seems invalid; no stack_end_ptr at %03o|24\n", seg);
        return 1;
    }
    if (stack_begin_pr.PR.snr != seg || stack_end_pr.PR.snr != seg) {
        //if (output)
            out_msg("Stack Trace: Stack header seems invalid; stack frames are in another segment.\n");
        return 1;
    }
    AR_PR_t lot_pr;
    if (words2its(Mem[hdr_addr+026], Mem[hdr_addr+027], &lot_pr) != 0) {
        //if (output)
            out_msg("Stack Trace: Stack header seems invalid; no LOT ptr at %03o|26\n", seg);
        return 1;
    }
    // TODO: sanity check LOT ptr

    if (output)
        out_msg("Stack Trace:\n");
    int framep = stack_begin_pr.wordno;
    int prev = 0;
    int finished = 0;
#if 0
    int need_hist_msg = 0;
#endif
    // while(framep <= stack_end_pr.wordno)
    for (;;) {
        // Might find ourselves in a different page while moving from frame to frame...
        // BUG: We assume a stack frame doesn't cross page boundries
        uint addr;
        if (convert_address(&addr, seg, framep, 0) != 0) {
            if (finished)
                break;  
            //if (output)
                out_msg("STACK Trace: Cannot convert address of frame %03o|%06o to absolute memory address.\n", seg, framep);
            return 1;
        }
        // Sanity check
        if (prev != 0) {
            AR_PR_t prev_pr;
            if (words2its(Mem[addr+020], Mem[addr+021], &prev_pr) == 0) {
                if (prev_pr.wordno != prev) {
                    if (output)
                        out_msg("STACK Trace: Stack frame's prior ptr, %03o|%o is bad.\n", seg, prev_pr.wordno);
                }
            }
        }
        prev = framep;
        // Print the current frame
        if (finished && Mem[addr+022] == 0 && Mem[addr+024] == 0 && Mem[addr+026] == 0)
            break;
#if 0
        if (need_hist_msg) {
            need_hist_msg = 0;
            out_msg("stack trace: ");
            out_msg("Recently popped frames (aka where we recently returned from):\n");
        }
#endif
        if (output)
            print_frame(seg, framep, addr);
        if (frame_listp)
            (*frame_listp).push_back(seg_addr_t(seg, framep));
        // Get the next one
        AR_PR_t next;
        if (words2its(Mem[addr+022], Mem[addr+023], &next) != 0) {
            if (!finished)
                if (output)
                    out_msg("STACK Trace: no next frame.\n");
            break;
        }
        if (next.PR.snr != seg) {
            if (output)
                out_msg("STACK Trace: next frame is in a different segment (next is in %03o not %03o.\n", next.PR.snr, seg);
            break;
        }
        if (next.wordno == stack_end_pr.wordno) {
            finished = 1;
            break;
#if 0
            need_hist_msg = 1;
            if (framep != AR_PR[6].wordno)
                out_msg("Stack Trace: Stack may be garbled...\n");
            // BUG: Infinite loop if enabled and garbled stack with "Unknowable entry {0,0}", "Unknown entry 15|0  (stack frame at 062|000000)", etc
#endif
        }
        if (next.wordno < stack_begin_pr.wordno || next.wordno > stack_end_pr.wordno) {
            if (!finished)
                //if (output)
                    out_msg("STACK Trace: DEBUG: next frame at %#o is outside the expected range of %#o .. %#o for stack frames.\n", next.wordno, stack_begin_pr.wordno, stack_end_pr.wordno);
            if (! output)
                return 1;
        }

        // Use the return ptr in the current frame to print the source line.
        if (! finished && output) {
            AR_PR_t return_pr;
            if (words2its(Mem[addr+024], Mem[addr+025], &return_pr) == 0) {
                where_t where;
                int offset = return_pr.wordno;
                if (offset > 0)
                    -- offset;      // call was from an instr prior to the return point
                if (seginfo_find_all(return_pr.PR.snr, offset, &where) == 0) {
                    out_msg("stack trace: ");
                    if (where.line_no >= 0) {
                        // Note that if we have a source line, we also expect to have a "proc" entry and file name
                        out_msg("\t\tNear %03o|%06o in %s\n",
                            return_pr.PR.snr, return_pr.wordno, where.entry);
                        // out_msg("\t\tSource:  %s, line %5d:\n", where.file_name, where.line_no);
                        out_msg("stack trace: ");
                        out_msg("\t\tLine %d of %s:\n", where.line_no, where.file_name);
                        out_msg("stack trace: ");
                        out_msg("\t\tSource:  %s\n", where.line);
                    } else
                        if (where.entry_offset < 0)
                            out_msg("\t\tNear %03o|%06o", return_pr.PR.snr, return_pr.wordno);
                        else {
                            int off = return_pr.wordno - where.entry_offset;
                            char sign = (off < 0) ? '-' : '+';
                            if (sign == '-')
                                off = - off;
                            out_msg("\t\tNear %03o|%06o %s %c%#o\n", return_pr.PR.snr, return_pr.wordno, where.entry, sign, off);
                        }
                }
            }
        }
        // Advance
        framep = next.wordno;
    }

    if (output) {
        out_msg("stack trace: ");
        out_msg("Current Location:\n");
        out_msg("stack trace: ");
        print_src_loc("\t", get_addr_mode(), PPR.PSR, PPR.IC, &cu.IR);

        log_any_io(0);      // Output of source/location info doesn't count towards requiring re-display of source
    }

    return 0;
}

//=============================================================================

// option: instead of doing change detection, register breakpoint/callback

class multics_stack_frame {
public:
    typedef enum { uninitialized, unchanged, initial_change, changed } change_t;
    class val_t {
        // Values of automatics in this frame; see the entry() for names and offsets
        private:
            int _initialized;   // constructor sets _initialized to false, but
            t_uint64 _val;      // also sets _val to a copy of what's in memory
        public:
            val_t(t_uint64 v) { _val = v; _initialized = 0; }
            int is_initialized() const { return _initialized; }
            t_uint64 val() { if (!_initialized) throw logic_error("uninitialized"); return _val; }
            change_t change(t_uint64 currval);
    };
private:
    int _segno;                 // Segno of stack segment (from PR[6])
    int _offset;                // offset of this frame within segno (from PR[6])
    int _addr;                  // 24-bit absolute address corresponding to above SP|offset
    const linkage_info* _linkage;   // Needed for entry_point with names of automatic variables, etc
    map <int, val_t> vals;      // Values of automatics w/o names; Key is stack offset
    int _all_initialized;
    int _refresh_all(int first);
public:
    // Iterators for values of automatics
    map<int, val_t>::iterator begin() { return vals.begin(); }
    map<int, val_t>::iterator end() { return vals.end(); }
public:
    multics_stack_frame(int segno, int offset);
    void set_linkage(const linkage_info* lip);
    int offset() const { return _offset; }
    int addr();
    int addr() const;
    const int size() const
        { return vals.size(); }
    const linkage_info* linkage() const
        { return _linkage; }
    const entry_point* entry() const
        { return (_linkage) ? _linkage->entry : NULL; }
public:
    void update_autos(int segno, int ic);   // checks & records changes to automatics in this frame
    void dump_autos() const;
#if 1
    void show(int all=1) const {};
#else
    void show(int all=1);   // debugging
private:
    class debug_t {
        public:
        seg_addr_t prev_sp; // at 020
        seg_addr_t next_sp; // at 022
        seg_addr_t returnp; // at 024
        seg_addr_t entry;   // at 026
        debug_t()
            : prev_sp(0,0), next_sp(0,0), returnp(0,0), entry(0,0) {}
    };
    debug_t debug;
#endif
};


class multics_stack {
private:
    int _segno;             // stack frame segno (from PR[6])
    list<multics_stack_frame> _frames;
    class partial {
        public:
        // Info about the latest partially discovered frame
        int finished;       // Have we finished finding out about the new frame yet?
        seg_addr_t bumper;  // PTR to instruction that bumped PR[6]
        seg_addr_t entryp;  // Value of entryp in frame at time of stack bump
        partial()
            : finished(0), bumper(0,0), entryp(0,0) {}
    } partial;
public:
    multics_stack(int segno);
    int segno() const { return _segno; }
public:
    // Frames
    int size() const { return _frames.size(); }
    int push(); // push(int offset, const linkage_info* lip);
    int pop();
    const list<multics_stack_frame>& frames() const
        { return _frames; }
    list<multics_stack_frame>::iterator begin()
        { return _frames.begin(); }
    list<multics_stack_frame>::iterator end()
        { return _frames.end(); }
    multics_stack_frame& back()
        { return _frames.back(); }
public:
    int change();           // adjust stack to match observed PR[6]
    void check_frame();     // see if a new under-construction frame has info we need
    int is_in_frame(int segno, int offset, int debug=0);    // returns true if given IC is within top frame's entry
    int unfinished() const { return ! partial.finished; }
};


static multics_stack* m_stackp = NULL;

//=============================================================================

multics_stack::multics_stack(int segno)
    : _segno(segno)
{
    log_msg(NOTIFY_MSG, "STACK", "Multics procedure stack now live.\n");
}

//=============================================================================

// TODO: rename
void state_invalidate_cache()
{
    if (AR_PR[6].PR.snr == 077777 || AR_PR[6].wordno == 0)
        return;

    if (m_stackp == NULL) {
        // log_msg(INFO_MSG, "STACK::invalidate", "Checking frame %#o|%#o.\n", AR_PR[6].PR.snr, AR_PR[6].wordno);
        if (is_stack_frame(AR_PR[6].PR.snr, AR_PR[6].wordno)) {
            log_msg(NOTIFY_MSG, "STACK::invalidate", "Initializing stack with %#o|%#o.\n",
                AR_PR[6].PR.snr, AR_PR[6].wordno);
            m_stackp = new multics_stack(AR_PR[6].PR.snr);
            m_stackp->change();
            // cancel_run(STOP_IBKPT);
        } else {
            // log_msg(NOTIFY_MSG, "STACK::invalidate", "No stack and PR[6] does not appear to point to a frame.\n");
        }
    } else {
        m_stackp->change();
    }
}

//=============================================================================

static int have_stack()
{
    return m_stackp != NULL && m_stackp->size() != 0;
}

static int stack_depth()
{
    if (m_stackp == NULL)
        return -1;
    else
        return m_stackp->size();
}

//=============================================================================

#if 0
static int its2pr(uint addr, seg_addr_t& a)
{
    AR_PR_t pr;
    if (words2its(Mem[addr], Mem[addr+1], &pr) == 0) {
        a = seg_addr_t(pr.PR.snr, pr.wordno);
        return 0;
    } else
        return 1;
}
#endif

static char* its2text(uint addr)
{
    AR_PR_t pr;
    static char buf[80];
    if (words2its(Mem[addr], Mem[addr+1], &pr) == 0) {
        sprintf(buf, "%#o|%#o", pr.PR.snr, pr.wordno);
    } else {
        sprintf(buf, "{ %#llo, %#llo }", Mem[addr], Mem[addr+1]);
    }
    return buf;
}


static int is_stack_frame(int segno, int offset)
{
    const char* moi = "STACK::is_stack";

    if (segno == 077777)
        return 0;
    if (offset == 0)
        return 0;   // stack header starts at zero; frames are after that

    uint hdr_addr;
    if (convert_address(&hdr_addr, segno, 0, 0) != 0)
        return 0;

    AR_PR_t pr;
    if (words2its(Mem[hdr_addr+026], Mem[hdr_addr+027], &pr) != 0) {
        // log_msg(INFO_MSG, moi, "Segment %#o does not have a valid header; no LOT ptr at offset 26.\n", segno);
        return 0;
    }
    if (pr.PR.snr != 015) {
        // log_msg(INFO_MSG, moi, "Segment %#o does not have a valid header; LOT ptr is not in segment 015.\n");
        return 0;
    }
    if (words2its(Mem[hdr_addr+022], Mem[hdr_addr+023], &pr) != 0) {
        // log_msg(INFO_MSG, moi, "Segment %#o does not have a valid header; no frame ptr at offset 22.\n", segno);
        return 0;
    }
    if (pr.PR.snr != segno) {
        log_msg(INFO_MSG, moi, "Segment %#o does not have a valid header; first frame ptr is outside the segment.\n", segno);
        return 0;
    }
    log_msg(INFO_MSG, moi, "Offset(020): null ptr %s\n", its2text(hdr_addr+020));
    log_msg(INFO_MSG, moi, "Offset(022): stack begin %s\n", its2text(hdr_addr+022));
    log_msg(INFO_MSG, moi, "Offset(024): stack end %s\n", its2text(hdr_addr+024));
    log_msg(INFO_MSG, moi, "Offset(026): lot ptr %s\n", its2text(hdr_addr+026));

    uint addr;
    if (convert_address(&addr, segno, offset, 0) != 0)
        return 0;

    return 1;
}

//=============================================================================

static void frame_trace()
{
    if (m_stackp == NULL) {
        out_msg("No stack.\n");
        return;
    }
    out_msg("trace of PR[6] stack history (%d entries):\n", m_stackp->size());

    list<multics_stack_frame>::const_iterator stackp;
    for (stackp = m_stackp->begin(); stackp != m_stackp->end(); ++ stackp) {
        const multics_stack_frame& msf = (*stackp);
        out_msg("\t%06o:  ", msf.offset());
        if (msf.linkage()) {
            if (msf.entry())
                if (msf.linkage()->name != msf.entry()->name) {
                    // should be impossible
                    out_msg("%s -> ", msf.linkage()->name.c_str(), msf.entry()->name.c_str());
                } else
                    out_msg("%s", msf.linkage()->name.c_str());
            else
                out_msg("%s", msf.linkage()->name.c_str());
        } else
            out_msg("unknown procedure");
        if (msf.entry() != NULL && msf.entry()->stack_owner != NULL)
            out_msg(" (stack owner %s)", (msf.entry()->stack_owner->name).c_str());
        out_msg("\n");
    }

}

//=============================================================================

multics_stack_frame::multics_stack_frame(int segno, int offset)
        : _segno(segno), _offset(offset), _linkage(NULL)
{

    // log_msg(INFO_MSG, "STACK::frame::init", "Creating frame for offset = %#o\n", offset);

    // WARNING -- we assume our stack never gets relocated to a different absolute address
    _addr = -1;
    _all_initialized = 0;

    if (addr() == -1)
        return;
#if 0
    memset(&debug, 0, sizeof(debug));
    (void) its2pr(_addr + 020, debug.prev_sp);
    (void) its2pr(_addr + 022, debug.next_sp);
    (void) its2pr(_addr + 024, debug.returnp);
    (void) its2pr(_addr + 026, debug.entry);
#endif
    show(1);
}

//=============================================================================

void multics_stack_frame::set_linkage(const linkage_info* lip)
{
    const char* moi = "STACK::frame::set";
    vals.clear();
    _all_initialized = 0;

    if (lip == NULL)
        return;
    _linkage = lip;

    log_msg(INFO_MSG, moi, "Frame %#o: li owner is %s\n", _offset, _linkage->name.c_str());

    if (entry() == NULL)
        return;

    // We loaded a source listing for this entry point

    if (_linkage->name != entry()->name)
        log_msg(WARN_MSG, moi, "Linkage %s name does not match entry name %s\n",
            _linkage->name.c_str(), entry()->name.c_str());

    const stack_frame* sfp = entry()->stack();
    if (sfp) {
        map<int,val_t>::iterator it = vals.end();
        for (map<int,var_info>::const_iterator autos_it = sfp->automatics.begin(); autos_it != sfp->automatics.end(); ++ autos_it) {
            int soffset = (*autos_it).first;
            t_uint64 curr = Mem[_addr+soffset];
            it = vals.insert(it, pair<int,val_t>(soffset, val_t(curr)));
        }
    } else {
        // We loaded entry point info from a source listing, but
        // didn't get any info re automatics in the stack
    }
}

//=============================================================================

#if 0
void multics_stack_frame::show(int all)
{
    (void) addr();

    seg_addr_t prev_sp(0,0);
    seg_addr_t next_sp(0,0);
    seg_addr_t returnp(0,0);
    seg_addr_t entry(0,0);
    (void) its2pr(_addr + 020, prev_sp);
    (void) its2pr(_addr + 022, next_sp);
    (void) its2pr(_addr + 024, returnp);
    (void) its2pr(_addr + 026, entry);

    int hdr = 0;
    if (all) {
        if (!hdr) log_msg(INFO_MSG, "STACK::frame", "Frame at SP offset %06o (%08o absolute):\n", _offset, _addr);
        hdr = 1;
    }

    if (all || prev_sp != debug.prev_sp) {
        if (!hdr) log_msg(INFO_MSG, "STACK::frame", "Frame at SP offset %06o (%08o absolute):\n", _offset, _addr);
        if (!hdr) cancel_run(STOP_IBKPT);
        hdr = 1;
        log_msg(INFO_MSG, "STACK::frame", "offset 020: prev_sp: %s\n",    its2text(_addr+020));
        debug.prev_sp = prev_sp;
    }

    if (all || next_sp != debug.next_sp) {
        if (!hdr) log_msg(INFO_MSG, "STACK::frame", "Frame at SP offset %06o (%08o absolute):\n", _offset, _addr);
        if (!hdr) cancel_run(STOP_IBKPT);
        hdr = 1;
        log_msg(INFO_MSG, "STACK::frame", "offset 022: next_sp: %s\n",    its2text(_addr+022));
        debug.next_sp = next_sp;
    }

    if (all || returnp != debug.returnp) {
        if (!hdr) log_msg(INFO_MSG, "STACK::frame", "Frame at SP offset %06o (%08o absolute):\n", _offset, _addr);
        if (!hdr) cancel_run(STOP_IBKPT);
        hdr = 1;
        log_msg(INFO_MSG, "STACK::frame", "offset 024: return_ptr: %s\n", its2text(_addr+024));
        debug.returnp = returnp;
    }

    if (all || entry != debug.entry) {
        if (!hdr) log_msg(INFO_MSG, "STACK::frame", "Frame at SP offset %06o (%08o absolute):\n", _offset, _addr);
        if (!hdr) cancel_run(STOP_IBKPT);
        hdr = 1;
        log_msg(INFO_MSG, "STACK::frame", "offset 026: entry_ptr: %s\n",  its2text(_addr+026));
        debug.entry = entry;
    }
}
#endif

//=============================================================================

int multics_stack_frame::addr() const
{
    if (_addr != -1)
        return _addr;
    uint a;
    if (convert_address(&a, _segno, _offset, 0) == 0)
        return a;
    return -1;
}

int multics_stack_frame::addr()
{
    if (_addr == -1) {
        uint a;
        if (convert_address(&a, _segno, _offset, 0) == 0)
            _addr = a;
    }
    return _addr;
}

//=============================================================================

int multics_stack::change()
{
    // adjust stack to match observed PR[6]

    if (AR_PR[6].PR.snr != _segno) {
        log_msg(NOTIFY_MSG, "STACK::change", "Internal error, PR[6] is seg %#o, but stack seg is %#o.\n", AR_PR[6].PR.snr, _segno);
        return 1;
    }
    int offset = AR_PR[6].wordno;

    int npop;
    for (npop = 0; size() > 0 && back().offset() > offset; ++npop)
        pop();
    if (npop > 1)
        log_msg(WARN_MSG, "STACK::change", "Popped %d frames.\n", npop);
    else
        if (npop == 1)
            log_msg(INFO_MSG, "STACK::change", "Popped a frame.\n");

    // if (npop > 0) cancel_run(STOP_IBKPT);
    if (size() == 0) {
        log_msg(INFO_MSG, "STACK::change", "Pushing inital frame.\n");
        return push();
    }
    if (back().offset() == offset) {
        if (! partial.finished)
            check_frame();
        back().show(0);
        return 0;
    }
    log_msg(INFO_MSG, "STACK::change", "Pushing new frame.\n");
    return push();
}

//=============================================================================

int multics_stack::pop()
{
    _frames.pop_back();
    return 0;
}

//=============================================================================

int multics_stack::push()
{
    if (AR_PR[6].PR.snr != _segno) {
        log_msg(INFO_MSG, "STACK", "Internal error, line %d\n", __LINE__);
        return 1;
    }
    int sp_offset = AR_PR[6].wordno;

    // Find linkage
    // Problem:
    // When the SP is bumped, we're probably executing in pl1_operators_$entry_operators, not
    // in the caller or the callee.  Also, the entry_ptr probably hasn't been written into the
    // frame yet.  Our choices are to "know" that the entry pointer is in PR[2] or to wait
    // for later instructions to either update entry_ptr or to move well past the location
    // where the SP was bumped.  We'll do the later.

    // const linkage_info *lip = NULL;
    _frames.push_back(multics_stack_frame(_segno, sp_offset));
    partial.finished = 0;
    partial.bumper = seg_addr_t(PPR.PSR, PPR.IC);
    AR_PR_t ep;
    if (stack_to_entry(back().addr(), &ep) == 0)
        partial.entryp = seg_addr_t(ep.PR.snr, ep.wordno);
    // cancel_run(STOP_IBKPT);
    return 0;
}

//=============================================================================

// Check to see if entrypoint info has been added to what was last seen to be
// a partially constructed frame 

void multics_stack::check_frame()
{
    const char* moi = "STACK::check_frame";

    if (size() == 0)
        return;
    if (partial.finished)
        return;

    // Get runtime entry-point offset from stack frame
    multics_stack_frame &msf = back();
    int addr = msf.addr();
    if (addr < 0) {
        log_msg(INFO_MSG, "STACK::push", "Cannot convert frame %#o|%#o to absolute memory location.\n", _segno, msf.offset());
        return;
    }
    AR_PR_t entryp;
    if (stack_to_entry(addr, &entryp) != 0) {
        log_msg(INFO_MSG, "STACK::push", "Frame %#o|%#o seems bad -- no entry point.\n", _segno, msf.offset());
        return;
    }
    seg_addr_t entry_pr(entryp.PR.snr, entryp.wordno);

    int ic_diff = ((unsigned) partial.bumper.segno != PPR.PSR);
    if (! ic_diff) {
        int delta = (partial.bumper.offset > PPR.IC) ? PPR.IC - partial.bumper.offset
            : partial.bumper.offset - PPR.IC;
        ic_diff = delta > 20;
    }
    if (entry_pr != partial.entryp) {
        if (ic_diff)
            log_msg(INFO_MSG, moi, "Entryp changed and IC jump seen.\n");
        else
            log_msg(INFO_MSG, moi, "Entryp changed.\n");
    } else
        if (ic_diff)
            log_msg(INFO_MSG, moi, "IC jump seen, prior entryp must have still been good.\n");
        else {
            // nothing interesting has happened
            return;
        }

    // Find linkage, if available

    partial.finished = 1;

    // Lookup entry-point's offset in symbol table
    const seginfo& seg = segments(entry_pr.segno);
    map<int,linkage_info>::const_iterator li_it = seg.find_entry(entry_pr.offset);
    if (li_it != seg.linkage.end()) {
        const linkage_info* lip = &(*li_it).second;
        log_msg(INFO_MSG, moi, "Frame discovered to have entry ptr %s to %s.\n",
            string(entry_pr).c_str(), lip->name.c_str());
        msf.set_linkage(lip);
    } else
        log_msg(INFO_MSG, moi, "Frame discovered to have entry ptr %s to unknown procedure.\n",
            string(entry_pr).c_str());

    // cancel_run(STOP_IBKPT);
}

//=============================================================================

int multics_stack::is_in_frame(int segno, int offset, int debug)
    // Returns true if given IC is within top frame's entry
    // Given IC should be current or prior instruction
{
    if (size() == 0)
        return 0;

    if (! partial.finished)
        check_frame();

    const multics_stack_frame& msf = back();
    const entry_point* epp = msf.entry();
    if (epp == NULL)
        return 0;

    AR_PR_t entryp;
    if (stack_to_entry(msf.addr(), &entryp) != 0) {
        log_msg(INFO_MSG, "STACK::is_in_frame", "Frame at offset %#o seems bad -- no entry point.\n", msf.offset());
        return 0;
    }
if (debug) log_msg(INFO_MSG, NULL, "-- is-in-frame: Frame in seg %#o\n", entryp.PR.snr);
    if (entryp.PR.snr != segno)
        return 0;

    const linkage_info *lip;
    if (epp->last > 0) {
        const linkage_info* lip = msf.linkage();
if (debug) log_msg(INFO_MSG, NULL, "-- is-in-frame: linkage is %#o|%#o .. %#o|%#o\n", entryp.PR.snr, lip->offset,  entryp.PR.snr, lip->offset + epp->last);
        return lip->offset <= offset && offset <= lip->offset + epp->last;
    } else {
        // Lookup current IC in symbol table
        const seginfo& seg = segments(segno);
        map<int,linkage_info>::const_iterator li_it = seg.find_entry(offset);
if (debug) log_msg(INFO_MSG, NULL, "-- is-in-frame: find_entry(%d) returns %d\n", offset, li_it != seg.linkage.end());
        return (li_it != seg.linkage.end());
    }
}

//=============================================================================

multics_stack_frame::change_t multics_stack_frame::val_t::change(t_uint64 curr_val)
    // Update the current value (if changed) and report status
{
    // Note that the constructor sets _initialized to false, but also sets _val
    // to a copy of what's in memory.

    if (_val == curr_val)
        return (_initialized) ? unchanged : uninitialized;
    else {
        int prev_init = _initialized;
        _val = curr_val;
        _initialized = 1;
        return (prev_init) ? changed : initial_change;
    }
}

//=============================================================================

extern "C" void show_variables(unsigned segno, int ic);

// Called by the CPU after every instruction
    
void show_variables(unsigned segno, int ic)
{
    if (! opt_debug)
        state_invalidate_cache();
    else
        if (m_stackp && m_stackp->size() != 0)
            m_stackp->back().show(0);
    addr_modes_t amode = get_addr_mode();
    check_autos((amode == APPEND_mode) ? (int) segno : -1, ic);
}

//=============================================================================

static void check_autos(
    int segno,              // Execution segment or -1 if in appending mode
    int ic)                 // Recent IC
{
    if (m_stackp == NULL)
        return;

    if (! m_stackp->is_in_frame(segno, ic)) {
        // log_msg(INFO_MSG, "STACK::check_autos", "No stack frame found for IC %#o|%#o.\n", segno, ic);
        return;
    }

    m_stackp->back().update_autos(segno, ic);
}

//=============================================================================

static void dump_autos()
{
    if (m_stackp == NULL)
        return;

    addr_modes_t amode = get_addr_mode();
    int segno = (amode == APPEND_mode) ? PPR.PSR : -1;
    int ic = PPR.IC;

    if (! m_stackp->is_in_frame(segno, ic)) {
        out_msg("stack trace:\tNo stack frame found for IC %#o|%#o.\n", segno, ic);
        return;
    }

    m_stackp->back().show(0);
    out_msg("stack trace: Automatics for stack frame belonging to IC %#o|%#o:\n",
        segno, ic);
    m_stackp->back().dump_autos();
}

//=============================================================================

void multics_stack_frame::dump_autos() const
{
    int sp_addr = addr();
    if (sp_addr == -1) {
        log_msg(INFO_MSG, "STACK", "Internal error, line %d\n", __LINE__);
        return;
    }
    if (entry() == NULL)
        return;

    const entry_point& ep = *entry();
    const stack_frame* sfp = ep.stack();
    if (sfp == NULL) {
        log_msg(NOTIFY_MSG, "check_autos", "sanity check fails -- no stack\n");
        cancel_run(STOP_WARN);
        return;
    }

    int segno = AR_PR[6].PR.snr;
    out_msg("stack trace:\t%d Automatics in frame at %#o|%#o (%08o) for %s (%s):\n",
        size(), segno, offset(), sp_addr, ep.name.c_str(), ep.source->fname.c_str());
    // Loop through all the val_t automatic values for this frame
    for (map<int, val_t>::const_iterator autos_it = vals.begin(); autos_it != vals.end(); ++ autos_it) {
        int soffset = (*autos_it).first;
        const val_t& autov = (*autos_it).second;
        // We have the values for this frame, find the names in the entry_point
        map<int,var_info>::const_iterator ni = sfp->automatics.find(soffset);
        const var_info& v = (*ni).second;
        out_msg("stack trace:\t\tAuto at offset %04o, %s: %s: ",
            soffset,
            ((string) seg_addr_t(segno, _offset + soffset)).c_str(),
            v.name.c_str());
        ostringstream obuf;
        out_auto(obuf, v, sp_addr + soffset, autov.is_initialized());
        out_msg("%s\n", obuf.str().c_str());
    }
}

//=============================================================================

static void out_auto(ostringstream& obuf, const var_info& v, int addr, int is_initialized)
{
    ostringstream buf;
    if (v.type == var_info::ptr && v.size == 72) {
        t_uint64 curr1 = Mem[addr];
        t_uint64 curr2 = Mem[addr + 1];
        buf << setw(12) << setfill('0') << oct;
        buf << curr1 << " " <<  setw(12) << curr2;
        buf << setw(0);
        AR_PR_t pr;
        if (words2its(curr1, curr2, &pr) == 0) {
            buf << showbase
                << " [ring " << pr.PR.rnr << ", "
                << "address " << pr.PR.snr << "|" << pr.wordno << ", "
                << "bitno " << dec << pr.PR.bitno << "]";
            if (!is_initialized)
                buf << " (uninitialized)";
        } else {
            if (is_initialized) {
                buf << " <invalid ptr>";
                // temp
                log_msg(NOTIFY_MSG, "automatic vars", "Autobreakpoint on bad ptr val.\n");
                cancel_run(STOP_IBKPT);
            } else {
                buf << " <uninitialized invalid ptr>";
            }
        }
    } else {
        t_uint64 curr = Mem[addr];
        // buf << dec << curr << setw(12) << setfill('0') << oct << "(" << curr << ")";
        buf << dec << setw(0) << curr
            << " ("
            << oct << setw(12) << setfill('0') << curr << setw(0) << setfill(' ')
            << ")";
        if (!is_initialized)
            buf << " (uninitialized)";
    }
    obuf << buf.str();
}

//=============================================================================


// Check values in saved frame versus the machine

void multics_stack_frame::update_autos(
    int segno, int ic)      // Used for printing source line
{
    int sp_addr = addr();
    if (sp_addr == -1) {
        log_msg(INFO_MSG, "STACK", "Internal error, line %d\n", __LINE__);
        return;
    }

    if (entry() == NULL) {
        if (! m_stackp->unfinished())
            log_msg(INFO_MSG, "update_autos", "sanity check fails -- no entry point for IC %#o|%#o\n", segno, ic);
        return;
    }
    const entry_point& ep = *entry();
    const stack_frame* sfp = ep.stack();        // holds variable names
    if (sfp == NULL) {
        if (! m_stackp->unfinished()) {
            // BUG in listing.cpp -- PL/1 listings explicitly mention when procedures share
            // another's stack frame, but don't mention this for non-procedure entry points,
            // and listing.cpp doesn't record the stack_owner for us.
            log_msg(INFO_MSG, "check_autos", "sanity check fails -- no stack description for %s.\n",
                ep.name.c_str());
            // cancel_run(STOP_WARN);
        }
        return;
    }

    log_ignore_ic_change();
    int any_changed = 0;
    for (map<int, val_t>::iterator autos_it = vals.begin(); autos_it != vals.end(); ++ autos_it) {
        int soffset = (*autos_it).first;
        val_t& autov = (*autos_it).second;
        t_uint64 curr = Mem[sp_addr + soffset];
        change_t chg = autov.change(curr);
        if (chg == initial_change || chg == changed) {
#if 0
            // Disabled -- var could be passed by reference to another procedure, possibly
            // even one in another segment (and without a stack)
            if (!any_changed && entry() && entry()->source && log_any_io(0)) {
                log_msg(INFO_MSG, NULL, "Source: %s\n", entry()->source->fname.c_str());
                const source_line* lnp = entry()->source->get_line(ic);
                if (lnp)
                    log_msg(INFO_MSG, NULL, "Source:  %3o|%06o, line %5d(%d): > %s\n",
                        segno, ic, lnp->line_no, stack_depth(), lnp->text.c_str());
            }
            any_changed = 1;
#endif
#if 1
            int show_owner = 0;
            AR_PR_t entryp;
            if (stack_to_entry(sp_addr, &entryp) != 0) {
                log_msg(INFO_MSG, "STACK::update", "Frame %#o|%#o seems bad -- no entry point.\n", _segno, _offset);
                // return;
            } else {
                seg_addr_t entry_pr(entryp.PR.snr, entryp.wordno);
                if (entryp.PR.snr != segno) {
                    show_owner = 1;
                    log_msg(INFO_MSG, NULL, "-- Auto in seg %#o; running in seg %#o\n", entryp.PR.snr, segno);
                } else {
                    show_owner = ! m_stackp->is_in_frame(segno, ic);    // FIXME: hack knowing global
                    if (show_owner)
                        log_msg(INFO_MSG, NULL, "-- IC %#o|%#o is not within auto's frame\n", segno, ic);
                }
            }
#endif
            map<int,var_info>::const_iterator ni = sfp->automatics.find(soffset);
            const var_info& v = (*ni).second;
            ostringstream buf;
            // buf << "Source:                            " << v.name << " = ";
            //buf << "Source:                               " << v.name << " = ";
            buf << "Source:                               ";
#if 1
            if (show_owner && entry())
                buf << entry()->name << "::";
#endif
            buf << v.name << " = ";
            out_auto(buf, v, sp_addr + soffset, autov.is_initialized());
            if (chg == initial_change)
                buf << " -- initial value";
            buf << "\n";
            log_msg(INFO_MSG, NULL, buf.str().c_str());
        }
    }
    log_any_io(0);      // Output of variables doesn't count towards requiring re-display of source
    log_notice_ic_change();
}

//=============================================================================

int seginfo_show_all(int seg, int first)
{
    where_t where;
    if (seginfo_find_all(seg, first, &where) != 0) {
        out_msg("Cannot find anything.\n");
        return -1;
    }
    if (where.file_name)
        out_msg("File name: %s\n", where.file_name);
    else
        out_msg("File name unknown.\n");
    if (where.entry)
        out_msg("Entry point: %s\n", where.entry);
    else
        out_msg("Entry point unknown.\n");
    if (where.line_no >= 0)
        out_msg("Line number %d: %s\n", where.line_no, where.line ? where.line : "<no text>");
    return 0;
}

//=============================================================================

int cmd_stats(int32 arg, char *buf)
{
    float secs = (float) sys_stats.total_msec / 1000;
    out_msg("Stats: %.1f seconds: %lld cycles at %.0f cycles/sec, %lld instructions at %.0f instr/sec\n",
        secs, sys_stats.total_cycles, sys_stats.total_cycles/secs, sys_stats.total_instr, sys_stats.total_instr/secs);

#if FEAT_INSTR_STATS
    out_msg("Per-instruction statistics:\n");
    out_msg("   %-20s  %8s  %7s  %s\n", "Opcode", "Count", "Seconds", "Instr/Sec");
    uint tot_nexec = 0;
    uint tot_msec = 0;
    for (unsigned op = 0; op < ARRAY_SIZE(sys_stats.instr); ++op) {
        if (sys_stats.instr[op].nexec == 0)
            continue;
        tot_nexec += sys_stats.instr[op].nexec;
        tot_msec += sys_stats.instr[op].nmsec;
        char *opname = opcodes2text[op];
        out_msg("   %-20s  %8u  %6.1f  ", opname, sys_stats.instr[op].nexec, (float) sys_stats.instr[op].nmsec / 1000);
        if (sys_stats.instr[op].nmsec == 0)
            out_msg("%13s\n", "N/A");
        else
            out_msg("%9u/sec\n", sys_stats.instr[op].nexec / sys_stats.instr[op].nmsec * 1000);
    }
    out_msg("   %-20s  %8s  %7s  %s\n", "--------------------", "------", "-------", "-------------");
    out_msg("   %-20s  %8u  %6.1f  ", "TOTAL:", tot_nexec, (float) tot_msec / 1000);
    if (tot_msec == 0)
        out_msg("%13s\n", "N/A");
    else
        out_msg("%9u/sec\n", tot_nexec / tot_msec * 1000);
#endif

    return 0;
}

//=============================================================================
