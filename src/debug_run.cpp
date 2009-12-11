/*
    debug_run.cpp

    Support for runtime debugging -- tracking and displaying changes
    to registers, the current pl1 source line, automatic (stack based)
    variables, etc.
*/

using namespace std;
#include <iostream>
#include <iomanip>
#include <ostream>
#include <stdexcept>
#include <sstream>

//#include <unistd.h>
//#include <sys/types.h>
//#include <sys/stat.h>
//#include <fcntl.h>

#include "hw6180.h"
#include "seginfo.h"

// BUG: The following externs are hacks
extern t_uint64 M[];    /* memory */    // BUG: hack
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

static int stack_trace(void);
static void print_src_loc(const char *prefix, addr_modes_t addr_mode, int segno, int ic, const instr_t* instrp);
static void check_autos(int segno, int ic);
static void dump_autos(void);

//=============================================================================

static void init_seg_debug()
{
    memset(seg_debug, 0, sizeof(seg_debug));
    seg_debug_init_done = 1;
}


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

#define ic_hist_max 60
static int ic_hist_ptr;
static int ic_hist_wrapped;
static struct {
    addr_modes_t addr_mode;
    uint seg;
    uint ic;
    instr_t instr;
} ic_hist[ic_hist_max];

void ic_history_init()
{
    ic_hist_wrapped = 0;
    ic_hist_ptr = 0;
}

//=============================================================================

void ic_history_add()
{
    // Maintain a queue of recently executed instructions for later display via cmd_dump_history()
    // Caller should make sure IR and PPR are set to the recently executed instruction.

    ic_hist[ic_hist_ptr].addr_mode = get_addr_mode();
    ic_hist[ic_hist_ptr].seg = PPR.PSR;
    ic_hist[ic_hist_ptr].ic = PPR.IC;

    memcpy(&ic_hist[ic_hist_ptr].instr, &cu.IR, sizeof(ic_hist[ic_hist_ptr].instr));
    if (++ic_hist_ptr == ic_hist_max) {
        ic_hist_wrapped = 1;
        ic_hist_ptr = 0;
    }
}

//=============================================================================

int cmd_dump_history(int32 arg, char *buf)
    // Dumps the queue of instruction history
{
    // The queue is implemented via an array and is circular,
    // so we make two passes through the array.
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
            int segno = (ic_hist[i].addr_mode == APPEND_mode) ? ic_hist[i].seg: -1;
            print_src_loc("", ic_hist[i].addr_mode, segno, ic_hist[i].ic, &ic_hist[i].instr);
        }
    }
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
    int segno = (amode == APPEND_mode) ? PPR.PSR : -1;
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
        log_msg(INFO_MSG, NULL, "Source:  %3o|%06o, line %5d: < %s\n", segno, PPR.IC, where.line_no, where.line);
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
    return words2its(M[abs_addr+026], M[abs_addr+027], prp);
}

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


static void print_frame(int seg, int offset, int addr)
{
    // Print a single stack frame for stack_trace()

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
        out_msg("\tUnknowable entry {%llo,%llo}  ", M[addr+026], M[addr+027]);
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
    stack_trace();
    out_msg("stack trace:\n");
    dump_autos();
    out_msg("\n");

    float secs = (float) total_msec / 1000;
    out_msg("Stats: %.1f seconds: %lld cycles at %.0f cycles/sec, %lld instructions at %.0f instr/sec\n",
        secs, total_cycles, total_cycles/secs, total_instr, total_instr/secs);

    return 0;
}

//=============================================================================

static int stack_trace(void)
    // Trace through the Multics stack frames
    // See stack_header.incl.pl1 and http://www.multicians.org/exec-env.html
{
    // PR6 should point to the current stack frame.  That stack frame
    // should be within the stack segment.
    int seg = AR_PR[6].PR.snr;

    uint curr_frame;
    if (convert_address(&curr_frame, seg, AR_PR[6].wordno, 0) != 0) {
        out_msg("STACK: Cannot convert pr6 to absolute memory address.\n");
        return 1;
    }

    // The stack header will be at offset 0 within the stack segment.
    int offset = 0;
    uint hdr_addr;  // 24bit main memory address
    if (convert_address(&hdr_addr, seg, offset, 0) != 0) {
        out_msg("Stack Trace: Cannot convert %03o|0 to absolute memory address.\n", seg);
        return 1;
    }
    AR_PR_t stack_begin_pr;
    if (words2its(M[hdr_addr+022], M[hdr_addr+023], &stack_begin_pr) != 0) {
        out_msg("Stack Trace: Stack header seems invalid; no stack_begin_ptr at %03o|22\n", seg);
        return 1;
    }
    AR_PR_t stack_end_pr;
    if (words2its(M[hdr_addr+024], M[hdr_addr+025], &stack_end_pr) != 0) {
        out_msg("Stack Trace: Stack header seems invalid; no stack_end_ptr at %03o|24\n", seg);
        return 1;
    }
    if (stack_begin_pr.PR.snr != seg || stack_end_pr.PR.snr != seg) {
        out_msg("Stack Trace: Stack header seems invalid; stack frames are in another segment.\n");
        return 1;
    }
    AR_PR_t lot_pr;
    if (words2its(M[hdr_addr+026], M[hdr_addr+027], &lot_pr) != 0) {
        out_msg("Stack Trace: Stack header seems invalid; no LOT ptr at %03o|26\n", seg);
        return 1;
    }
    // TODO: sanity check LOT ptr

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
            out_msg("STACK Trace: Cannot convert address of frame %03o|%06o to absolute memory address.\n", seg, framep);
            return 1;
        }
        // Sanity check
        if (prev != 0) {
            AR_PR_t prev_pr;
            if (words2its(M[addr+020], M[addr+021], &prev_pr) == 0) {
                if (prev_pr.wordno != prev) {
                    out_msg("STACK Trace: Stack frame's prior ptr, %03o|%o is bad.\n", seg, prev_pr.wordno);
                }
            }
        }
        prev = framep;
        // Print the current frame
        if (finished && M[addr+022] == 0 && M[addr+024] == 0 && M[addr+026] == 0)
            break;
#if 0
        if (need_hist_msg) {
            need_hist_msg = 0;
            out_msg("stack trace: ");
            out_msg("Recently popped frames (aka where we recently returned from):\n");
        }
#endif
        print_frame(seg, framep, addr);
        // Get the next one
        AR_PR_t next;
        if (words2its(M[addr+022], M[addr+023], &next) != 0) {
            if (!finished)
                out_msg("STACK Trace: no next frame.\n");
            break;
        }
        if (next.PR.snr != seg) {
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
                out_msg("STACK Trace: DEBUG: next frame is outside the expected range for stack frames.\n");
        }

        // Use the return ptr in the current frame to print the source line.
        if (! finished) {
            AR_PR_t return_pr;
            if (words2its(M[addr+024], M[addr+025], &return_pr) == 0) {
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

    out_msg("stack trace: ");
    out_msg("Current Location:\n");
    out_msg("stack trace: ");
    print_src_loc("\t", get_addr_mode(), PPR.PSR, PPR.IC, &cu.IR);

    log_any_io(0);      // Output of source/location info doesn't count towards requiring re-display of source

    return 0;
}

//=============================================================================

class multics_stack_frame {
public:
    enum change_t { uninitialized, unchanged, initial_change, changed };
    class val_t {
        private:
            int _initialized;   // constructor sets _initialized to false, but
            t_uint64 _val;      // also sets _val to a copy of what's in memory
        public:
            val_t(t_uint64 v) { _val = v; _initialized = 0; }
            int is_initialized() const { return _initialized; }
            t_uint64 val() { if (!_initialized) throw logic_error("unitialized"); return _val; }
            change_t change(t_uint64 currval);
    };
private:
    seg_addr_t ptr;
    int _addr;                  // 24-bit absolute address corresponding to ptr
    const entry_point& _owner;
    int _all_initialized;
    map <int, val_t> vals;  // Key is stack offset
    int _refresh_all(int first);
public:
    map<int, val_t>::iterator begin() { return vals.begin(); }
    map<int, val_t>::iterator end() { return vals.end(); }
public:
    multics_stack_frame(int segno, int offset, entry_point& e);
    int addr();
    int addr() const;
    const int size() const { return vals.size(); }
    int operator == (const seg_addr_t& x) const  { return x == ptr; }
    const entry_point& owner() const { return _owner; }
};

static list<multics_stack_frame> multics_stack;
static multics_stack_frame* find_frame(int segno, int ic);
static void update_autos(int segno, int ic,  multics_stack_frame& msf);
static void dump_autos(multics_stack_frame& msf);

//=============================================================================

// option: instead of doing change detection, register breakpoint/callback

multics_stack_frame::multics_stack_frame(int segno, int offset, entry_point& e)
    : ptr(segno,offset), _owner(e)
{
    // WARNING -- we assume our stack never gets relocated to a different absolute address
    _addr = -1;
    _all_initialized = 0;

    const stack_frame* sfp = _owner.stack();
    if (sfp == NULL)
        throw logic_error("multics_stack_frame: entry has no stack info\n");
    if (addr() == -1)
        return;
    map<int,val_t>::iterator it = vals.end();
    for (map<int,string>::const_iterator autos_it = sfp->automatics.begin(); autos_it != sfp->automatics.end(); ++ autos_it) {
        int soffset = (*autos_it).first;
        t_uint64 curr = M[_addr+soffset];
        it = vals.insert(it, pair<int,val_t>(soffset, val_t(curr)));
    }
}

int multics_stack_frame::addr() const
{
    if (_addr != -1)
        return _addr;
    uint a;
    if (convert_address(&a, ptr.segno, ptr.offset, 0) == 0)
        return a;
    return -1;
}

int multics_stack_frame::addr()
{
    if (_addr == -1) {
        uint a;
        if (convert_address(&a, ptr.segno, ptr.offset, 0) == 0)
            _addr = a;
    }
    return _addr;
}


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

static int push_frame(int segno, int ic, const seginfo& seg)
{
    const char* moi = "STACK::push";
    // BUG/TODO: Caller should know the entry and pass it to us (and not call us if the segment
    // doesn't have stack info).

    if (seg.empty()) {
        cerr << "Impossible at line " << dec << __LINE__ << ": given segment is empty." << simh_endl;
        return -1;
    }

    // Find entry point corresponding to IC
    map<int,linkage_info>::const_iterator li_it = seg.find_entry(ic);
    if (li_it == seg.linkage.end())
        return -1;
    const linkage_info& li = (*li_it).second;
    if (li.entry == NULL) {
        // Nothing loaded via "xlist" command
        return -1;
    }

    // Find our stack description
    stack_frame *sfp = li.entry->stack();
    if (sfp == NULL) {
        // We didn't get a source listing that provided stack info
        // BUG: temp msg
            int stack_segno = AR_PR[6].PR.snr;  // TODO: add a constructor for AR_PR_t?
            int stack_offset = AR_PR[6].wordno;
        log_msg(INFO_MSG, moi, "Cannot push frame for SP %o|%o -- Address %03o|%06o is for linkage entry %s; No stack info.\n",
            stack_segno, stack_offset,
            segno, ic, li.name.c_str());
        // BUG: example: "bce_error$com_err" entry uses frame for "bce_error"
        return -1;
    }

    // Check to see that entry point listed in the frame matches our expectation

    int stack_segno = AR_PR[6].PR.snr;  // TODO: add a constructor for AR_PR_t?
    int stack_offset = AR_PR[6].wordno;

    if (stack_segno == 077777)
        return -1;          // Multics probably hasn't setup any stack frames yet

    AR_PR_t entry_pr;
    if (stack_to_entry(stack_segno, stack_offset, &entry_pr) != 0) {
        out_msg("Automatics: Stack frame at %03o|%06o does not contain a valid entry ptr.\n", stack_segno, stack_offset);
        return -1;
    }

    // create stack_frame
ostringstream s;
s << "Pushing new frame at depth ";
s << multics_stack.size() << " for " << li.name << " with PR6 == " << seg_addr_t(stack_segno,stack_offset);
    log_msg(INFO_MSG, moi, "%s\n", s.str().c_str());
    multics_stack.push_back(multics_stack_frame(stack_segno, stack_offset, *li.entry));

    return 0;
}

//=============================================================================

extern "C" void show_variables(unsigned segno, int ic);

void show_variables(unsigned segno, int ic)
{
    addr_modes_t amode = get_addr_mode();
    //int segno = (amode == APPEND_mode) ? PPR.PSR : -1;
    // check_autos(segno, ic);
    check_autos((amode == APPEND_mode) ? segno : -1, ic);
}

//=============================================================================

static void check_autos(int segno, int ic)
{
    multics_stack_frame* msfp = find_frame(segno, ic);
    if (msfp)
        update_autos(segno, ic, *msfp);
}

//=============================================================================

/*
 *
 * Find or create the stack frame for the entrypoint corresponding
 * to the given execution segment and offset.
 *
 */

static multics_stack_frame* find_frame(int segno, int ic)
{
    // segno and ic should be last executed instruction
    // TODO: improve efficiency -- Maybe only check when about to display source change.  Maybe use memory-write range breakpoints.
    const char *moi = "check_autos";

    const seginfo& seg = segments(segno);
    if (seg.empty()) {
        if (segno != -1)
            cerr << "check_autos: Odd, segment " << oct << segno << " is empty." << simh_endl;
        return NULL;
    }

    // TODO: check source_changed re checking for frame change

    seg_addr_t ptr(AR_PR[6].PR.snr, AR_PR[6].wordno);   // TODO: add a constructor for AR_PR_t?
    // Search backwards from the top of the stack
    list<multics_stack_frame>::reverse_iterator rstackp = find(multics_stack.rbegin(), multics_stack.rend(), ptr);
    if (rstackp == multics_stack.rend()) {
        push_frame(segno, ic, seg);
        return NULL;
    }
    list<multics_stack_frame>::iterator stackp = rstackp.base();
    -- stackp;          // Convert the reverse iterator into a forward iterator

    if (stackp != -- multics_stack.end()) {
        // Pop one or more frames
        if (0) {
            multics_stack_frame& curr = *stackp;
            multics_stack_frame& last = (*-- multics_stack.end());
            log_msg(INFO_MSG, "STACK", "Popping frames.  Curr is %s; popping through %s\n",
                curr.owner().name.c_str(), last.owner().name.c_str());
        }
        multics_stack.erase(++stackp, multics_stack.end());
        stackp = -- multics_stack.end();
    }
    return &(*stackp);
}

//=============================================================================

static void dump_autos()
{
    seg_addr_t ptr(AR_PR[6].PR.snr, AR_PR[6].wordno);   // TODO: add a constructor for AR_PR_t?
    // Search backwards from the top of the stack
    list<multics_stack_frame>::reverse_iterator rstackp = find(multics_stack.rbegin(), multics_stack.rend(), ptr);
    out_msg("stack trace: Automatics:\n");
    if (rstackp != multics_stack.rend())
        dump_autos(*rstackp);
    else if (multics_stack.size() == 0)
        out_msg("stack trace:\tNo stack frames have been recorded yet.\n");
    else {
        out_msg("stack trace:\tNo stack frame found for PR[6]==%#o|%#o; Dumping automatics for last known stack frame.\n", AR_PR[6].PR.snr, AR_PR[6].wordno);
        multics_stack_frame& msf = multics_stack.back();
        out_msg("stack trace:\t%s\n", msf.owner().name.c_str());
        dump_autos(msf);
    }
}

static void dump_autos(multics_stack_frame& msf)
{
    int sp_addr = msf.addr();
    if (sp_addr == -1)
        log_msg(INFO_MSG, "STACK", "Internal error, line %d\n", __LINE__);
    else {
        const entry_point& ep = msf.owner();
        const stack_frame* sfp = ep.stack();
        if (sfp == NULL) {
            log_msg(NOTIFY_MSG, "check_autos", "sanity check fails -- no stack\n");
            cancel_run(STOP_WARN);
            return;
        }
        out_msg("stack trace:\t%d Automatics:\n", msf.size());
        for (map<int, multics_stack_frame::val_t>::const_iterator autos_it = msf.begin(); autos_it != msf.end(); ++ autos_it) {
            int soffset = (*autos_it).first;
            const multics_stack_frame::val_t& autov = (*autos_it).second;
            t_uint64 curr = M[sp_addr + soffset];
            map<int,string>::const_iterator ni = sfp->automatics.find(soffset);
            const string& name = (*ni).second;
            out_msg("stack trace:\t\tAuto %s: %lld (%012llo)%s\n",
                name.c_str(), curr, curr, 
                (autov.is_initialized()) ? "" : " (uninitialized)");
        }
    }
}

//=============================================================================


// Check values in saved frame versus the machine

static void update_autos(int segno, int ic, multics_stack_frame& msf)
{
    int sp_addr = msf.addr();
    if (sp_addr == -1)
        log_msg(INFO_MSG, "STACK", "Internal error, line %d\n", __LINE__);
    else {
        const entry_point& ep = msf.owner();
        const stack_frame* sfp = ep.stack();
        if (sfp == NULL) {
            log_msg(NOTIFY_MSG, "check_autos", "sanity check fails -- no stack for seg %o...\n", segno);
            cancel_run(STOP_WARN);
            return;
        }
        int changed = 0;
        for (map<int, multics_stack_frame::val_t>::iterator autos_it = msf.begin(); autos_it != msf.end(); ++ autos_it) {
            int soffset = (*autos_it).first;
            multics_stack_frame::val_t& autov = (*autos_it).second;
            t_uint64 curr = M[sp_addr + soffset];
            enum multics_stack_frame::change_t chg = autov.change(curr);
            if (chg == multics_stack_frame::initial_change || chg == multics_stack_frame::changed) {
                if (!changed && msf.owner().source && log_any_io(0)) {
                    log_msg(INFO_MSG, NULL, "Source: %s\n", msf.owner().source->fname.c_str());
                    const source_line* lnp = msf.owner().source->get_line(ic);
                    if (lnp)
                        log_msg(INFO_MSG, NULL, "Source:  %3o|%06o, line %5d: > %s\n", segno, ic, lnp->line_no, lnp->text.c_str());
                }
                changed = 1;
                map<int,string>::const_iterator ni = sfp->automatics.find(soffset);
                const string& name = (*ni).second;
                log_msg(INFO_MSG, NULL, "Source:                            %s = %lld (%012llo)%s\n", name.c_str(), curr, curr, (chg == multics_stack_frame::initial_change) ? " -- initial value" : "");
            }
        }
        log_any_io(0);      // Output of variables doesn't count towards requiring re-display of source
    }
}

//=============================================================================

