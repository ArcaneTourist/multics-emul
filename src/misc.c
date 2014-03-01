/*
    misc.c -- small routines that didn't have any other home.

    Currently, only some debug messaging functions.
        
*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#include <stdio.h>
#include <stdarg.h>
#include "hw6180.h"
#include "seginfo.hpp"

extern DEVICE cpu_dev;
extern FILE *sim_deb, *sim_log;

static void msg(enum log_level level, const char *who, const char* format, va_list ap);
uint ignore_IC = 0;
uint last_IC;
uint last_IC_seg;

static int _log_any_io = 0;

int log_any_io(int val)
    // Callers may use to determine if other functions have call log_xxx to do output
{
    int tmp = _log_any_io;
    _log_any_io = val;
    return tmp;
}

int log_ignore_ic_change()
{
    int old = ignore_IC;
    ignore_IC = 1;
    return old;
}

int log_notice_ic_change()
{
    int old = ignore_IC;
    ignore_IC = 0;
    return old;
}

void log_forget_ic()
{
    last_IC = 0;
    last_IC_seg = 0;
}

void log_msg(enum log_level level, const char* who, const char* format, ...)
{
    if (level == DEBUG_MSG) {
        if (opt_debug == 0)
            return;
        if (cpu_dev.dctrl == 0 && opt_debug < 1)        // todo: should CPU control all debug settings?
            return;
    }

    // Make sure all messages have a prior display of the IC
    if (!ignore_IC && (PPR.IC != last_IC || PPR.PSR != last_IC_seg)) {
        last_IC = PPR.IC;
        last_IC_seg = PPR.PSR;
        char *tag = "Debug";
        // char *who = "IC";
            // out_msg("\n%s: %*s %s  %*sIC: %o\n", tag, 7-strlen(tag), "", who, 18-strlen(who), "", PPR.IC);
        char icbuf[80];
        addr_modes_t addr_mode = get_addr_mode();
        ic2text(icbuf, addr_mode, PPR.PSR, PPR.IC);
        // out_msg("\n");
        // out_msg("%s: %*s IC: %s\n", tag, 7-strlen(tag), "", icbuf);
        msg(DEBUG_MSG, NULL, "\n", NULL);
        char buf[80];
        sprintf(buf, "%s: %*s IC: %s\n", tag, 7 - (int) strlen(tag), "", icbuf);
        msg(DEBUG_MSG, NULL, buf, NULL);
    }

    va_list ap;
    va_start(ap, format);
#if 0
    char *tag = (level == DEBUG_MSG) ? "Debug" :
        (level == INFO_MSG) ? "Info" :
        (level == NOTIFY_MSG) ? "Note" :
        (level == WARN_MSG) ? "WARNING" :
        (level == ERR_MSG) ? "ERROR" :
            "???MESSAGE";
    msg(tag, who, format, ap);
#else
    msg(level, who, format, ap);
#endif
    va_end(ap);
}

#if 0
void debug_msg(const char* who, const char* format, ...)
{
    if (opt_debug == 0)
        return;
    if (cpu_dev.dctrl == 0 && opt_debug < 1)        // todo: should CPU control all debug settings?
        return;
    va_list ap;
    va_start(ap, format);
    msg("Debug", who, format, ap);
    va_end(ap);
}

void warn_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("WARNING", who, format, ap);
    va_end(ap);
}

void complain_msg(const char* who, const char* format, ...)
{
    va_list ap;
    va_start(ap, format);
    msg("ERROR", who, format, ap);
    va_end(ap);
}
#endif


static void crnl_out(FILE *stream, const char *format, va_list ap)
{
    // SIMH does something odd with the terminal, so output CRNL
    int len =strlen(format);
    int nl = *(format + len - 1) == '\n';
    if (nl) {
        char *f = malloc(len + 2);
        if (f) {
            strcpy(f, format);
            *(f + len - 1) = '\r';
            *(f + len) = '\n';
            *(f + len + 1) = 0;
            if (ap == NULL)
                fprintf(stream, "%s", f);
            else
                vfprintf(stream, f, ap);
            free(f);
        } else {
            if (ap == NULL)
                printf("%s", format);
            else
                vprintf(format, ap);
            if (*(format + strlen(format) - 1) == '\n')
                fprintf(stream, "\r");
        }
    } else {
        if (ap == NULL)
            fprintf(stream, "%s", format);
        else
            vfprintf(stream, format, ap);
        if (*(format + strlen(format) - 1) == '\n')
            fprintf(stream, "\r");
    }
    _log_any_io = 1;
}


void out_msg(const char* format, ...)
{
    va_list ap;
    va_start(ap, format);

    FILE *stream = (sim_log != NULL) ? sim_log : stdout;
    crnl_out(stream, format, ap);
    va_end(ap);
    if (sim_deb != NULL) {
        va_start(ap, format);
        crnl_out(sim_deb, format, ap);
        va_end(ap);
    }
}

static void out_sym_stm(FILE *stream, int is_write, t_addr simh_addr, t_value *val, UNIT *uptr, int32 sw)
{
    if (stream == NULL)
        return;
    fprintf(stream, "%s Memory ",
        (is_write) ? "Write" : "Read");
    fprint_addr(stream, NULL, simh_addr);
    fprintf(stream, ": ");
    fprint_sym(stream, simh_addr, val, uptr, sw);
    crnl_out(stream, "\n", NULL);
}


void out_sym(int is_write, t_addr simh_addr, t_value *val, UNIT *uptr, int32 sw)
{
    FILE *stream = (sim_log != NULL) ? sim_log : stdout;
    out_sym_stm(stream, is_write, simh_addr, val, uptr, sw);
    if (sim_deb != NULL) {
        fprintf(sim_deb, "Debug: ");
        out_sym_stm(sim_deb, is_write, simh_addr, val, uptr, sw);
    }
}


#if 0
static void sim_hmsg(const char* tag, const char *who, const char* format, va_list ap)
{
    // This version uses SIMH facilities -- not tested
    char buf[2000];
    
    snprintf(buf, sizeof(buf), "%s: %*s %s: %*s", tag, 7-strlen(tag), "", who, 18-strlen(who), "");
    int l = strlen(buf);
    vsnprintf(buf + l, sizeof(buf) - l, format, ap);
    // TODO: setup every device with sim_debtab entries to reflect different debug levels
    sim_debug(~0, &cpu_dev, "%s", buf);
}
#endif

static void msg(enum log_level level, const char *who, const char* format, va_list ap)
{
    // This version does not use SIMH facilities -- except for the sim_deb and sim_log streams

    enum { con, dbg };
    FILE *streams[2];

    streams[con] = (sim_log != NULL) ? sim_log : stdout;
    streams[dbg] = sim_deb;
    if (level == DEBUG_MSG || level == INFO_MSG) {
        // Debug and info messages go to a debug log if one exists, otherwise to
        //  the console
        if (streams[dbg] != NULL)
            streams[con] = NULL;
    } else {
        // Non debug msgs always go to the console.  If a seperate debug
        // log exists, it also gets non-debug msgs.
        streams[dbg] = (sim_log == sim_deb) ? NULL : sim_deb;
    }

    char *tag = (level == DEBUG_MSG) ? "Debug" :
        (level == INFO_MSG) ? "Info" :
        (level == NOTIFY_MSG) ? "Note" :
        (level == WARN_MSG) ? "WARNING" :
        (level == WARN_MSG) ? "WARNING" :
        (level == ERR_MSG) ? "ERROR" :
            "???MESSAGE";

    for (int s = 0; s <= dbg; ++s) {
        FILE *stream = streams[s];
        if (stream == NULL)
            continue;
        if (who != NULL)
            fprintf(stream, "%s: %*s %s: %*s", tag, 7 - (int) strlen(tag), "", who, 18 - (int) strlen(who), "");
    
        if (ap == NULL)
            crnl_out(stream, format, NULL);
        else {
            va_list aq;
            va_copy(aq, ap);
            crnl_out(stream, format, aq);
            va_end(aq);
        }
        if (level != DEBUG_MSG) // BUG: ?
            fflush(stream);
    }
}

// ============================================================================


int cmd_find(int32 arg, char *buf)
{
    if (*buf == 0) {
        out_msg("USAGE xfind \"string\" <memory range>\n");
        return 1;
    }
    if (strlen(buf) > 512)
        return 1;
    char *bufp = buf;
    bufp += strspn(bufp, "  ");
    if (*bufp != '"') {
        out_msg("USAGE: xfind \"string\" <memory range>.  A string was expected.\n");
        return 1;
    }
    char search[513];
    char *searchp = search;
    char *tailp = bufp + 1;
    for (;;) {
        char *currp = tailp;
        tailp = strchr(tailp, '"');
        if (! tailp)
            break;
        if (tailp[-1] == '\\') {
            strncpy(searchp, currp, tailp - currp);
            searchp += tailp - currp;
            searchp[-1] = '"';
            ++ tailp;
        }
        else {
            strncpy(searchp, currp, tailp - currp);
            searchp += tailp - currp;
            *searchp = 0;
            break;
        }
    }
    if (tailp == NULL) {
        out_msg("Missing close quote(\").\n");
        return 1;
    }
    if (tailp == bufp + 1) {
        out_msg("Nothing to search for.\n");
        return 1;
    }

    ++ tailp;
    tailp += strspn(tailp, "    ");
    if (*tailp == 0) {
        out_msg("Specify memory address(es) to search within.\n");
        return 1;
    }

    //t_addr lo = 0, hi = 0;
    t_addr lo;
    int n = -1;

    // Get first address
    char *lo_text = tailp;
    lo = (*sim_vm_parse_addr)(NULL, tailp, &tailp);
    if (! tailp) {
        out_msg("Cannot parse first address in address range.\n");
        return 1;
    }
    addr_modes_t lo_mode;
    unsigned lo_segno;
    unsigned lo_offset;
    if (addr_simh_to_emul(lo, &lo_mode, &lo_segno, &lo_offset) != 0)
        return 1;
    
    addr_modes_t hi_mode;
    unsigned hi_segno;
    unsigned hi_offset;

    // Expect either: -<addr> or /count
    char *lo_tail = tailp;
    tailp += strspn(tailp, "    ");
    if (*tailp == 0) {
        //  xfind <string> <addr>
        //hi_mode = lo_mode;
        //hi_segno = lo_segno;
        //hi_offset = lo_offset;
        n = 1;
    } else {
        if (*tailp == '/') {
            //  xfind <string> <addr> / <count>
            ++ tailp;
            if (strspn(tailp, "01234567     ") == strlen(tailp))
                (void) sscanf(tailp, "%o", (unsigned *) &n);
            else {
                out_msg("Expecting a count after '/'.\n");
                return 1;
            }
        } else if (*tailp == '-') {
            //  xfind <string> <addr> - <addr>
            ++ tailp;
            tailp += strspn(tailp, "    ");
            n = -1;
            t_addr hi = (*sim_vm_parse_addr)(NULL, tailp, &tailp);
            if (! tailp) {
                out_msg("Cannot parse second address.\n");
                return 1;
            }
            if (addr_simh_to_emul(hi, &hi_mode, &hi_segno, &hi_offset) != 0)
                return 1;
            if (hi_mode != lo_mode || hi_segno != lo_segno) {
                out_msg("ERROR: Cannot mix segmentation information.\n");
                return 1;
            }
            tailp += strspn(tailp, "    ");
            if (*tailp != 0) {
                out_msg("WARNING: Extra text after second address.\n");
            }
        } else {
            out_msg("Cannot parse address range.\n");
            return 1;
        }
    }
    * lo_tail = 0;

    int n_words = (n == - 1) ? hi_offset - lo_offset : n;
    if (lo_mode == APPEND_mode)
        if (n == -1)
            out_msg("Searching for '%s' in %#o|%#o .. |%#o (%d words)\n",
                search, lo_segno, lo_offset, hi_offset, n_words);
        else
            out_msg("Searching for '%s' in %#o|%#o/%d\n", search, lo_segno, lo_offset, n);
    else
        if (n == -1)
            out_msg("Searching for '%s' in %#o ..%#o (%d words)\n",
                search, lo_offset, hi_offset, n_words);
        else
            out_msg("Searching for '%s' in %#o/%d\n", search, lo_offset, n);

    char *matchp = search;
    unsigned char stupid[2][513];
    memset(stupid, 0, sizeof(stupid));
    int nstupid = 0;
    int which = 0;
    int found = 0;
    addr_modes_t saved_amode = get_addr_mode();
    uint saved_seg = TPR.TSR;
    int seg = lo_segno;
    TPR.TSR = lo_segno;
    set_addr_mode(lo_mode);
    int addr = lo_offset;
    int warn_uninit = sys_opts.warn_uninit;
    sys_opts.warn_uninit = 0;
    for (int addr = lo_offset; addr <= lo_offset + n_words; ++ addr) {
        t_uint64 word;
        if (fetch_word(addr, &word) != 0) {
            TPR.TSR = saved_seg;
            set_addr_mode(saved_amode);
            sys_opts.warn_uninit = warn_uninit;
            return 1;
        }
        // Really horrible -- no worse algorithm is possible
        // But it was quick to code and doesn't matter for interactive use
        for (int b = 0; b < 4; ++b) {
            uint byte = (word >> 27) & MASKBITS(8);
            word <<= 9;
            if (byte == 0) {
                nstupid = 0;
                continue;
            }
            if (nstupid >= sizeof(*stupid) - 1) {
                memcpy(stupid[!which], stupid[which]+1, sizeof(*stupid));
                which = ! which;
                stupid[which][sizeof(*stupid)-1] = byte;
                // out_msg("DEBUG: stupid[%d] is now %s.\n", which, bytes2text(stupid[which], nstupid));
            } else {
                stupid[which][nstupid++] = byte;
                // out_msg("DEBUG: stupid[%d] is %s.\n", which, bytes2text(stupid[which], nstupid));
            }
            char* p;
            if ((p = strstr((char *) stupid[which], search)) != 0) {
                int s = strlen(search);
                int a = addr * 4 + b - s + 1;
                int b = a % 4;
                a = a / 4;
                if (seg >= 0)
                    out_msg("Found it address %o|%#o, byte %o\n", seg, a, b);
                else
                    out_msg("Found it address %#o(%d), byte %o\n", a, a, b);
                // found = 1;
                //break;
#if 0
                int o = (unsigned char *) p - stupid[which];
                out_msg("DEBUG: copying offset %d, len %d to other array\n", o + 1, sizeof(*stupid) - o - 1);
                memset(stupid[!which], 0, sizeof(*stupid));
                memcpy(stupid[!which], p + 1, sizeof(*stupid) - o - 1);
                which = ! which;
                nstupid -= o;
#else
                memset(stupid[!which], 0, sizeof(*stupid));
                memcpy(stupid[!which], p + 1, s - 1);
                nstupid = s - 1;
                which = ! which;
#endif
                // out_msg("DEBUG: nstupid now %d\n", nstupid);
                // out_msg("DEBUG: stupid[%d] is now %s.\n", which, bytes2text(stupid[which], nstupid));
            }
        }
    }

    TPR.TSR = saved_seg;
    set_addr_mode(saved_amode);
    sys_opts.warn_uninit = warn_uninit;
    return 0;
}

// ============================================================================

int cmd_symtab_parse(int32 arg, char *buf)
{
    if (*buf == 0)
        seginfo_dump();
    else if (strcmp(buf, "dump") == 0)
        seginfo_dump();
    else {
        char *p = buf;
        char fname[1024];   // BUG: WARNING: buffer overflow possible
        int first, last;
        char dummy;
        int seg;
        if (sscanf(buf, "source %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            seginfo_add_source_file(-1, first, last, fname);
        } else if (sscanf(buf, "source %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            seginfo_add_source_file(seg, first, last, fname);
#if 0
        } else if (sscanf(buf, "entry %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            symtab_add_entry(-1, first, last, fname);
        } else if (sscanf(buf, "entry %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            symtab_add_entry(seg, first, last, fname);
#endif
        } else if (strncmp(buf, "dump", 4) == 0 && sscanf(buf, "dump %c", &dummy) == 0) {
            seginfo_dump();
        } else if (sscanf(buf, "where %i %c", &first, &dummy) == 1) {
            seginfo_show_all(-1, first);
        } else if (sscanf(buf, "where %i|%i %c", &seg, &first, &dummy) == 2) {
            seginfo_show_all(seg,first);
        } else if (sscanf(buf, "find %i %c", &first, &dummy) == 1) {
            seginfo_show_all(-1, first);
        } else if (sscanf(buf, "find %i|%i %c", &seg, &first, &dummy) == 2) {
            seginfo_show_all(seg,first);
        } else
            fprintf(stderr, "xsymtab: cannot parse '%s'\n", buf);
    }
    return 0;
}

// ============================================================================

int apu_show_seg(FILE *st, UNIT *uptr, int val, void *desc)
{
    // FIXME: use FILE *st

    const char* bufp = desc;
    if (bufp == NULL) {
        out_msg("Error, segment number required\n");
        return SCPE_ARG;
    }
    unsigned segno;
    char c;
    int n;
    if (sscanf(bufp, "%o %c", &segno, &c) != 1) {
        out_msg("Error, expecting an octal segment number.\n");
        return SCPE_ARG;
    }
    return scan_seg(segno, 1);
}

// ============================================================================

void flush_logs()
{
    if (sim_log != NULL)
        fflush(sim_log);
    if (sim_deb != NULL)
        fflush(sim_deb);
}

// ============================================================================

/*
 * bin2text()
 *
 * Display as bit string.
 *
 */

#include <ctype.h>

char *bin2text(t_uint64 word, int n)
{
    // WARNING: static buffer
    static char str1[65];
    static char str2[65];
    static char *str = NULL;
    if (str == NULL)
        str = str1;
    else if (str == str1)
        str = str2;
    else
        str = str1;
    str[n] = 0;
    int i;
    for (i = 0; i < n; ++ i) {
        str[n-i-1] = ((word % 2) == 1) ? '1' : '0';
        word >>= 1;
    }
    return str;
}

// ============================================================================

#if 0
static char *bytes2text(const unsigned char *s, int n)
{
    static char buf[2015];
    sprintf(buf, "<%d> ", n);
    char *bufp = buf + strlen(buf);
    while(n-- > 0) {
        if (isprint(*s))
            *bufp++ = *s++;
        else {
            sprintf(bufp, "\\%03o", *s++);
            bufp += 4;
        }
    }
    *bufp = 0;
    return buf;
}
#endif

// ============================================================================


void word2pr(t_uint64 word, AR_PR_t *prp)
{
    // same as lprpN -- BUG

    prp->PR.bitno = getbits36(word, 0, 6);
    prp->AR.charno = prp->PR.bitno / 9;
    prp->AR.bitno = prp->PR.bitno % 9;
    if (getbits36(word, 6, 12) == 07777)
        prp->PR.snr = 070000;   // bits 0..2 of 15-bit register
    else
        prp->PR.snr = 0;
    prp->PR.snr |= getbits36(word, 6, 12);
    prp->wordno = getbits36(word, 18, 18);
}

// ============================================================================

int words2its(t_uint64 word1, t_uint64 word2, AR_PR_t *prp)
{
    if ((word1 & MASKBITS(6)) != 043) {
        return 1;
    }
    prp->PR.snr = getbits36(word1, 3, 15);
    prp->wordno = getbits36(word2, 0, 18);
    prp->PR.rnr = getbits36(word2, 18, 3);  // not strictly correct; normally merged with other ring regs
    prp->PR.bitno = getbits36(word2, 57 - 36, 6);
    prp->AR.charno = prp->PR.bitno / 9;
    prp->AR.bitno = prp->PR.bitno % 9;
    return 0;
}

// ============================================================================

#if 0
int get_addr(uint segno, uint offset, uint *addrp)
{
    uint saved_seg = TPR.TSR;
    TPR.TSR = seg;
    fault_gen_no_fault = 1;
    if (get_seg_addr(offset, 0, addrp) != 0) {
        if (saved_seg != -1)
            TPR.TSR = saved_seg;
        fault_gen_no_fault = 0;
        *optr = cptr;
        return 0;
    }
    fault_gen_no_fault = 0;
    TPR.TSR = saved_seg;
    return 1;
}
#endif

