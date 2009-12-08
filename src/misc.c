/*
    misc.c -- small routines that didn't have any other home.

    Currently, only some debug messaging functions.
        
*/

#include "hw6180.h"
#include <stdio.h>
#include <stdarg.h>

extern DEVICE cpu_dev;
extern FILE *sim_deb, *sim_log;

static void msg(enum log_level level, const char *who, const char* format, va_list ap);
static int _scan_seg(uint segno, int msgs);
static int seginfo_show_all(int seg, int first);
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
        sprintf(buf, "%s: %*s IC: %s\n", tag, 7-strlen(tag), "", icbuf);
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
            vfprintf(stream, f, ap);
            free(f);
        } else {
            vprintf(format, ap);
            if (*(format + strlen(format) - 1) == '\n')
                fprintf(stream, "\r");
        }
    } else {
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
    if (sim_deb != NULL)
        crnl_out(sim_deb, format, ap);
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
            fprintf(stream, "%s: %*s %s: %*s", tag, 7-strlen(tag), "", who, 18-strlen(who), "");
    
        crnl_out(stream, format, ap);
        if (level != DEBUG_MSG) // BUG: ?
            fflush(stream);
    }
}

// ============================================================================

static void word2pr(t_uint64 word, AR_PR_t *prp);
int fetch_acc(uint addr, char bufp[513]);

t_stat cmd_seginfo(int32 arg, char *buf)
{
    // See descripton at _scan_seg().

    if (*buf == 0) {
        out_msg("USAGE xseginfo <segment number>\n");
        return 1;
    }
    unsigned segno;
    char c;
    int n;
    if (sscanf(buf, "%o %c", &segno, &c) != 1) {
        out_msg("xseginfo: Expecting a octal segment number.\n");
        return 1;
    }
    return scan_seg(segno, 1);
}


// ============================================================================

int scan_seg(uint segno, int msgs)
{
    // See descripton at _scan_seg().

    uint saved_seg = TPR.TSR;
    fault_gen_no_fault = 1;
    addr_modes_t amode = get_addr_mode();
    set_addr_mode(APPEND_mode);

    int saved_debug = opt_debug;
    opt_debug = 0;
    t_stat ret = _scan_seg(segno, msgs);
    if (ret > 1 && msgs)
        out_msg("xseginfo: Error processing request.\n");

    opt_debug = saved_debug;
    TPR.TSR = saved_seg;
    fault_gen_no_fault = 0;
    set_addr_mode(amode);
    return ret;
}


static int _scan_seg(uint segno, int msgs)
{

    // Dump the linkage info for an in-memory segment.
    // Can be invoked by an interactive command.
    // Also invoked by the CPU to discover entrypoint names and locations so
    // that the CPU can display location information.
    // Any entrypoints found are passed to seginfo_add_linkage().

    t_uint64 word0, word1;

    if (opt_debug) log_msg(DEBUG_MSG, "scan-seg", "Starting for seg %#o\n", segno);
    if (msgs) {
        /* Get last word of segment -- but in-memory copy may be larger than original, so search for last non-zero word */
        TPR.TSR = segno;
        if (fetch_word(0, &word0) != 0) {
            if (msgs)
                out_msg("xseginfo: Cannot read first word of segment %#o.\n", segno);
        }
        SDW_t *sdwp = get_sdw();        // Get SDW for TPR.TSR
        if (sdwp == 0) {
            out_msg("xseginfo: Cannot find SDW segment descriptor word for segment %#o.\n", segno);
        } else {
            out_msg("SDW segment descriptor word for %#o: %s\n", segno, sdw2text(sdwp));
            int bound = 16 * (sdwp->bound + 1);
            t_uint64 last = 0;
            for (bound -= 2; bound > 0; bound -= 2) {
                if (fetch_pair(bound, &word0, &word1) != 0) {
                    out_msg("xseginfo: Error reading %#o|%#o/2\n", segno, bound);
                    break;
                }
                // don't stop for 'b' 'k' 'p' 't'
                if (word0 != 0 && word0 != 0142153160164) {
                    last = word0;
                    break;
                }
                if (word1 != 0 && word1 != 0142153160164) {
                    last = word1;
                    ++ bound;
                    break;
                }
            }
            if (bound > 0) {
                out_msg("Last non-zero word might be at %#o|%#o: %012llo\n", segno, bound, last);
            }
        }
    }

    /* Read LOT */

    TPR.TSR = 015;

    if (fetch_word(segno, &word0) != 0) {
        if (msgs)
            out_msg("xseginfo: Error reading LOT entry 15|%o\n", segno);
        return 1;
    }
    if (word0 == 0) {
        if (msgs)
            out_msg("LOT entry for seg %#o is empty.\n", segno);
        return 0;
    }
    AR_PR_t linkage;
    word2pr(word0, &linkage);
    if (msgs)
        out_msg("LOT entry for seg %#o is %o|%#o linkage pointer.\n", segno, linkage.PR.snr, linkage.wordno);

    /* Linkage section -- read definitions pointer */
    //uint linkage_addr;
    //if (get_addr(linkage.PR.snr, linkage.wordno, &linkage_addr) != 0)
    //  return 2;
    TPR.TSR = linkage.PR.snr;
    AR_PR_t defs = { 0, { 0, 0, 0 }, { 0, 0} };
    if (fetch_pair(linkage.wordno, &word0, &word1) != 0)
        return 2;
    if (word0 == 0 && word1 == 0) {
        if (msgs)
            out_msg("Seg %#o has a linkage section at %o|%o with no ITS pointer to definitions.\n", segno, linkage.PR.snr, linkage.wordno);
        return 1;
    }
    words2its(word0, word1, &defs);
    if (msgs)
        out_msg("Linkage section has ITS to defs at %o|%o\n", defs.PR.snr, defs.wordno);

    /* Linkage section -- read link pair info */
    if (fetch_word(linkage.wordno + 6, &word1) != 0)
        return 2;
    uint first_link = word1 >> 18;
    uint last_link = word1 & MASK18;
    if (msgs) {
        if (word1 == 0)
            out_msg("Linkage section has no references to other segments.\n");
        else {
            out_msg("Linkage section has %d references to other segments at entry #s %#o .. %#o.\n", (last_link - first_link) / 2 + 1, first_link, last_link);
        }
    }

    TPR.TSR = defs.PR.snr;

    uint defp = defs.wordno;
    if (fetch_word(defp, &word0) != 0)
        return 2;
    uint off = word0 >> 18;
    if (off == 0) {
        if (msgs)
            out_msg("Definition header points doesn't point to any entries\n");
    } else {
        if (msgs) {
            out_msg("\n");
            out_msg("Definition header points to first entry at offset %#o.   This segment provides the following:\n", off);
        }
        defp += off;
        char entryname[1025];
        char *entryp = entryname;
        uint segment_defs = 0;  // Note that no definitions will appear at offset zero
        for (;;) {
            t_uint64 word2, word3;
            if (fetch_word(defp, &word0) != 0)
                return 2;
            if (word0 == 0)
                break;  
            if (fetch_word(defp + 1, &word1) != 0)
                return 2;
            if (fetch_word(defp + 2, &word2) != 0)
                return 2;
            //if (fetch_word(defp + 3, &word3) != 0)
            //  return 2;
            uint fwdp = word0 >> 18;
            uint class = word1 & 7;
            // out_msg("Def entry at %o|%o (offset %#o): fwdp %#o, class %#o.  ", TPR.TSR, defp, defp - defs.wordno, fwdp, class);
            if (msgs)
                out_msg("Def entry at %o|%04o (offset %04o): class %#o.  ", TPR.TSR, defp, defp - defs.wordno, class);
            uint namep = word2 >> 18;
            char buf[513];
            if (fetch_acc(defs.wordno + namep, buf) != 0)
                return 2;
            if (class == 3) {
                uint firstp = word2 & MASK18;
                // Segments can have multiple names
                // seginfo_add_name(segno, thing_relp, buf);    // call unneeded, only first name matters
                if (firstp == segment_defs) {
                    if (msgs) {
                        *(entryp - 1) = 0;
                        out_msg("Segment %s has alias %s\n", entryname, buf);
                        *(entryp - 1) = '$';
                    }
                } else {
                    if (msgs)
                        out_msg("Segment Name is %s; first def is at offset %#o.\n", buf, firstp);
                    segment_defs = firstp;
                    strcpy(entryname, buf);
                    entryp = entryname + strlen(entryname);
                    *entryp++ = '$';
                }
            } else {
                char sectbuf[20];
                const char *sect;
                if (class == 0)
                    sect = "text";
                else if (class == 2)
                    sect = "symbol";
                else {
                    sect = sectbuf;
                    sprintf(sectbuf, "class %d", class);
                }
                uint thing_relp = word1 >> 18;
                //t_uint64 tmp_word;
                //if (fetch_word(defs.wordno + thing_relp, &tmp_word) != 0) {
                //  out_msg("%s name is %s; offset is <error>\n", sect, buf);
                //  return 2;
                //}
                //uint off = tmp_word >> 18;
                if (class == 0) {
                    // Entries may or may not already have segment info.   For example,
                    // bound_active1 has a class 3 definition for Segment "wire_proc",
                    // followed by class 0 definitions for: wire_proc$unwire_proc and unwire_proc
                    if (msgs)
                        out_msg("Text %s: link %o|%#o\n", buf, segno, thing_relp);
                    // if (strncmp(entryname, buf, entryp-entryname) == 0)
                    if (strchr(buf, '$') != NULL)
                        seginfo_add_linkage(segno, thing_relp, buf);
                    else {
                        strcpy(entryp, buf);
                        seginfo_add_linkage(segno, thing_relp, entryname);
                    }
                } else if (class == 2) {
                    if (msgs)
                        out_msg("Name is %s; offset is %#o within %s section\n", buf, thing_relp, sect);
#if 0
                    if (strcmp(buf, "bind_map") == 0) {
                    } else if (strcmp(buf, "symbol_table") == 0) {
                    } else
                            ...
#endif
                } else
                    if (msgs)
                        out_msg("Name is %s with offset thing_relp = %#o within the %s section.\n", buf, thing_relp, sect);
            }
            defp = defs.wordno + fwdp;
        }
    }

    if (msgs && first_link != 0 && last_link != 0) {
        out_msg("\n");
        out_msg("Linkage section:\n");
        for (int link = first_link; link <= last_link; link += 2) {
            TPR.TSR = linkage.PR.snr;
            if (fetch_pair(linkage.wordno + link, &word0, &word1) != 0)
                return 2;
            if ((word0 & 077) == 043) {
                // snapped link
                AR_PR_t pr;
                if (words2its(word0, word1, &pr) != 0)
                    out_msg("Link pair at %#o: %012llo %012llo: snapped link -- bad ptr\n", link, word0, word1);
                else
                    out_msg("Link pair at %#o: %012llo %012llo: snapped link %o|%#o\n", link, word0, word1, pr.PR.snr, pr.wordno);
            } else if ((word0 & 077) == 046) {
                // unsnapped link
                uint exp_ptr = word1 >> 18;
                // out_msg("Link pair at %#o: %012llo %012llo: unsnapped link with exp-ptr %#o.\n", link, word0, word1, exp_ptr);
                out_msg("Link pair at %#o: %#o|%#04o: unsnapped link ", link, TPR.TSR, linkage.wordno + link);
                t_uint64 word2;
                TPR.TSR = defs.PR.snr;
                if (fetch_word(defs.wordno + exp_ptr, &word2) != 0) {
                    out_msg("Cannot read expression word at %o|%o\n", TPR.TSR, defs.wordno + exp_ptr);
                    return 2;
                }
                uint offset = word2 >> 18;
                t_uint64 a, q;
                if (fetch_word(defs.wordno + offset, &a) != 0) {
                    out_msg("... exp-word at %o|%o: %012llo: offset %o|%o => <error>\n", linkage.PR.snr, exp_ptr, word2, TPR.TSR, offset);
                    return 2;
                }
                if (fetch_word(defs.wordno + offset + 1, &q) != 0) {
                    out_msg("... exp-word at %o|%o: %012llo: offset %o|%o => <error>\n", linkage.PR.snr, exp_ptr, word2, TPR.TSR, offset);
                    return 2;
                }
                // out_msg("\texp-word %012llo: offset %o => type-pair (%#llo,%#llo)\n", word2, offset, a, q);
                int exp_offset = word2 & MASK18;
                int typ = a >> 18;
                if (typ == 0 || typ > 6) {
                    out_msg("of unknown type %d.\n", typ);
                    continue;
                } else if (typ == 1 || typ == 2 || typ == 5) {
                    out_msg("of Type %d.\n", typ);
                    continue;
                } else {
                    // types 3, 4, and 6
                    char sname[513];
                    char ename[513];
                    int ql = q & MASK18;
                    int qu = q >> 18;
                    if (qu == 0)
                        *sname = 0;
                    else if (fetch_acc(defs.wordno + qu, sname) != 0)
                        return 2;
                    if (ql == 0)
                        *ename = 0;
                    else if (fetch_acc(defs.wordno + ql, ename) != 0)
                        return 2;
                    if (exp_offset == 0)
                        out_msg("of type %d to %s$%s\n", typ, sname, ename);
                    else
                        out_msg("of type %d to %s$%s%+d\n", typ, sname, ename, exp_offset);
                }
            } else {
                out_msg("Link pair at %#o: %012llo %012llo: unrecognized/ignorable.\n", link, word0, word1);
            }
        }
    }

    return 0;
}

// ============================================================================

int fetch_acc(uint addr, char bufp[513])
{
    // Fetch an "ACC" string which is a string with a 9bit length prefix
    t_uint64 word;
    char *cp = bufp;
    uint n;
    if (fetch_word(addr++, &word) != 0)
        return 1;
    n = word >> 27;
    bufp[n] = 0;
    *cp++ = (word >> 18) & 255;
    *cp++ = (word >> 9) & 255;
    *cp++ = word & 255;
    for (int i = 0; i < n/4; ++i) {
        if (fetch_word(addr++, &word) != 0)
            return 1;
        *cp++ = (word >> 27) & 255;
        *cp++ = (word >> 18) & 255;
        *cp++ = (word >> 9) & 255;
        *cp++ = word & 255;
    }
    return 0;
}

// ============================================================================

static void word2pr(t_uint64 word, AR_PR_t *prp)
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

    t_addr lo = 0, hi = 0;
    int n = -1;
    char *lo_text = tailp;
    lo = (*sim_vm_parse_addr)(NULL, tailp, &tailp);
    if (! tailp) {
        out_msg("Cannot parse first address in address range.\n");
        return 1;
    }
    char *lo_tail = tailp;
    tailp += strspn(tailp, "    ");
    if (*tailp == 0) {
        hi = lo;
    } else {
        if (*tailp == '/') {
            ++ tailp;
            hi = 0;
            if (strspn(tailp, "01234567     ") == strlen(tailp))
                (void) sscanf(tailp, "%o", (unsigned *) &n);
            else {
                out_msg("Expecting a count after '/'.\n");
                return 1;
            }
        } else if (*tailp == '-') {
            ++ tailp;
            tailp += strspn(tailp, "    ");
            n = -1;
            hi = (*sim_vm_parse_addr)(NULL, tailp, &tailp);
            if (! tailp) {
                out_msg("Cannot parse second address.\n");
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
    if (n == -1)
        out_msg("Searching for '%s' in %#o .. %#o\n", search, lo, hi);
    else
        out_msg("Searching for '%s' in %#o/%#o\n", search, lo, n);
    int addr = -1;
    char *matchp = search;
    unsigned char stupid[2][513];
    memset(stupid, 0, sizeof(stupid));
    int nstupid = 0;
    int which = 0;
    int found = 0;
    int seg, offset;
    uint saved_seg = TPR.TSR;
    addr_modes_t saved_amode = get_addr_mode();
    set_addr_mode(APPEND_mode);
    if (strchr(lo_text, '|') == 0) {
        seg = -1;
        addr = lo;
    } else {
        char dummy;
        if (sscanf(lo_text, "%o | %o %c", (unsigned *) &seg, (unsigned *) &offset, &dummy) == 2) {
            out_msg("Using seg %o, initial offset %o\n", seg, offset);
            TPR.TSR = seg;
        } else {
            out_msg("Cannot parse segmented address %s.\n", lo_text);
            seg = -1;
        }
    }
    for (;;) {
        t_uint64 word;
        if (seg >= 0) {
            if (n >= 0) {
                if (--n == 0)
                    break;
            } else {
                // BUG: infinite loop, <seg>|<off1> - <seg>|<off2> not supported here
            }
            addr = offset;
            if (fetch_word(addr, &word) != 0) {
                TPR.TSR = saved_seg;
                set_addr_mode(saved_amode);
                return 1;
            }
        } else {
            if (n >= 0) {
                if (addr - lo == n)
                    break;
            } else
                if (addr > hi)
                    break;
            if (fetch_abs_word(addr, &word) != 0) {
                TPR.TSR = saved_seg;
                set_addr_mode(saved_amode);
                return 1;
            }
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
        /* Advance to the next word */
        if (seg >= 0) {
            ++offset;
            // addr = offset;
        } else
            ++addr;
    }

    TPR.TSR = saved_seg;
    set_addr_mode(saved_amode);
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
            fprintf(stderr, "procs: cannot parse '%s'\n", buf);
    }
    return 0;
}

// ============================================================================

static int seginfo_show_all(int seg, int first)
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

// ============================================================================

void flush_logs()
{
    if (sim_log != NULL)
        fflush(sim_log);
    if (sim_deb != NULL)
        fflush(sim_deb);
}
