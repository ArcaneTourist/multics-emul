/*
    listing.cpp -- parse PL1 compiler listings for location info that the
    simulator can use for displaying the source lines associated with
    the assembly being executed or displayed.

    TODO: Now that we're extracting multiple items from the listing, it's
    probably better to redo this in yacc...

    TODO: more info from the listings:
        wire_and_mask;     proc;  lines 2733..2738..2348; offset 017525
        stack size info
        entry types: internal proc, external proc, entry
        from mixed asm/source:
            BEGIN PROCEDURE
            ENTRY TO (note, procs get an entry-to too)
            ENTRY TO (may next in proc)
            END PROCEDURE (may nest)
*/

using namespace std;
#include "seginfo.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <vector>

#include "sim_defs.h"
#include "symtab.h"
extern "C" void out_msg(const char* format, ...);

int listing_parse(FILE *f, source_file &src);

#if 0
static int consume(int lineno, unsigned loc, const char *line)
{
    printf("Consuming line %d at loc %o: %s<<<\n", lineno, loc, line);
    return 0;
}

int main()
{
    int r = listing_parse(stdin, consume);
    printf("Return code is %d\n", r);
}
#endif

// ============================================================================

// static unsigned listing_offset;
// static int listing_segno;

extern "C" int cmd_load_listing(int32 arg, char *buf)
{
    // Implements the "xlist" interactive command

    if (*buf == 0) {
        out_msg("USAGE xlist <segment number> <pathname>\n");
        return 1;
    }
    char *s = buf + strspn(buf, " \t");
    int n = strspn(buf, "01234567");
    char sv = buf[n];
    buf[n] = 0;
    unsigned segno;
    char c;
    if (sscanf(s, "%o %c", &segno, &c) != 1) {
        out_msg("xlist: Expecting a octal segment number.\n");
        buf[n] = sv;
        return 1;
    }
    buf[n] = sv;
    s += n;

    // listing_segno = segno;

    int offset;
    if (*s == '|' || *s == '$') {
        ++s;
        int n = strspn(s, "01234567");
        char sv = s[n];
        s[n] = 0;
        char c;
        if (sscanf(s, "%o %c", (unsigned*) &offset, &c) != 1) {
            out_msg("xlist: Expecting a octal offset, not '%s'.\n", s);
            buf[n] = sv;
            return 1;
        }
        s[n] = sv;
        s += n;
    } else
        offset = -1;

    s += strspn(s, " \t");
    FILE *f;
    if ((f = fopen(s, "r")) == NULL) {
        perror(s);
        return 1;
    }

    // BUG: we require the user to tell us the relocation info
    source_file& src = seginfo_add_source_file(segno, s, offset);

    int ret = listing_parse(f, src);
    if (fclose(f) != 0) {
        perror(s);
        ret = -1;
    }
    if (ret != 0)
        out_msg("xlist: Problems loading listing %s for segment %o|%o.\n", s, segno, offset);
    return ret;
}

// ============================================================================

static int str_pmatch(const char *buf, const char *s)
{
        // Prefix match -- Ignoring initial white space, does s match the beginning of buf?
        buf += strspn(buf, " \t\f");
        s += strspn(s, " \t\f");
        return strncmp(buf, s, strlen(s));
}


// ============================================================================

int listing_parse(FILE *f, source_file &src)
{
    // Scans input.  Adds lines to source_file object.
    // Works by first seeing all the source lines and saving them in a list.
    // Next, while reading the mapping of source line number to code offset,
    // saved lines are added to the source_file object.

    char lbuf[500];
    vector<string> lines;           // index is line_no
    unsigned nlines;
    
    int ret = 0;

    lines.reserve(400);

    int seen_ll_hdr = 0;    // Seen the "LINE LOC LINE LOC ..." header ?
    int seen_line_locs = 0;     // Seen all of the actual line locations that follow the ll header
    int asm_seen_begin = 0;     // Seen the ^"BEGIN" that starts the mixed source/assembly listings
    int source_finished = 0;    // Seen something that says no more source (just statics and info)
    int asm_consume_next = 0;   // State flag for mixed source/assembly
    int asm_is_end = 0;         // State flag for mixed source/assembly
    int seen_automatic = 0;     // Seen header "STORAGE FOR AUTOMATIC VARIABLES"
    char auto_frame[sizeof(lbuf)] = {0};    // State for listing of automatics
    int auto_join_next = 0; // Stat flag for automatic varibles re line continuation
    int seen_explicit_dcl = 0;  // Seen header for names declared by explicit dcls
    // using strcmp and strspn is very portable, but not as pretty as other approaches...
    string seg_name;
    for (nlines = 0; fgets(lbuf, sizeof(lbuf), f) != NULL; ++ nlines) {
        // check for buffer too small
        size_t n = strlen(lbuf);
        if (n == sizeof(lbuf) - 1) {
            fflush(stdout);
            fprintf(stderr, "Line %u too long: %s\n", nlines + 1, lbuf);
            errno = ERANGE;
            return -1;
        }
        // chomp
        if (lbuf[n-1] == '\n')
            lbuf[n-1] = 0;
        char *lbufp = lbuf;
        if (*lbufp == '\f')
            ++lbufp;
        if (*lbuf == 0)
            continue;
        if (str_pmatch(lbufp, "COMPILATION LISTING OF SEGMENT") == 0) {
            lbufp += strspn(lbufp, " \t");
            lbufp += strlen("COMPILATION LISTING OF SEGMENT");
            lbufp += strspn(lbufp, " \t");
            seg_name = lbufp;
        }

        if (!source_finished) {
            if (str_pmatch(lbufp, "SOURCE FILES USED") == 0) {
                source_finished = 1;
                continue;
            }
            // Check for a source line
            if (strspn(lbufp, " \t0123456789") >= 9) {
                char c = lbufp[9];
                if (c >= '0' && c <= '9') {
                    fflush(stdout);
                    fprintf(stderr, "Line %u has unexpected 10th char after source number: %s\n", nlines + 1, lbufp);
                    errno = EINVAL;
                    return -1;
                }
                lbufp[9] = 0;
                unsigned lineno;
                char dummy;
                if (sscanf(lbufp, "%u %c", &lineno, &c) == 1) {
                    lbufp[9] = c;
                    char *line = lbufp + 9;
                    line += strspn(line, " \t");
                    if (lineno != lines.size()+1) {
                        if (*line == 0) {
                            // OK, an empty line that probably represents use of an include file
                            continue;
                        }
                        fprintf(stderr, "listing_parse: found line %u, expecting line %u\n", lineno, lines.size()+1);
                        errno = EINVAL;
                        return -1;
                    }
                    lines.push_back(line);
                } else {
                    // probably an included line with multiple line numbers
                    lbufp[9] = c;
                }
            }
            continue;
        }
        if (!seen_ll_hdr) {
            // Look for header consisting of alternating "LINE" and "LOC"
            char *s = lbufp;
            while (*s) {
                s += strspn(s, " \t");
                if (strncmp(s, "LINE", 4) != 0)
                    break;
                s += 4;
                s += strspn(s, " \t");
                if (strncmp(s, "LOC", 3) != 0)
                    break;
                s += 3;
                seen_ll_hdr = 1;
            }
            seen_ll_hdr = seen_ll_hdr && (*s == 0);
            if (seen_ll_hdr)
                continue;
        }
        if (seen_ll_hdr && ! seen_line_locs) {
            // look for pairs of numbers -- line # and loc #
            int any = 0;
            int lineno = -1;
            char *s = lbufp;
            while (*s) {
                s += strspn(s, " \t");
                int n = strspn(s, "0123456789");
                if (n == 0)
                    break;
                (void) sscanf(s, "%d", &lineno);
                s += n;
                s += strspn(s, " \t");
                n = strspn(s, "01234567");
                if (n == 0)
                    break;
                unsigned loc;
                (void) sscanf(s, "%o", &loc);
                s += n;
                if (lineno <= 0 || (unsigned) lineno > lines.size()) {
                    fflush(stdout);
                    fprintf(stderr, "Found reference to non-existant {line %d, loc %#o} at input line %d.  Expecting line number in range 1..%u\n", lineno, loc, nlines + 1, lines.size());
                    errno = EINVAL;
                    return -1;
                }
                // BUG: doing relocation here instead of deferring
                if (src.lines.find(loc) != src.lines.end()) {
                    fflush(stdout);
                    fprintf(stderr, "Found multiple lines starting at offset %u at input line %d\n", loc, nlines + 1);
                    errno = EINVAL;
                    return -1;
                }
                src.lines[loc] = source_line(loc, lineno, lines[lineno-1]);
                lineno = -1;
                any = 1;
            }

            // If no pairs, we're finished
            if (!any)
                seen_line_locs = 1;

            // Check for additional garbage on the same line as the pairs
            if (any && (*s != 0 || lineno != -1)) {
                fflush(stdout);
                fprintf(stderr, "Oddly formatted line/loc entries on line %u: %s\n", nlines + 1, lbuf);
                errno = EINVAL;
                return -1;
            }
        }
        // Look for mixed source/assembly
        unsigned alm_lineno;
        if (!asm_seen_begin) {
            if (str_pmatch(lbufp, "BEGIN") == 0) {
                asm_seen_begin = 1;
                alm_lineno = -1;
                asm_consume_next = 0;
                continue;
            }
        } else {
            // Look for mixed source/assembly
            if (strspn(lbufp, "01234567") == 6 && (lbufp[6] == ' ' || lbufp[6] == '\t')) {
                unsigned loc;
                (void) sscanf(lbufp, "%o", &loc);
                if (asm_consume_next) {
                    src.lines[loc] = source_line(loc, alm_lineno, lines[alm_lineno-1]);
                    asm_consume_next = 0;
                }
            } else {
                int stmt;
                char dummy;
                if (sscanf(lbufp, " STATEMENT %d ON LINE %u %c", &stmt, &alm_lineno, &dummy) == 2) {
                    if (alm_lineno <= 0 || alm_lineno > lines.size()) {
                        fflush(stdout);
                        fprintf(stderr, "Found reference to non-existant {line %d} at input line %d.\n", alm_lineno, nlines + 1);
                        errno = EINVAL;
                        return -1;
                    }
                    asm_is_end = 0;
                    asm_consume_next = 1;
                } else {
                    const char *s = lbufp + strspn(lbufp, " \t");
                    if (!asm_is_end)
                        asm_is_end = strncmp(s, "end", 3) == 0  || strncmp(s, "END", 3) == 0;
                }
            }
        }
#if 1
        if (!seen_explicit_dcl) {
            seen_explicit_dcl = str_pmatch(lbufp, "NAMES DECLARED BY EXPLICIT CONTEXT") == 0;
        } else {
            if (strstr(lbufp, "NAMES DECLARED BY CONTEXT") != NULL) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (strstr(lbufp, "NAME DECLARED BY CONTEXT") != NULL) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (str_pmatch(lbufp, "STORAGE REQUIREMENTS FOR THIS PROGRAM") == 0) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (strspn(lbufp, " \t\f") > 80)
                continue;   // continuation line listing ref locations
            const char *s = lbufp + strspn(lbufp, " \t");
            int l = strcspn(s, " \t");
            char edcl_name[sizeof(lbuf)];
            strncpy(edcl_name, s, l);
            edcl_name[l] = 0;
            s += l;
            s += strspn(s, " \t");
            if (strspn(s, "01234567") != 6) {
                    fprintf(stderr, "Explicit dcls: Found garbled entry at input line %d (no location): %s.\n", nlines, lbufp);
                    errno = EINVAL;
                    ret = -1;
                    continue;
            } 
            unsigned edcl_loc;
            char edcl_type1[sizeof(lbuf)];
            char edcl_type2[sizeof(lbuf)];
            char edcl_rest[sizeof(lbuf)];
            if (sscanf(s, "%o %s %s %9999c", &edcl_loc, edcl_type1, edcl_type2, edcl_rest) != 4) {
                fflush(stdout);
                fprintf(stderr, "Explicit dcls: Found garbled entry at input line %d (too few args).\n", nlines);
                errno = EINVAL;
                ret = -1;
            }
            if (str_pmatch(edcl_type1, "constant") != 0) {
                fflush(stdout);
                fprintf(stderr, "Explicit dcls: Odd, entry isn't for a constant at input line %d.\n", nlines);
                errno = EINVAL;
                ret = -1;
            }
            if (str_pmatch(edcl_type2, "entry") == 0) {
                printf("Explicit dcls: Found entry %s at loc %06o\n", edcl_name, edcl_loc);
#if 1
                // untested
                if (! seg_name.empty()) {
                    string entry_name = seg_name;
                    entry_name += '$';
                    entry_name += edcl_name;
                    // Note: no relocation...
                    if (src.entries.find(edcl_loc) != src.entries.end()) {
                        fflush(stdout);
                        fprintf(stderr, "Found multiple entry points starting at offset %u at input line %d\n", edcl_loc, nlines + 1);
                        errno = EINVAL;
                        return -1;
                    }
                    src.entries[edcl_loc] = entry_point(entry_name, edcl_loc);
                }
#endif
            }
        }
#endif
#if 1
        if (!seen_automatic) {
            seen_automatic = str_pmatch(lbufp, "STORAGE FOR AUTOMATIC VARIABLES") == 0;
        } else {
            if (str_pmatch(lbufp, "STACK FRAME") == 0)
                continue;
            if (str_pmatch(lbufp, "THE FOLLOWING") == 0) {
                seen_automatic = 0;
                continue;
            }
            char auto_block[sizeof(lbuf)];
            const char *s = lbufp + strspn(lbufp, " \t");
            if (strspn(s, "01234567") != 6) {
                // Not a 6 digit location; it's either a 4 token line or a continuation line
                int l = strcspn(s, " \t");
                if (auto_join_next) {
                    strcpy(auto_block, s);
                } else {
                    strncpy(auto_frame, s, l);
                    auto_frame[l] = 0;
                    s += l;
                    s += strspn(s, " \t");
                }
            } else {
                // Got a 6 digit location
                if (auto_join_next) {
                    // oddity on prior line
                    fprintf(stderr, "Automatic variabless: Found garbled stack frame listing at input line %d (too few args).\n", nlines);
                    auto_join_next = 0;
                }
                if (*auto_frame == 0) {
                    fflush(stdout);
                    fprintf(stderr, "Found garbled stack frame listing at input line %d (found loc before frame).\n", nlines + 1);
                    errno = EINVAL;
                    ret = -1;
                    continue;
                }
            }
            unsigned auto_loc;
            char auto_id[sizeof(lbuf)];
            // char dummy;
            if (auto_join_next) {
                auto_join_next = 0;
                strcpy(auto_block, s);
                printf("Automatic Variables: Found (continued): Stack Frame %s, loc %06o, id %s, block %s\n", auto_frame, auto_loc, auto_id, auto_block);
                // TODO: make use of this info
            } else {
                memset(auto_block, 0, sizeof(auto_block));
                int nargs;
                if ((nargs = sscanf(s, "%o %s %9999c", &auto_loc, auto_id, auto_block)) == 3) {
                    printf("Automatic Variables: Found: Stack Frame %s, loc %06o, id %s, block %s\n", auto_frame, auto_loc, auto_id, auto_block);
                    // TODO: make use of this info
                } else if (nargs == 2) {
                    auto_join_next = 1;
                } else {
                    fflush(stdout);
                    fprintf(stderr, "Automatic variabless: Found garbled stack frame listing at input line %d (too few args).\n", nlines + 1);
                    errno = EINVAL;
                    ret = -1;
                }
            }
        }
#endif
    }

    if (ferror(f)) {
        int e = errno;
        fprintf(stderr, "listing_parse: error reading file.\n");
        errno = e;
        return -1;
    }
    if (asm_consume_next && ! asm_is_end) {
        fflush(stdout);
        fprintf(stderr, "Found dangling source line without corresponding assembly at EOF.\n");
        errno = EINVAL;
        return -1;
    }
    return ret;
}

// ============================================================================

