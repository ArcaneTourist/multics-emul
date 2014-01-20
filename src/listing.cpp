/*
    listing.cpp -- parse PL1 compiler listings for location info that the
    simulator can use for displaying the source lines associated with
    the assembly being executed or displayed.

    TODO: Now that this has grown and now that we're extracting multiple
    items from the listing, it's probably better to redo this in yacc...

    TODO: more info from the listings:
        wire_and_mask;     proc;  lines 2733..2738..2348; offset 017525
        stack size info
        entry types: internal proc, external proc, entry -- done
        from mixed asm/source:
            BEGIN PROCEDURE
            ENTRY TO (note, procs get an entry-to too)
            ENTRY TO (may next in proc)
            END PROCEDURE (may nest)
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

using namespace std;
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <vector>
#include "sim_defs.h"
#include "seginfo.hpp"

extern "C" void out_msg(const char* format, ...);

static int listing_parse(FILE *f, source_file &src);
static int load_listing(FILE *f, source_file &src);

#if 0
static int consume(int lineno, unsigned loc, const char *line)
{
    printf("Consuming line %d at loc %o: %s<<<\n", lineno, loc, line);
    return 0;
}

int main()
{
    int r = load_listing(stdin, consume);
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
    char c;

    int debug = 0;
    if (sscanf(s, "-d%c", &c) == 1 && isspace(c)) {
        debug = 1;
        s += strcspn(s, " \t");
        s += strspn(s, " \t");
    }

    int n = strspn(s, "01234567");
    char sv = s[n];
    s[n] = 0;
    unsigned segno;
    if (sscanf(s, "%o %c", &segno, &c) != 1) {
        out_msg("xlist: Expecting a octal segment number.\n");
        buf[n] = sv;
        return 1;
    }
    s[n] = sv;
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

    // BUG: we require the user to tell us the relocation info -- fixed?
    source_file& src = seginfo_add_source_file(segno, s, offset);

    out_msg("Loading PL/1 compiler listing, %s\n", s);
    int ret = load_listing(f, src);
    if (fclose(f) != 0) {
        perror(s);
        ret = -1;
    }
    if (ret != 0)
        out_msg("xlist: Problems loading listing %s for segment %o|%o.\n", s, segno, offset);
    if (debug) {
        cout << "Dump of file " << s << ":" << simh_nl;
        src.print(cout, 4);
    }
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

static int load_listing(FILE *f, source_file &src)
{
    int ret = listing_parse(f, src);
    if (ret != 0)
        return ret;

    // No C++0x yet... for (auto it = src.entries.begin(); it != src.entries.end(); ++it)
    map<int,entry_point>::iterator it;
    const entry_point* proc = NULL;
    for (it = src.entries.begin(); it != src.entries.end(); ++it) {
        int offset = (*it).first;
        entry_point& ep = (*it).second;
        if (ep.is_proc)
            proc = &ep;
        else {
            if (!ep.is_proc && ! ep._stack && ! ep.stack_owner) {
                if (proc)
                    ep.stack_owner = proc;
                else
                    cout << "Discovered that " << ep.name << " is an entry point to an unknown procedure.\n";
            }
        }
    }
    
    return ret;
}

// ============================================================================

static int listing_parse(FILE *f, source_file &src)
{
    // Scans input.  Adds lines to source_file object.
    // Works by first seeing all the source lines and saving them in a list.
    // Next, while reading the mapping of source line number to code offset,
    // saved lines are added to the source_file object.

    char lbuf[500];
    vector<string> lines;           // index is line_no
    vector<unsigned> incl_files;    // index is incl file num, value is lines[] index
    unsigned nlines;
    map<string,var_info> vars;
    
    int ret = 0;

    lines.reserve(800);

    int seen_ll_hdr = 0;    // Seen the "LINE LOC LINE LOC ..." header ?
    int seen_line_locs = 0;     // Seen all of the actual line locations that follow the ll header
    int asm_seen_begin = 0;     // Seen the ^"BEGIN" that starts the mixed source/assembly listings
    int source_finished = 0;    // Seen something that says no more source (just statics and info)
    int asm_consume_next = 0;   // State flag for mixed source/assembly
    int asm_is_end = 0;         // State flag for mixed source/assembly
    int seen_automatic = 0;     // Seen header "STORAGE FOR AUTOMATIC VARIABLES"
    int seen_blocks = 0;        // Seen header "BLOCK NAME ... STACK SIZE .. WHO SHARES"
    int doing_storage_req = 0;  // Seen headers "STORAGE REQUIREMENTS" and "Object Text..."
    char auto_frame[sizeof(lbuf)] = {0};    // State for listing of automatics
    int auto_join_next = 0; // Stat flag for automatic varibles re line continuation
    int doing_dcl_stmt = 0;     // Seen header "NAMES DECLARED BY DECLARE STATEMENT."
    int seen_explicit_dcl = 0;  // Seen header for names declared by explicit dcls
    // using strcmp and strspn is very portable, but not as pretty as other approaches...
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
            src.seg_name = lbufp;
        }

        // Check for a source line
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
                unsigned fileno;
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
                        fprintf(stderr, "listing_parse: found line %u, expecting line %lu\n", lineno, (unsigned long) lines.size()+1);
                        errno = EINVAL;
                        return -1;
                    }
                    lines.push_back(line);
                } else if (sscanf(lbufp, "%u %u %c", &fileno, &lineno, &c) == 2) {
                    // probably an included line with multiple line numbers
                    lbufp[9] = c;
                    if (fileno == incl_files.size()) {
                        // more lines from the previously seen include
                    } else if (fileno == incl_files.size()+1) {
                        incl_files.push_back(lines.size());
                        // fprintf(stderr, "listing_parse: found include file # %u at line %u\n", fileno, lines.size() + 1);
                    } else if (fileno < incl_files.size()+1) {
                        // prior include file was nested
                    } else
                        fprintf(stderr, "listing_parse: found include file # %u; expecting # %lu\n", fileno, (unsigned long) incl_files.size()+1);
                } else {
                    lbufp[9] = c;
                    fprintf(stderr, "listing_parse: unexpected source line near line %lu: %s\n", (unsigned long) lines.size()+1, lbufp + 9);
                }
            }
            continue;
        }

        // Check for LINE/LOC map
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
                    fprintf(stderr, "Found reference to non-existant {line %d, loc %#o} at input line %d.  Expecting line number in range 1..%lu\n", lineno, loc, nlines + 1, (unsigned long) lines.size());
                    errno = EINVAL;
                    return -1;
                }
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
                alm_lineno = 0;
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

        // TODO: Use a single state variable
        // TODO: Generalize the line scanning code

#if 1
        // Look for declarations -- unfinished code to pick up type info for automatics and/or handle based and external statics
        // int doing_dcl_stmt = alm_lineno == 5;
        if (!doing_dcl_stmt) {
            doing_dcl_stmt = str_pmatch(lbufp, "NAMES DECLARED BY DECLARE STATEMENT.") == 0;
        } else {
            const char *section = "dcl stmt";
            seen_explicit_dcl = str_pmatch(lbufp, "NAMES DECLARED BY EXPLICIT CONTEXT") == 0 ||
                str_pmatch(lbufp, "NAME DECLARED BY EXPLICIT CONTEXT") == 0;
            if (seen_explicit_dcl) {
                doing_dcl_stmt = 0;
                continue;
            }
            if (strstr(lbufp, "NAMES DECLARED BY") != NULL) {
                doing_dcl_stmt = 0;
                continue;
            }
            if (strstr(lbufp, "NAME DECLARED BY") != NULL) {
                doing_dcl_stmt = 0;
                continue;
            }
            if (str_pmatch(lbufp, "STORAGE REQUIREMENTS FOR THIS PROGRAM") == 0) {
                doing_dcl_stmt = 0;
                continue;
            }
            if (strspn(lbufp, " \t\f") > 80)
                continue;   // continuation line
            // First pull off the name
            // fprintf(stderr, "DEBUG: Parsing: %s\n", lbufp);
            char *s = lbufp + strspn(lbufp, " \t");
            int l = strcspn(s, " \t");
            char dcl_name[sizeof(lbuf)];
            strncpy(dcl_name, s, l);
            dcl_name[l] = 0;
            s += l;
            s += strspn(s, " \t");
            // Check next token to see if it's an optional offset
            if (s - lbufp < 27) {
#if 1
                s += strspn(s, "()0123456789");
                s += strspn(s, " \t");
#else
                l = strspn(s, "()0123456789");
                if (l < 6) {
                    char offset[sizeof(lbuf)];
                    strncpy(offset, s, l);
                    offset[l] = 0;
                    fprintf(stderr, "\tfound %u char sym offset %s at %u.\n", l, offset, s - lbufp);
                    s += l;
                    s += strspn(s, " \t");
                    fprintf(stderr, "\tParse string now: %s\n", s);
                } else {
                    fprintf(stderr, "\tfound odd %u char offset at %u: %s\n", l, s- lbufp, s);
                }
#endif
            }

            // Check next token to see if it's an optional location
            int have_dcl_loc = 0;
            unsigned int dcl_loc;
            if (strspn(s, "01234567") == 6) {
                if (sscanf(s, "%o", &dcl_loc) != 1) {
                    fprintf(stderr, "%s: Found garbled entry at input line %d (no location): %s.\n", section, nlines, lbufp);
                    errno = EINVAL;
                    ret = -1;
                    continue;
                }
                s += 6;
                s += strspn(s, " \t");
                have_dcl_loc = 1;
            } else if (strspn(s, "01234567") != 0) {
                // based info of form # or #(#)
            } else {
                have_dcl_loc = 0;
            }
            // Next word is a storage class.  One of: automatic, based, builtin function,
            // constant, external static, parameter, or stack reference
            var_info::vartype t = var_info::unknown;
            unsigned sz = 0;
            unsigned sz2 = 0;
            if (str_pmatch(s, "automatic") == 0) {
                s += strlen("automatic");
                s += strspn(s, " \t");
                // possible next tokens include: bit(#), char(#), fixed bin(#,#),
                // picture(#), pointer, structure, and varying char(#)
                if (str_pmatch(s, "pointer") == 0) {
                    t=var_info::ptr;
                    sz = 72;    // could be 72 bits or 36 bits
                } else if (str_pmatch(s, "char") == 0) {
                    t=var_info::str;
                } else if (str_pmatch(s, "bit") == 0) {
                    t=var_info::bit;
                } else if (str_pmatch(s, "varying char") == 0) {
                    t=var_info::vchar;
                } else if (str_pmatch(s, "fixed bin") == 0) {
                    t=var_info::fixedbin;
                } else {
                    // ignoring
                }
                if (t != var_info::unknown && sz == 0) {
                    s = strchr(s, '(');
                    if (s) {
                        char *s2 = strchr(s, ')');
                        if (s2) {
                            *s2 = 0;
                            (void) sscanf(s+1, "%u,%u", &sz, &sz2);
                        }
                    }
                    if (t == var_info::str)
                        sz *= 9;
                }
            }
            if (have_dcl_loc) {
                // fprintf(stderr, "%s: Found name %s at loc %#o with info %s\n", 
                    // section, dcl_name, dcl_loc, s);
            } else {
                // fprintf(stderr, "%s: Found name %s at unspecified loc with info %s\n", section, dcl_name, s);
            }
            vars[dcl_name] = var_info(dcl_name, t, sz, sz2);
        }
#endif

        // Look for entry point declarations
        if (!seen_explicit_dcl) {
            seen_explicit_dcl = str_pmatch(lbufp, "NAMES DECLARED BY EXPLICIT CONTEXT") == 0 ||
                str_pmatch(lbufp, "NAME DECLARED BY EXPLICIT CONTEXT") == 0;
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
                // printf("Explicit dcls: Found entry %s at loc %06o\n", edcl_name, edcl_loc);
                if (! src.seg_name.empty()) {
                    string entry_name;
                    if (strchr(edcl_name, '$') != 0) {
                        entry_name = edcl_name;
                    } else
                        entry_name = src.seg_name + '$' + edcl_name;
                    // Note: no relocation...
                    if (src.entries.find(edcl_loc) != src.entries.end()) {
                        fflush(stdout);
                        fprintf(stderr, "Found multiple entry points starting at offset %u at input line %d\n", edcl_loc, nlines + 1);
                        errno = EINVAL;
                        return -1;
                    }
                    src.entries[edcl_loc] = entry_point(entry_name, edcl_loc);
                    src.entries_by_name[edcl_name] = &src.entries[edcl_loc];    // BUG: this should be private
                    // src.entries[edcl_name].source = &src;
                    entry_point& ep = src.entries[edcl_loc];
                    ep.source = &src;
                    // cout << "Entry " << entry_name << " added." << "\r\n";
                }
            }
        }

        // Look for storage requirements (which tells us where the last entry point ends)
        if (!doing_storage_req) {
            doing_storage_req = str_pmatch(lbufp, "STORAGE REQUIREMENTS FOR THIS PROGRAM") == 0;
        } else {
            if (str_pmatch(lbufp, "Object\tText\tLink\tSymbol\tDefs\tStatic") == 0)
                continue;
            else if (str_pmatch(lbufp, "Start") == 0) {
                int nargs;
                char dummy;
                unsigned obj, text, link, sym, defs, statics;
                if ((nargs = sscanf(lbufp, "Start %o %o %o %o %o %o %c", &obj, &text, &link, &sym, &defs, &statics, &dummy)) != 6) {
                    cerr << "Garbled storage requirement line: " << lbufp << "\r\n";
                    doing_storage_req = 0;
                } else {
                    if (obj != 0 || text != 0) {
                        cerr << "Expecting code generated to address zero." << "\r\n";
                        doing_storage_req = 0;
                    }
                }
            } else if (str_pmatch(lbufp, "Length") == 0) {
                int nargs;
                char dummy;
                unsigned obj, text, link, sym, defs, statics;
                if ((nargs = sscanf(lbufp, "Length %o %o %o %o %o %o %c", &obj, &text, &link, &sym, &defs, &statics, &dummy)) != 6) {
                    cerr << "Garbled storage requirement line: " << lbufp << "\r\n";
                } else {
                    src._hi = text;
                }
                doing_storage_req = 0;
            } else {
                doing_storage_req = 0;
                if (src._hi < 0) {
                    cout << "Giving up on storage requirement scanning without finding text size..." << "\r\n";
                    cout << "Killer line is: <<<" << lbufp << ">>>\r\n";
                }
            }
        }

        // Todo: Look for block list (which indicates which blocks (entry points) share stack frames
        if (! seen_blocks) {
            seen_blocks = str_pmatch(lbufp, "BLOCK NAME") == 0;
        } else {
            if (!seen_automatic) {
                seen_automatic = str_pmatch(lbufp, "STORAGE FOR AUTOMATIC VARIABLES") == 0;
                if (seen_automatic) {
                    seen_blocks = 0;
                    continue;
                }
            }
            if (str_pmatch(lbufp, "begin block") == 0)
                continue;
            const char *s = lbufp + strspn(lbufp, " \t");
            int l = strcspn(s, " \t");
            char block[sizeof(lbuf)];
            strncpy(block, s, l);
            block[l] = 0;
            entry_point *epp = src.find_entry(block);
            if (epp != NULL)
                epp->is_proc = 1;
            s += l;
            s += strcspn(s, " \t");
            if ((l = strspn(s, "01234567")) != 0) {
                // Numeric, so it's a stack size
                s += l;
                s += strcspn(s, " \t");
                continue;
            }
            // BUG: It turns out that we didn't really need to know that a proc shares another's
            // stack frame because the simulator can just look at changes to pr6 and assume that
            // if pr6 hasn't changed then whoever is running is sharing that frame.
            // OTOH, that doesn't work for procs called via an entry point.
            const char *pat;
            const char *p;
            if ((p = strstr(s, pat = "shares stack frame of external procedure")) == NULL)
                p = strstr(s, pat = "shares stack frame of internal procedure");
            if (p != NULL) {
                char stack_owner[sizeof(lbuf)];
                p += strlen(pat);
                p += strspn(p, " \t");
                strcpy(stack_owner, p);
                for (l = strlen(stack_owner)-1; l > 0 && stack_owner[l] == ' '; --l)
                    stack_owner[l] = 0;
                if (stack_owner[l = strlen(stack_owner)-1] == '.')
                    stack_owner[l] = 0;
                if (epp == NULL) {
                    cerr << "Listing: Cannot find entry point " << block  << ", so cannot record stack info." << "\r\n";
                    ret = 1;
                    errno = EINVAL;
                } else {
                    const entry_point *ownerp = src.find_entry(stack_owner);
                    if (ownerp == NULL) {
                        cerr << "Listing: Cannot find entry point " << stack_owner  << ", so cannot point" << block << "'s stack at it." << "\r\n";
                        errno = EINVAL;
                        ret = 1;
                    } else
                            epp->stack_owner = ownerp;
                }
            }
        }

        // Look for map of automatic locations
        if (!seen_automatic) {
            seen_automatic = str_pmatch(lbufp, "STORAGE FOR AUTOMATIC VARIABLES") == 0;
            if (seen_automatic)
                seen_blocks = 0;
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
                    if (src.stack_frames.find(auto_frame) != src.stack_frames.end()) {
                        fprintf(stderr, "Automatic variables: Duplicate stack frame '%s' at input line %d.\n", auto_frame, nlines);
                        errno = EINVAL;
                        ret = -1;
                    } else {
                        stack_frame& sframe = src.stack_frames[auto_frame];
                        sframe.size = -1;
                        entry_point *ep;
                        if ((ep = src.find_entry(auto_frame)) == NULL) {
                            cerr << "Stack Frame: Cannot find entry point for stack frame owner, " << auto_frame << "\r\n";
                            sframe.owner = NULL;
                        } else {
                            // NOTE: Caller should find other entries and point them at this stack
                            // cout << "Stack Frame: Found entry point for " << auto_frame << "\r\n";
                            if (ep->_stack != NULL)
                                cerr << "Entry " << ep->name << " already has a stack frame!" << "\r\n";
                            else
                                ep->_stack = &src.stack_frames[auto_frame];
                            sframe.owner = ep;
                        }
                    }
                }
            } else {
                // Got a 6 digit location
                if (auto_join_next) {
                    // oddity on prior line
                    fprintf(stderr, "Automatic variables: Found garbled stack frame listing at input line %d (too few args).\n", nlines);
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
            int have_auto = 0;
            if (auto_join_next) {
                auto_join_next = 0;
                strcpy(auto_block, s);
                // printf("Automatic Variables: Found (continued): Stack Frame %s, loc %06o, id %s, block %s\n", auto_frame, auto_loc, auto_id, auto_block);
                have_auto = 1;
            } else {
                memset(auto_block, 0, sizeof(auto_block));
                int nargs;
                if ((nargs = sscanf(s, "%o %s %9999c", &auto_loc, auto_id, auto_block)) == 3) {
                    // printf("Automatic Variables: Found: Stack Frame %s, loc %06o, id %s, block %s\n", auto_frame, auto_loc, auto_id, auto_block);
                    have_auto = 1;
                } else if (nargs == 2) {
                    auto_join_next = 1;
                } else {
                    fflush(stdout);
                    fprintf(stderr, "Automatic variables: Found garbled stack frame listing at input line %d (too few args).\n", nlines + 1);
                    errno = EINVAL;
                    ret = -1;
                }
            }
            if (have_auto) {
                map<string,stack_frame>::iterator it = src.stack_frames.find(auto_frame);
                if (it == src.stack_frames.end())
                    printf("stack frame %s not found...\n", auto_frame);
                else {
                    stack_frame& sframe = (*it).second;
                    // sframe.automatics[auto_loc] = auto_id;
                    map<string,var_info>::iterator vit = vars.find(auto_id);
                    if (vit != vars.end())
                        sframe.automatics[auto_loc] = (*vit).second;
                    else
                        sframe.automatics[auto_loc] = var_info(auto_id, var_info::unknown, 0);
                }
            }
        }
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

    // Fixup pointer types -- some are one word and some are two workds
    for (map<string,stack_frame>::iterator sit = src.stack_frames.begin(); sit != src.stack_frames.end(); ++sit) {
        stack_frame& sframe = (*sit).second;
        // map<string,var_info>& autos = sframe.automatics;
        map<int,var_info>::iterator vit;
        for (vit = sframe.automatics.begin(); vit != sframe.automatics.end(); ++ vit) {
            int off = (*vit).first;
            var_info& vi = (*vit).second;
            if (vi.type == var_info::ptr && vi.size == 72)
                if (sframe.automatics.find(off + 1) != sframe.automatics.end())
                    vi.size = 36;
        }
    }

    // Fixup entries that quietly share their parent's stack frame
    for (map<string,entry_point*>::const_iterator eit = src.entries_by_name.begin(); eit != src.entries_by_name.end(); ++eit) {
        entry_point *ep = (*eit).second;
        if (ep->stack() != NULL)
            continue;
        string name = (*eit).first;
        size_t cpos = name.find('$');
        if (cpos == string::npos) {
            if (src.stack_frames.size() == 0)
                fprintf(stderr, "LISTING: Entry %s has no stack and neither does anything in this source file.\n", name.c_str());
            else if (src.stack_frames.size() != 1) {
                // fprintf(stderr, "LISTING: Entry %s has no stack.\n", name.c_str());
            } else {
                const stack_frame& sf = (*src.stack_frames.begin()).second;
                const entry_point* parent = sf.owner;
                if (parent == NULL)
                    fprintf(stderr, "LISTING: Entry %s has no stack and the single stack for this source file doesn't have an owner!.\n", name.c_str());
                else {
                    // fprintf(stderr, "LISTING: Entry %s will be assumed to share the stack frame of %s.\n", name.c_str(), parent->name.c_str());
                    ep->stack_owner = parent;
                }
            }
        } else {
            name.erase(cpos);   // cannot just say: name[cpos] = 0;
            entry_point* parent = src.find_entry(name);
#if 0
            if (parent == NULL) {
                // Parent may be a procedure, e.g., "TURN_OFF", and we may
                // have renamed it, e.g., real_initializer$TURN_OFF
                fprintf(stderr, "LISTING: DEBUG: Entry %s has no stack and parent entry %s does not exist...\n", ep->name.c_str(), name.c_str());
                name = src.seg_name + '$' + name;
                parent = src.find_entry(name);
            }
#endif
            if (parent == NULL) {
                fprintf(stderr, "LISTING: DEBUG: Entry %s has no stack and parent entry %s does not exist.\n", ep->name.c_str(), name.c_str());
                fprintf(stderr, "LISTING: DEBUG: Entries (by name) are:\n");
                for (map<string,entry_point*>::const_iterator xit = src.entries_by_name.begin(); xit != src.entries_by_name.end(); ++xit) {
                    fprintf(stderr, "\t%s => %s.\n", (*xit).first.c_str(), (*xit).second->name.c_str());
                }
            } else {
                if (parent->stack() == NULL)
                    fprintf(stderr, "LISTING: Entry %s has no stack and neither does parent %s.\n", ep->name.c_str(), name.c_str());
                else {
                    // fprintf(stderr, "LISTING: Note: entry %s has no stack but parent %s has one.\n", ep->name.c_str(), name.c_str());
                    ep->stack_owner = parent;
                }
            }
        }
    }

    return ret;
}

// ============================================================================

