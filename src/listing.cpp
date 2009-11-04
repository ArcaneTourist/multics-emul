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

    // BUG: we require the user to tell us the relocation info -- fixed?
    source_file& src = seginfo_add_source_file(segno, s, offset);

    out_msg("Loading PL/1 compiler listing, %s\n", s);
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
    int seen_blocks = 0;        // Seen header "BLOCK NAME ... STACK SIZE .. WHO SHARES"
    int doing_storage_req = 0;  // Seen headers "STORAGE REQUIREMENTS" and "Object Text..."
    char auto_frame[sizeof(lbuf)] = {0};    // State for listing of automatics
    int auto_join_next = 0; // Stat flag for automatic varibles re line continuation
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
                    fprintf(stderr, "Found reference to non-existant {line %d, loc %#o} at input line %d.  Expecting line number in range 1..%u\n", lineno, loc, nlines + 1, lines.size());
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

        // TODO: Use a single state variable
        // TODO: Generalize the line scanning code

#if 0
        // Look for declarations -- unfinished code to pick up type info for automatics and/or handle based and external statics
        int doing_dcl_stmt = alm_lineno == 5;
        if (!doing_dcl_stmt) {
            doing_dcl_stmt = str_pmatch(lbufp, "NAMES DECLARED BY DECLARE STATEMENT") == 0;
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
            const char *s = lbufp + strspn(lbufp, " \t");
            int l = strcspn(s, " \t");
            char dcl_name[sizeof(lbuf)];
            strncpy(dcl_name, s, l);
            dcl_name[l] = 0;
            s += l;
            s += strspn(s, " \t");
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
            // next word is one of: parameter, automatic, constant, based, "external static"
            // following tokens include: structure, pointer, fixed bin(#,#), char(#), bit(#)
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
                    string entry_name = src.seg_name;
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
        if (! seen_blocks)
            seen_blocks = str_pmatch(lbufp, "BLOCK NAME");
        else {
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
            const char *pat;
            char *p;
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
                entry_point *epp = src.find_entry(block);
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
                            // BUG: probably need to find other entries and point them at this stack
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
                    sframe.automatics[auto_loc] = auto_id;
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
    return ret;
}

// ============================================================================
