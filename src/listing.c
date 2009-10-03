/*
    listing.c -- parse PL1 compiler listings for location info that the
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

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>

#include "sim_defs.h"
#include "symtab.h"

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

static int add_line(char ***pool, size_t *max, int lineno, const char *line)
{
    // Maintains an array of source lines.   Used by listing_parse() which sees
    // all the source lines before it sees the mapping of line numbers to
    // code offset.

    line += strspn(line, " \t");
    if (lineno <= 0) {
        errno = EINVAL;
        return -1;
    }
    if (lineno >= *max) {
        char **newpool = calloc(*max * 2, sizeof(*newpool));
        if (newpool == NULL)
            return -1;
        memcpy(newpool, *pool, *max * sizeof(*newpool));
        *max *= 2;
        free(*pool);
        *pool = newpool;
    }
    if ((*pool)[lineno] != NULL) {
        fflush(stdout);
        fprintf(stderr, "Line # %d already seen once.  Prior: %s.  New: %s.\n", lineno, (*pool)[lineno], line);
        errno = EINVAL;
        return -1;
    }
    if (((*pool)[lineno] = strdup(line)) == NULL)
        return -1;
    return 0;
}


static int str_pmatch(const char *buf, const char *s)
{
        // Prefix match -- Ignoring initial white space, does s match the beginning of buf?
        buf += strspn(buf, " \t\f");
        s += strspn(s, " \t\f");
        return strncmp(buf, s, strlen(s));
}


int listing_parse(FILE *f, int(*consumer)(int lineno, unsigned loc, const char *line))
{
    // Scans input.  Calls (*consumer)() for each source line.
    // Works by first seeing all the source lines and saving them in a list.
    // Next, while reading the mapping of source line number to code offset,
    // the (*consumer)() is called and passed the appropriate previously saved line.

    // TODO: expand consumer to accept
    //      int loc, enum {line,automatic,entry} type, char * str, int lineno
    //  struct {
    //      enum {line, automatic, entry} type;
    //      union {
    //          struct {
    //                  int lineno; int loc; char * str;
    //          } line;
    //          struct {
    //                  int loc, char *name;
    //          } automatic;
    //          struct {
    //                  int loc, char *name;
    //          } entry;
    //      } details;
    //  }

    char lbuf[500];
    char **lines;
    size_t maxlines;
    unsigned nlines;
    
    int ret = 0;

    maxlines = 400;
    if ((lines = calloc(maxlines, sizeof(*lines))) == NULL) {
        int e = errno;
        fprintf(stderr, "listing_parse: malloc error.\n");
        errno = e;
        return -1; 
    }

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
        if (*lbuf == 0)
            continue;
        if (!source_finished) {
            if (str_pmatch(lbuf, "SOURCE FILES USED") == 0) {
                source_finished = 1;
                continue;
            }
            // Check for a source line
            if (strspn(lbuf, " \t0123456789") >= 9) {
                char c = lbuf[9];
                if (c >= '0' && c <= '9') {
                    fflush(stdout);
                    fprintf(stderr, "Line %u has unexpected 10th char after source number: %s\n", nlines + 1, lbuf);
                    errno = EINVAL;
                    return -1;
                }
                lbuf[9] = 0;
                int lineno;
                char dummy;
                if (sscanf(lbuf, "%d %c", &lineno, &c) == 1) {
                    lbuf[9] = c;
                    if (add_line(&lines, &maxlines, lineno, lbuf + 9) == -1) {
                        int e = errno;
                        fprintf(stderr, "listing_parse: error saving line.\n");
                        errno = e;
                        return -1;
                    }
                } else {
                    // probably an included line with multiple line numbers
                    lbuf[9] = c;
                }
            }
            continue;
        }
        if (!seen_ll_hdr) {
            // Look for header consisting of alternating "LINE" and "LOC"
            char *s = lbuf;
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
            char *s = lbuf;
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
                if (lineno <= 0 || lineno > maxlines) {
                    fflush(stdout);
                    fprintf(stderr, "Found reference to non-existant {line %d, loc %#o} at input line %d.\n", lineno, loc, nlines + 1);
                    errno = EINVAL;
                    return -1;
                }
                if ((*consumer)(lineno, loc, lines[lineno]) != 0)
                    return -1;
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
        int alm_lineno;
        if (!asm_seen_begin) {
            if (str_pmatch(lbuf, "BEGIN") == 0) {
                asm_seen_begin = 1;
                alm_lineno = -1;
                asm_consume_next = 0;
                continue;
            }
        } else {
            // Look for mixed source/assembly
            if (strspn(lbuf, "01234567") == 6 && (lbuf[6] == ' ' || lbuf[6] == '\t')) {
                unsigned loc;
                (void) sscanf(lbuf, "%o", &loc);
                if (asm_consume_next) {
                    if ((*consumer)(alm_lineno, loc, lines[alm_lineno]) != 0)
                        return -1;
                    asm_consume_next = 0;
                }
            } else {
                int stmt;
                char dummy;
                if (sscanf(lbuf, " STATEMENT %d ON LINE %d %c", &stmt, &alm_lineno, &dummy) == 2) {
                    if (alm_lineno <= 0 || alm_lineno > maxlines) {
                        fflush(stdout);
                        fprintf(stderr, "Found reference to non-existant {line %d} at input line %d.\n", alm_lineno, nlines + 1);
                        errno = EINVAL;
                        return -1;
                    }
                    asm_is_end = 0;
                    asm_consume_next = 1;
                } else {
                    const char *s = lbuf + strspn(lbuf, " \t");
                    if (!asm_is_end)
                        asm_is_end = strncmp(s, "end", 3) == 0  || strncmp(s, "END", 3) == 0;
                }
            }
        }
#if 1
        if (!seen_explicit_dcl) {
            seen_explicit_dcl = str_pmatch(lbuf, "NAMES DECLARED BY EXPLICIT CONTEXT") == 0;
        } else {
            if (strstr(lbuf, "NAMES DECLARED BY CONTEXT") != NULL) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (strstr(lbuf, "NAME DECLARED BY CONTEXT") != NULL) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (str_pmatch(lbuf, "STORAGE REQUIREMENTS FOR THIS PROGRAM") == 0) {
                seen_explicit_dcl = 0;
                continue;
            }
            if (strspn(lbuf, " \t\f") > 80)
                continue;   // continuation line listing ref locations
            const char *s = lbuf + strspn(lbuf, " \t");
            int l = strcspn(s, " \t");
            char edcl_name[sizeof(lbuf)];
            strncpy(edcl_name, s, l);
            edcl_name[l] = 0;
            s += l;
            s += strspn(s, " \t");
            if (strspn(s, "01234567") != 6) {
                    fprintf(stderr, "Explicit dcls: Found garbled entry at input line %d (no location): %s.\n", nlines, lbuf);
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
                // TODO: Use this
            }
        }
#endif
#if 1
        if (!seen_automatic) {
            seen_automatic = str_pmatch(lbuf, "STORAGE FOR AUTOMATIC VARIABLES") == 0;
        } else {
            if (str_pmatch(lbuf, "STACK FRAME") == 0)
                continue;
            if (str_pmatch(lbuf, "THE FOLLOWING") == 0) {
                seen_automatic = 0;
                continue;
            }
            char auto_block[sizeof(lbuf)];
            const char *s = lbuf + strspn(lbuf, " \t");
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

