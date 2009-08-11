/*
    listing.c -- parse PL1 compiler listings for location info that the
    simulator can use for displaying the source lines associated with
    the assembly being executed or displayed.
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


int listing_parse(FILE *f, int(*consumer)(int lineno, unsigned loc, const char *line))
{
    char lbuf[500];
    char **lines;
    size_t maxlines;
    unsigned nlines;
    
    maxlines = 400;
    if ((lines = calloc(maxlines, sizeof(*lines))) == NULL) {
        int e = errno;
        fprintf(stderr, "listing_parse: malloc error.\n");
        errno = e;
        return -1; 
    }

    int seen_ll_hdr = 0;    // Seen the "LINE LOC LINE LOC ..." header ?
    int seen_line_locs = 0;     // Seen all of the actual line locations that follow the ll header
    int seen_begin = 0;         // Seen the ^"BEGIN" that starts the mixed source/assembly listings
    int source_finished = 0;    // Seen something that says no more source (just statics and info)
    int consume_next = 0;       // State flag for mixed source/assembly
    int is_end = 0;             // State flag for mixed source/assembly
    // very portable, but not as pretty as perl
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
        if (!source_finished) {
            char *s = lbuf;
            s += strspn(s, " \t\f");
            const char *str = "SOURCE FILES USED";
            if (strncmp(s, str, strlen(str)) == 0) {
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
        if (!seen_begin) {
            if (strncmp(lbuf, "BEGIN", strlen("BEGIN")) == 0) {
                seen_begin = 1;
                alm_lineno = -1;
                consume_next = 0;
                continue;
            }
        } else {
            // Look for mixed source/assembly
            if (strspn(lbuf, "01234567") == 6 && (lbuf[6] == ' ' || lbuf[6] == '\t')) {
                unsigned loc;
                (void) sscanf(lbuf, "%o", &loc);
                if (consume_next) {
                    if ((*consumer)(alm_lineno, loc, lines[alm_lineno]) != 0)
                        return -1;
                    consume_next = 0;
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
                    is_end = 0;
                    consume_next = 1;
                } else {
                    const char *s = lbuf + strspn(lbuf, " \t");
                    if (!is_end)
                        is_end = strncmp(s, "end", 3) == 0  || strncmp(s, "END", 3) == 0;
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
    if (consume_next && ! is_end) {
        fflush(stdout);
        fprintf(stderr, "Found dangling source line without corresponding assembly at EOF.\n");
        errno = EINVAL;
        return -1;
    }
    return 0;
}
