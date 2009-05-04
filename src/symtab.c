/*
    symtab.c -- symbol table for address ranges
*/

/*
 * BUG: we need to use a symbol table per type, not combine them
 * Alternatively: we could have typed links running though the symbol tables
*/ 

#include <stdlib.h>
#include <stdio.h>

#include "sim_defs.h"
#include "symtab.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) ( sizeof(a) / sizeof((a)[0]) )
#endif

#define MAX_SEGNO 511
typedef struct symtab_s {
    int n;
    int size;
    t_symtab_ent *tbl;
} symtab_t;

static struct syms_s {
    symtab_t procs; // sources and procs
    symtab_t lines;
} symbols[MAX_SEGNO + 2];

static void symtab_dump_tbl(const symtab_t *tblp);
static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b);
static void print_entry(const t_symtab_ent *p);

#if 0
void symtab_init()
{
    struct syms_s *syms;
    for (syms = symbols; syms < symbols + ARRAY_SIZE(symbols); ++syms) {
        if (syms->procs.tbl != NULL)
            free(syms->procs.tbl);
        if (syms->lines.tbl != NULL)
            free(syms->lines.tbl);
    }
    memset(symbols, 0, sizeof(symbols));
}
#endif

#if 0
main()
{
    char buf[1024];
    while (gets(buf) != NULL) {
        // fprintf(stderr, "Read: %s.\n", buf);
        cmd_symtab_parse(0, buf);
    }
}
#endif


int cmd_symtab_parse(int32 arg, char *buf)
{
    if (*buf == 0)
        symtab_dump();
    else if (strcmp(buf, "dump") == 0)
        symtab_dump();
    else {
        char *p = buf;
        char fname[1024];   // BUG: WARNING: buffer overflow possible
        int first, last;
        char dummy;
        int seg;
        if (sscanf(buf, "source %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            symtab_add_file(-1, first, last, fname);
        } else if (sscanf(buf, "source %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            symtab_add_file(seg, first, last, fname);
        } else if (sscanf(buf, "entry %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            symtab_add_entry(-1, first, last, fname);
        } else if (sscanf(buf, "entry %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            symtab_add_entry(seg, first, last, fname);
        } else if (strncmp(buf, "dump", 4) == 0 && sscanf(buf, "dump %c", &dummy) == 0) {
            symtab_dump();
        } else if (sscanf(buf, "find %i %c", &first, &dummy) == 1) {
            int types[] = { symtab_file, symtab_proc, symtab_line };
            for (int i = 0; i < ARRAY_SIZE(types); ++i) {
                int type = types[i];
                t_symtab_ent *p = symtab_find(-1, first, type);
                if (p == NULL)
                    fprintf(stderr, "0%o not found.\n", first);
                else
                    print_entry(p);
            }
        } else if (sscanf(buf, "find %i|%i %c", &seg, &first, &dummy) == 2) {
            int types[] = { symtab_file, symtab_proc, symtab_line };
            for (int i = 0; i < sizeof(types)/sizeof(types[0]); ++i) {
                int type = types[i];
                t_symtab_ent *p = symtab_find(seg, first, type);
                if (p == NULL)
                    fprintf(stderr, "0%o not found.\n", first);
                else
                    print_entry(p);
            }
        } else
            fprintf(stderr, "procs: cannot parse '%s'\n", buf);
    }
    return 0;
}


void symtab_dump()
{
    struct syms_s *syms;

    syms = &symbols[MAX_SEGNO + 1];
    symtab_dump_tbl(&syms->procs);
    for (syms = symbols; syms <= symbols + MAX_SEGNO; ++ syms)
        symtab_dump_tbl(&syms->procs);

    syms = &symbols[MAX_SEGNO + 1];
    symtab_dump_tbl(&syms->lines);
    for (syms = symbols; syms <= symbols + MAX_SEGNO; ++ syms)
        symtab_dump_tbl(&syms->lines);
}


static void symtab_dump_tbl(const symtab_t *tblp)
{
    if (tblp == NULL)
        return;
    if (tblp->tbl == NULL || tblp->n == 0)
        return;

    t_symtab_ent *e;
    for (e = tblp->tbl; e < tblp->tbl + tblp->n; ++e)
        print_entry(e);
}


static t_symtab_ent *symtab_get(int seg, int first, int last, t_symtab_type type)
{
    struct syms_s *syms;
    if (seg < 0) {
        seg = -1;
        syms = &symbols[MAX_SEGNO + 1];
    } else if (seg > MAX_SEGNO)
        return NULL;
    else {
        syms = symbols + seg;
    }

    symtab_t *tblp;
    if (type == symtab_line)
        tblp = &syms->lines;
    else
        tblp = &syms->procs;

    if (tblp->tbl == NULL) {
        // create
        tblp->n = 0;
        tblp->size = 20;
        if ((tblp->tbl = malloc(sizeof(*tblp->tbl) * tblp->size)) == NULL)
            return NULL;
    }
    
    if (tblp->n == tblp->size) {
        // grow
        t_symtab_ent *new;
        if ((new = malloc(tblp->size * 2 * sizeof(*new))) == NULL)
            return NULL;
        memcpy(new, tblp->tbl, sizeof(*new) * tblp->size);
        free(tblp->tbl);
        tblp->size *= 2;
        tblp->tbl = new;
    }

    t_symtab_ent key = { seg, first, last, 0, 0, 0, 0 };
    int i;
    for (i = 0; i < tblp->n; ++i)
        if (cmp_symtab(&key, tblp->tbl+i) <= 0)
            break;
    if (tblp->tbl[i].seg == seg && tblp->tbl[i].addr_lo == first)
        return tblp->tbl + i;

    if (tblp->n - i != 0)
        (void) memmove(tblp->tbl+i+1, tblp->tbl+i, (tblp->n - i) * sizeof(*tblp->tbl));
    memset(tblp->tbl + i, 0, sizeof(tblp->tbl[i]));
    tblp->tbl[i].seg = seg;
    tblp->tbl[i].addr_lo = first;
    tblp->tbl[i].addr_hi = last;

    ++ tblp->n;
    return tblp->tbl + i;
}


int symtab_add_file (int seg, int first, int last, const char *fname)
{
    if (fname == NULL)
        return -1;
    t_symtab_ent *p = symtab_get(seg, first, last, symtab_file);
    if (p == NULL)
        return -1;

    p->types |= symtab_file;
    if ((p->fname = strdup(fname)) == NULL)
        return -1;
    return 0;
}

int symtab_add_entry(int seg, int loc, int last, const char *ename)
{
    if (ename == NULL)
        return -1;
    t_symtab_ent *p = symtab_get(seg, loc, last, symtab_proc);
    if (p == NULL)
        return -1;

    p->types |= symtab_proc;
    if ((p->ename = strdup(ename)) == NULL)
        return -1;
    return 0;
}

int symtab_add_line(int seg, int loc, int last, int line_no, const char *line)
{
    if (line == NULL)
        return -1;
    t_symtab_ent *p = symtab_get(seg, loc, last, symtab_line);
    if (p == NULL)
        return -1;

    p->types |= symtab_line;
    p->line_no = line_no;
    if ((p->line = strdup(line)) == NULL)
        return -1;
    return 0;
}

static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b)
{
    if (a == NULL)
        return -1;
    if (b == NULL)
        return 1;

    if (a->seg < b->seg)
        return -1;
    if (a->seg > b->seg)
        return 1;

    if (a->addr_lo < b->addr_lo)
        return -1;
    if (a->addr_lo > b->addr_lo)
        return 1;
    
    if (a->addr_hi < b->addr_hi)
        return -1;
    if (a->addr_hi > b->addr_hi)
        return 1;
    return 0;
}


t_symtab_ent *symtab_find(int seg, int addr, int type)
{
    // Find an entry matching the given segment and address.
    // Type is either symtab_any or a bitmask of which types to find

int msgs = 0;
if (addr >= 032425 && addr <= 032427) msgs = 1;
if (seg==031 && addr <= 7) msgs = 1;
if (seg==0400 && addr==06544) msgs = 1;
msgs=0;

    struct syms_s *syms;
    if (seg < 0) {
        seg = -1;
        syms = &symbols[MAX_SEGNO + 1];
    } else if (seg > MAX_SEGNO)
        return NULL;
    else
        syms = symbols + seg;

    symtab_t *tblp;
    if (type == symtab_line)
        tblp = &syms->lines;
    else
        tblp = &syms->procs;

    if (tblp->n == 0 || tblp->tbl == NULL)
        return NULL;

    int lo = 0;
    int hi = tblp->n;
if (msgs) out_msg("FIND: searching for {seg=%o, addr = %#o, type = %d}; range is 0..%d\n", seg, addr, type, hi);
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (tblp->tbl[mid].addr_lo < addr)
            lo = mid + 1;
        else {
            hi = mid;
        }
    }
if(msgs) { if (lo == tblp->n) out_msg("FIND: Initial hit/fail at tail -- %d\n", lo);
else out_msg("FIND: Initial hit/fail at %d: {seg=%o, addr = %#o..%#o, types = %o}; range is 0..%d\n", lo, tblp->tbl[lo].seg, tblp->tbl[lo].addr_lo, tblp->tbl[lo].addr_hi, tblp->tbl[lo].types); }
    // ranged means we probably want the one prior to where we stopped
    if (lo == tblp->n)
        -- lo;
    while (lo > 0 && tblp->tbl[lo-1].addr_lo == tblp->tbl[lo].addr_lo)
        --lo;
    if (lo > 0)
        --lo;
#if 1
if(msgs)out_msg("FIND: Backup yields %d\n", lo);
#else
lo=0;
hi = tblp->n;
#endif

    // Scan for correct type and check if there's a addr_hi that matches better than a wildcard -1
    t_symtab_ent *found = NULL;
    while (lo < tblp->n) {
if(msgs)out_msg("FIND: Checking { lo=%#o, hi=%#o, type=%#o}\n", tblp->tbl[lo].addr_lo, tblp->tbl[lo].addr_hi, tblp->tbl[lo].types);
        if (tblp->tbl[lo].addr_lo > addr)
            break;
        if (tblp->tbl[lo].addr_hi == -1) {
            if ((type & tblp->tbl[lo].types) != 0)
                found = tblp->tbl + lo;
        } else if (addr <= tblp->tbl[lo].addr_hi)
            if ((type & tblp->tbl[lo].types) != 0)
                // return tblp->tbl + lo;
                found = tblp->tbl + lo;
        ++ lo;
    }

if (msgs) {
if (found)
out_msg("FIND: Returning { lo=%#o, hi=%#o, type=%#o}\n", found->addr_lo, found->addr_hi, found->types);
else out_msg("FIND: Returning NULL\n"); }

    return found;
}

#if 0
static int match_word(char **bufpp, const char* word)
{
    char *bufp = *bufpp;
    bufp += strspn(bufp, " \t");
    int n = strlen(word);
    if (strncmp(bufp, word, n) == 0 && (bufp[n] == 0 || isspace(bufp[n])) {
        bufp += n;
        *bufpp = bufp;
        return 1;
    }
    return 0;
}

static char* extract_int(char **bufpp, int * intp)
{
    char *bufp = *bufpp;
    bufp += strspn(bufp, " \t");
    int n;
    if (sscanf("%i"
    int n = strspn(bufp, "0123456789"
    return 0;
}
#endif 

static void print_entry(const t_symtab_ent *p)
{
    if (!p)
        return;
    if (p->seg >= 0) {
        if ((p->types & symtab_file) != 0)
            printf("Range %o|%04o .. %o|%04o, Source: %s.\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->fname);
        if ((p->types & symtab_proc) != 0)
            printf("Range %o|%04o .. %o|%04o, Entry: %s.\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->ename);
        if ((p->types & symtab_line) != 0)
            printf("Range %o|%04o .. %o|%04o, Line: %d: %s\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->line_no, p->line);
    } else {
        if ((p->types & symtab_file) != 0)
            printf("Range %04o .. %04o, Source: %s.\n", p->addr_lo, p->addr_hi, p->fname);
        if ((p->types & symtab_proc) != 0)
            printf("Range %04o .. %04o, Entry: %s.\n", p->addr_lo, p->addr_hi, p->ename);
        if ((p->types & symtab_line) != 0)
            printf("Range %04o .. %04o, Line: %d: %s\n", p->addr_lo, p->addr_hi, p->line_no, p->line);
    }
}
