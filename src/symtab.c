/*
    symtab.c -- symbol table for address ranges
*/

#include <stdlib.h>
#include <stdio.h>

#include "sim_defs.h"
#include "symtab.h"

static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b);
static void print_entry(const t_symtab_ent *p);

#define MAX_SEGNO 511
static struct syms_s {
    int n_sym;
    int tblsize;
    t_symtab_ent *symtab;
} symbols[MAX_SEGNO + 2];


#if 0
void symtab_init()
{
    struct syms_s *syms;
    for (syms = symbols; syms < symbols + ARRAY_SIZE(symbols); ++syms) {
        if (syms->symtab != NULL)
            free(syms->symtab);
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
        symtab_parse(0, buf);
    }
}
#endif


int symtab_parse(int32 arg, char *buf)
{
    if (*buf == 0)
        symtab_dump();
    else if (strcmp(buf, "dump") == 0)
        symtab_dump();
    else {
        char *p = buf;
        char fname[1024];   // WARNING: buffer overflow possible
        int first, last;
        char dummy;
        int seg;
        if (sscanf(buf, "source %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            symtab_add_file(-1, first, last, fname);
        } else if (sscanf(buf, "source %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            symtab_add_file(seg, first, last, fname);
        } else if (strncmp(buf, "dump", 4) == 0 && sscanf(buf, "dump %c", &dummy) == 0) {
            symtab_dump();
        } else if (sscanf(buf, "find %i %c", &first, &dummy) == 1) {
            t_symtab_ent *p = symtab_find(-1, first, symtab_any);
            if (p == NULL)
                fprintf(stderr, "0%o not found.\n", first);
            else
                print_entry(p);
        } else if (sscanf(buf, "find %i|%i %c", &seg, &first, &dummy) == 2) {
            t_symtab_ent *p = symtab_find(seg, first, symtab_any);
            if (p == NULL)
                fprintf(stderr, "0%o not found.\n", first);
            else
                print_entry(p);
        } else
            fprintf(stderr, "symtab: cannot parse '%s'\n", buf);
    }
    return 0;
}


void symtab_dump()
{
    struct syms_s *syms;

    syms = &symbols[MAX_SEGNO + 1];
    if (syms->symtab != NULL && syms->n_sym != 0) {
        t_symtab_ent *p;
        for (p = syms->symtab; p < syms->symtab + syms->n_sym; ++p)
            print_entry(p);
    }
    for (syms = symbols; syms <= symbols + MAX_SEGNO; ++ syms) {
        if (syms->symtab == NULL || syms->n_sym == 0)
            continue;
        t_symtab_ent *p;
        for (p = syms->symtab; p < syms->symtab + syms->n_sym; ++p)
            print_entry(p);
    }
}


static t_symtab_ent *symtab_get(int seg, int first, int last)
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
    if (syms->symtab == NULL) {
        // create
        syms->n_sym = 0;
        syms->tblsize = 20;
        if ((syms->symtab = malloc(sizeof(*syms->symtab) * syms->tblsize)) == NULL)
            return NULL;
    }
    
    if (syms->n_sym == syms->tblsize) {
        // grow
        t_symtab_ent *new;
        if ((new = malloc(syms->tblsize * 2 * sizeof(*new))) == NULL)
            return NULL;
        memcpy(new, syms->symtab, sizeof(*new) * syms->tblsize);
        free(syms->symtab);
        syms->tblsize *= 2;
        syms->symtab = new;
    }

    t_symtab_ent key = { seg, first, last, 0, 0, 0, 0 };
    int i;
    for (i = 0; i < syms->n_sym; ++i)
        if (cmp_symtab(&key, syms->symtab+i) <= 0)
            break;
    if (syms->symtab[i].seg == seg && syms->symtab[i].addr_lo == first)
        return syms->symtab + i;

    if (syms->n_sym - i != 0)
        (void) memmove(syms->symtab+i+1, syms->symtab+i, (syms->n_sym - i) * sizeof(*syms->symtab));
    memset(syms->symtab + i, 0, sizeof(syms->symtab[i]));
    syms->symtab[i].seg = seg;
    syms->symtab[i].addr_lo = first;
    syms->symtab[i].addr_hi = last;

    ++ syms->n_sym;
    return syms->symtab + i;
}


int symtab_add_file (int seg, int first, int last, const char *fname)
{
    if (fname == NULL)
        return -1;
    t_symtab_ent *p = symtab_get(seg, first, last);
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
    t_symtab_ent *p = symtab_get(seg, loc, last);
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
    t_symtab_ent *p = symtab_get(seg, loc, last);
    if (p == NULL)
        return -1;

    p->types |= symtab_line;
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
    else {
        syms = symbols + seg;
    }

    if (syms->n_sym == 0)
        return NULL;
    int lo = 0;
    int hi = syms->n_sym;
if (msgs) out_msg("FIND: searching for {seg=%o, addr = %#o, type = %d}; range is 0..%d\n", seg, addr, type, hi);
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (syms->symtab[mid].addr_lo < addr)
            lo = mid + 1;
        else {
            hi = mid;
        }
    }
if(msgs) { if (lo == syms->n_sym) out_msg("FIND: Initial hit/fail at tail -- %d\n", lo);
else out_msg("FIND: Initial hit/fail at %d: {seg=%o, addr = %#o..%#o, types = %o}; range is 0..%d\n", lo, syms->symtab[lo].seg, syms->symtab[lo].addr_lo, syms->symtab[lo].addr_hi, syms->symtab[lo].types); }
    // ranged means we probably want the one prior to where we stopped
    if (lo == syms->n_sym)
        -- lo;
    while (lo > 0 && syms->symtab[lo-1].addr_lo == syms->symtab[lo].addr_lo)
        --lo;
    if (lo > 0)
        --lo;
#if 1
if(msgs)out_msg("FIND: Backup yields %d\n", lo);
#else
lo=0;
hi = syms->n_sym;
#endif

    // Scan for correct type and check if there's a addr_hi that matches better than a wildcard -1
    t_symtab_ent *found = NULL;
    while (lo < syms->n_sym) {
if(msgs)out_msg("FIND: Checking { lo=%#o, hi=%#o, type=%#o}\n", syms->symtab[lo].addr_lo, syms->symtab[lo].addr_hi, syms->symtab[lo].types);
        if (syms->symtab[lo].addr_lo > addr)
            break;
        if (syms->symtab[lo].addr_hi == -1) {
            if (type == symtab_any || (type & syms->symtab[lo].types) != 0)
                found = syms->symtab + lo;
        } else if (addr <= syms->symtab[lo].addr_hi)
            if (type == symtab_any || (type & syms->symtab[lo].types) != 0)
                // return syms->symtab + lo;
                found = syms->symtab + lo;
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
#if 0
        const char *name = (p->ename != NULL) p->ename : p->fname;
        printf("%o|%04o .. %o|%04o: %s\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->name);
#else
        if ((p->types & symtab_file) != 0)
            printf("Range %o|%04o .. %o|%04o, Source: %s.\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->fname);
        if ((p->types & symtab_proc) != 0)
            printf("Range %o|%04o .. %o|%04o, Entry: %s.\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->ename);
#endif
        if ((p->types & symtab_line) != 0)
            printf("Range %o|%04o .. %o|%04o, Line: %s\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->line);
    } else {
#if 0
        const char *name = (p->ename != NULL) p->ename : p->fname;
        printf("range 0%05o .. 0%05o: %s\n", p->addr_lo, p->addr_hi, p->name);
#else
        if ((p->types & symtab_file) != 0)
            printf("Range %04o .. %04o, Source: %s.\n", p->addr_lo, p->addr_hi, p->fname);
        if ((p->types & symtab_proc) != 0)
            printf("Range %04o .. %04o, Entry: %s.\n", p->addr_lo, p->addr_hi, p->fname);
#endif
        if ((p->types & symtab_line) != 0)
            printf("Range %04o .. %04o, Line: %s.\n", p->addr_lo, p->addr_hi, p->fname);
    }
}
