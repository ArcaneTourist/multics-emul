/*
    symtab.c -- symbol table for address ranges
*/

#include "hw6180.h"
#include <stdlib.h>
#include <stdio.h>

static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b);

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


t_stat symtab_parse(int32 arg, char *buf)
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
            symtab_add(-1, first, last, symtab_file, fname);
        } else if (sscanf(buf, "source %s %i|%i %i %c", fname, &seg, &first, &last, &dummy) == 4) {
            symtab_add(seg, first, last, symtab_file, fname);
        } else if (strncmp(buf, "dump", 4) == 0 && sscanf(buf, "dump %c", &dummy) == 0) {
            symtab_dump();
        } else if (sscanf(buf, "find %i %c", &first, &dummy) == 1) {
            t_symtab_ent *p = symtab_find(-1, first);
            if (p == NULL)
                fprintf(stderr, "0%o not found.\n", first);
            else
                fprintf(stderr, "0%o: Range 0%05o..0%05o, %s.\n", first, p->addr_lo, p->addr_hi, p->name);
        } else if (sscanf(buf, "find %i|%i %c", &seg, &first, &dummy) == 2) {
            t_symtab_ent *p = symtab_find(seg, first);
            if (p == NULL)
                fprintf(stderr, "0%o not found.\n", first);
            else
                fprintf(stderr, "0%o: Range %o|%04o .. %o|%04o, %s.\n", first, seg, p->addr_lo, seg, p->addr_hi, p->name);
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
            printf("range 0%05o .. 0%05o: %s\n", p->addr_lo, p->addr_hi, p->name);
    }
    for (syms = symbols; syms <= symbols + MAX_SEGNO; ++ syms) {
        if (syms->symtab == NULL || syms->n_sym == 0)
            continue;
        t_symtab_ent *p;
        for (p = syms->symtab; p < syms->symtab + syms->n_sym; ++p)
            printf("%o|%04o .. %o|%04o: %s\n", p->seg, p->addr_lo, p->seg, p->addr_hi, p->name);
    }
}


int symtab_add(int seg, int first, int last, enum symtab_type type, const char *fname)
{
    struct syms_s *syms;
    if (seg < 0) {
        seg = -1;
        syms = &symbols[MAX_SEGNO + 1];
    } else if (seg > MAX_SEGNO)
        return -1;
    else {
        syms = symbols + seg;
    }
    if (syms->symtab == NULL) {
        // create
        syms->n_sym = 0;
        syms->tblsize = 20;
        if ((syms->symtab = malloc(sizeof(*syms->symtab) * syms->tblsize)) == NULL)
            return 1;
    }
    
    if (syms->n_sym == syms->tblsize) {
        // grow
        t_symtab_ent *new;
        if ((new = malloc(syms->tblsize * 2 * sizeof(*new))) == NULL)
            return 1;
        memcpy(new, syms->symtab, sizeof(*new) * syms->tblsize);
        free(syms->symtab);
        syms->tblsize *= 2;
        syms->symtab = new;
    }

    t_symtab_ent key = { seg, first, last, 0, 0 };
    int i;
    for (i = 0; i < syms->n_sym; ++i)
        if (cmp_symtab(&key, syms->symtab+i) <= 0)
            break;
    if (syms->symtab[i].seg == seg && syms->symtab[i].addr_lo == first && strcmp(syms->symtab[i].name, fname) == 0) {
        // duplicate
        // out_msg("SYM: DEBUG: ignoring duplicate {seg=%#o,first=%#o,last,%s}\n", seg, first, fname);
        return 0;
    }
    // out_msg("SYM: DEBUG: adding {seg=%#o,first=%#o,last,%s}\n", seg, first, fname);
    if (syms->n_sym - i != 0)
        (void) memmove(syms->symtab+i+1, syms->symtab+i, (syms->n_sym - i) * sizeof(*syms->symtab));
    syms->symtab[i].seg = seg;  // redundant...
    syms->symtab[i].addr_lo = first;
    syms->symtab[i].addr_hi = last;
    syms->symtab[i].types |= type;
    if ((syms->symtab[i].name = strdup(fname)) == NULL)
        return 1;
    ++ syms->n_sym;
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


t_symtab_ent *symtab_find(int seg, int addr)
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

    if (syms->n_sym == 0)
        return NULL;
    int lo = 0;
    int hi = syms->n_sym;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (syms->symtab[mid].addr_lo < addr)
            lo = mid + 1;
        else {
            hi = mid;
        }
    }
    // ranged means we probably want the one prior to where we stopped
    if (lo > 0)
        --lo;
    if (syms->symtab[lo].addr_lo <= addr && (addr <= syms->symtab[lo].addr_hi || syms->symtab[lo].addr_hi == -1))
        return syms->symtab + lo;
    ++lo;
    if (lo < syms->n_sym && syms->symtab[lo].addr_lo <= addr && (addr <= syms->symtab[lo].addr_hi || syms->symtab[lo].addr_hi == -1))
        return syms->symtab + lo;
    return 0;
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
