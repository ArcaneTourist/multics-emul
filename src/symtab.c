/*
    symtab.c -- symbol table for address ranges
*/

#include "hw6180.h"
#include <stdlib.h>
#include <stdio.h>

static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b);
static int symtab_add(int first, int last, enum symtab_type type, const char *fname);

static int n_sym = 0;
static int sym_tblsize = 0;
static t_symtab_ent *symtab = NULL;

#if 0
void symtab_init()
{
    if (symtab != NULL)
        free(symtab);
    symtab = NULL;
    n_sym = 0;
    sym_tblsize = 0;
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
    if (*buf == 0) {
        symtab_dump();
    } else {
        char *p = buf;
        char fname[1024];   // WARNING: buffer overflow possible
        int first, last;
        char dummy;
        if (sscanf(buf, "source %s %i %i %c", fname, &first, &last, &dummy) == 3) {
            symtab_add(first, last, symtab_file, fname);
        } else if (strncmp(buf, "dump", 4) == 0 && sscanf(buf, "dump %c", &dummy) == 0) {
            symtab_dump();
        } else if (sscanf(buf, "find %i %c", &first, &dummy) == 1) {
            t_symtab_ent *p = symtab_find(first);
            if (p == NULL)
                fprintf(stderr, "0%o not found.\n", first);
            else
                fprintf(stderr, "0%o: Range 0%05o..0%05o, %s.\n", first, p->addr_lo, p->addr_hi, p->name);
        } else
            fprintf(stderr, "symtab: cannot parse '%s'\n", buf);
    }
    return 0;
}

void symtab_dump()
{
    if (symtab == NULL || n_sym == 0)
        return;
    t_symtab_ent *p;
    for (p = symtab; p < symtab + n_sym; ++p)
        printf("range 0%05o .. 0%05o: %s\n", p->addr_lo, p->addr_hi, p->name);
}


static int symtab_add(int first, int last, enum symtab_type type, const char *fname)
{
    if (symtab == NULL) {
        // create
        n_sym = 0;
        sym_tblsize = 40;
        if ((symtab = malloc(sizeof(*symtab) * sym_tblsize)) == NULL)
            return 1;
    }
    
    if (n_sym == sym_tblsize) {
        // grow
        t_symtab_ent *new;
        if ((new = malloc(sym_tblsize * 2 * sizeof(*new))) == NULL)
            return 1;
        memcpy(new, symtab, sizeof(*symtab));
        free(symtab);
        sym_tblsize *= 2;
        symtab = new;
    }

    t_symtab_ent key = { first, last, 0, 0 };
    int i;
    for (i = 0; i < n_sym; ++i)
        if (cmp_symtab(&key, symtab+i) <= 0)
            break;
    if (n_sym - i != 0)
        (void) memmove(symtab+i+1, symtab+i, (n_sym - i) * sizeof(*symtab));
    symtab[i].addr_lo = first;
    symtab[i].addr_hi = last;
    symtab[i].types |= type;
    if ((symtab[i].name = strdup(fname)) == NULL)
        return 1;
    ++ n_sym;
}

static int cmp_symtab(const t_symtab_ent *a, const t_symtab_ent *b)
{
    if (a == NULL)
        return -1;
    if (b == NULL)
        return 1;

    if (a->addr_lo < b->addr_lo)
        return -1;
    if (a->addr_lo > b->addr_lo)
        return 1;
    
    if (a->addr_hi < b->addr_hi)
        return -1;
    if (a->addr_hi == b->addr_hi)
        return 0;
    return 1;
}


t_symtab_ent *symtab_find(int addr)
{
    if (n_sym == 0)
        return NULL;
    int lo = 0;
    int hi = n_sym;
    while (lo < hi) {
        int mid = (lo + hi) / 2;
        if (symtab[mid].addr_lo < addr)
            lo = mid + 1;
        else {
            hi = mid;
        }
    }
    // ranged means we probably want the one prior to where we stopped
    if (lo > 0)
        --lo;
    if (symtab[lo].addr_lo <= addr && addr <= symtab[lo].addr_hi)
        return symtab + lo;
    ++lo;
    if (lo < n_sym && symtab[lo].addr_lo <= addr && addr <= symtab[lo].addr_hi)
        return symtab + lo;
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
