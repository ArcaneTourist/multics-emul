enum symtab_type { symtab_none, symtab_file, symtab_proc };

typedef struct {
    int seg;
    int addr_lo;
    int addr_hi;
    unsigned types; // bit mask, unused
    char *name;
} t_symtab_ent;

t_symtab_ent *symtab_find(int seg, int addr);
void symtab_dump();
t_stat symtab_parse(int32 arg, char *buf);
int symtab_add(int seg, int first, int last, enum symtab_type type, const char *name);
