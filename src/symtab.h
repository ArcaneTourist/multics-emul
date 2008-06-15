enum symtab_type { symtab_none, symtab_file = 1 };

typedef struct {
    int addr_lo;
    int addr_hi;
    unsigned types; // bit mask, unused
    char *name;
} t_symtab_ent;

t_symtab_ent *symtab_find(int addr);
void symtab_dump();
t_stat symtab_parse(int32 arg, char *buf);
