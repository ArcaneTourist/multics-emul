typedef enum symtab_type { symtab_none = 0, symtab_file=1, symtab_proc=2, symtab_line=4  } t_symtab_type;

/*
    A t_symtab_ent represents a segment address or range of addresses.
    Those address(es) may have one or more of the following associated
    with it:
        file -- Source file that provides this code
        ename -- entry point name
        line_no -- line number within source file above
        line -- text of the source line
*/

typedef struct {
    int seg;
    int addr_lo;
    int addr_hi;
    unsigned types;
    int line_no;
    char *fname;
    char *ename;
    char *line;
} t_symtab_ent;

t_symtab_ent *symtab_find(int seg, int addr, int type);
void symtab_dump();
int cmd_symtab_parse(int32 arg, char *buf);

int symtab_add_file (int seg, int first, int last, const char *fname);
int symtab_add_entry(int seg, int loc, int last, const char *ename);
int symtab_add_line(int seg, int loc, int last, int line_no, const char *line);

int listing_parse(FILE *f, int(*consumer)(int lineno, unsigned loc, const char *line));
