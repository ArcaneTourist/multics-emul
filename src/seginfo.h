#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *file_name;
    const char *entry;
    int entry_offset;
    int entry_hi;
    int line_no;
    const char *line;
} where_t;

void seginfo_dump(void);
int seginfo_add_linkage(int seg, int offset, const char* name);
int seginfo_add_source_file(int seg, int first, int last, const char* fname);
int seginfo_find_all(int seg, int offset, where_t *wherep);

#ifdef __cplusplus
}
#endif


// ============================================================================

#ifdef __cplusplus

#include <iostream>
#include <string.h>
#include <list>
#include <map>


class offset_t {
public:
    offset_t(int o) { offset = o; }
    offset_t() { offset = -1; }
    operator int() const { return offset; }
    int offset; // BUG: rename to val
    friend ostream& operator<<(ostream& out, const offset_t& o);
};

class seg_addr_t {
public:
    seg_addr_t(int s, int o) { segno = s; offset = o; }
    int segno;
    offset_t offset;
    friend ostream& operator<<(ostream& out, const seg_addr_t& sa);
};


class source_line {
public:
    source_line() { offset = -1; line_no = -1; }
    source_line(int loc, int line_num, const string& ltext)
        { offset = loc; line_no = line_num; text = ltext; }
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const source_line& sl)
        { return sl.print(out,0); }
    offset_t offset;            // segment offset of first of probably multiple instructions for the line
    int line_no;
    string text;
};

class stack_frame;

// Entry point information as loaded from a source listing.   Offsets are
// those reported by the compiler; note that in-core offsets may differ
// because of relocation by the binder.
class entry_point {
public:
    entry_point() { offset = -1, last = -1; stack = NULL; };
    entry_point(const string& nm, int off, int lst = -1)
        { name = nm; offset = off; last = lst; stack = NULL; }
    string name;
    offset_t offset;
    offset_t last;              // negative if unknown
    stack_frame* stack;     // multiple entry points may share the same stack frame
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const entry_point& ep)
        { return ep.print(out, 0); }
};

// Information found in in-core linkage sections of segments.
class linkage_info {
public:
    linkage_info() { offset = -1; entry = NULL; }
    linkage_info(const char *nm, int off)
        { name = nm; offset = off; entry = NULL; }
    string name;
    int offset;
    // int last;                // negative if unknown
    entry_point* entry;
    int hi() const
        { return (entry == NULL) ? -1 : int(entry->last); }
    ostream& print(ostream& out, int indent) const;
};

class automatic {
public:
    int offset;         // offset relative to stack pointer
    string name;
};

class stack_frame {
public:
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const stack_frame& sf)
        { return sf.print(out, 0); }
    entry_point *owner;
    int size;
    map <int, automatic> automatics;    // Key is automatic.offset
};

#if 0
class asm_file {
public:
    asm_file(){};
    asm_file(const char* s, int seg_no, int off_lo, int off_hi)
        { fname = s; segno = seg_no, lo = off_lo; hi = off_hi; };
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const asm_file& afile)
        { return afile.print(out, 0); }
    string fname;
    int segno;
    offset_ lo;
    offset_t hi;
};
#endif

class source_file {
public:
    source_file(const char* name) { fname = name; }
    string fname;
    offset_t lo;
    offset_t hi;
    map <int, source_line> lines;   // Key is source_line.offset
    map <int, entry_point> entries; // Key is entry_point.offset
    list <stack_frame> stack_frames;
    ostream& print(ostream& out, int indent) const;
#if 0
    friend ostream& operator<<(ostream& out, const source_file& pfile)
        { return pfile.print(out, 0); }
#endif
};


// Problem: info from source listing will be unrelocated values and
// linkage tables will have relocated values

class seginfo {
    public:
    map <int, linkage_info> linkage;        // Key is linkage_info.offset
    list <source_file> source_list;
    map <int, source_file*> source_map; // Key is source_file.lo; sources with negative lo offsets are not entered
    bool empty() const {
        return linkage.empty() && source_map.empty() && source_list.empty();
    }
};

// ============================================================================

// Multics allows segment numbers 0..511.  We use 512 for holding info
// related to absolute offsets that don't have an associated segment.
const int max_seg = 511;

source_file& seginfo_add_source_file(int segno, const char *fname, int offset = -1);

#endif //__cplusplus
