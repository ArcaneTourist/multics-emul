/*
    seginfo.cpp -- Symbol tables and related functions for representing information about segments.
    Only used for debug output, not for actual instruction execution.
*/

#include "seginfo.h"
#include <string.h>
#include <iostream>
#include <list>
#include <map>
#include <vector>
using namespace std;

class source_line {
    int offset;         // segment offset of first of probably multiple instructions for the line
    int line_no;
    string text;
};

class stack_frame;

class entry_point {
public:
    entry_point() { offset = -1, last = -1; stack = NULL; };
    entry_point(const char* nm, int off, int lst = -1)
        { name = nm; offset = off; last = lst; stack = NULL; }
    string name;
    int offset;
    int last;               // negative if unknown
    stack_frame* stack;     // multiple entry points may share the same stack frame
};

class automatic {
    int offset;         // offset relative to stack pointer
    string name;
};

class stack_frame {
    entry_point *owner;
    int size;
    map <int, automatic> automatics;    // Key is automatic.offset
    // list of automatics
};

class asm_file {
public:
    asm_file(){};
    asm_file(const char* s, int o, int l){ fname = s; offset =o; len = l; };
    string fname;
    int offset;
    int len;
};

class pl1_file {
    string fname;
    map <int, source_line> lines;   // Key is source_line.offset
    map <int, entry_point> entries; // Key is entry_point.offset
    list <stack_frame> stack_frames;
};

// Problem: info from source listing will be unrelocated values and
// linkage tables will have relocated values

struct seginfo {
    map <int, entry_point> linkage; // Key is entry_point.offset
    list <pl1_file> pl1_files;
    map <int, asm_file> asm_files;  // Key is sm_file.offset
};

// Multics allows segment numbers 0..511.  We use 512 for holding info
// related to absolute offsets that don't have an associated segment.
const int max_seg = 511;
static vector <seginfo> segments;

// ============================================================================

void seginfo_dump(void)
{
    cerr << "seginfo_dump: Not implemented\n"; // BUG
}
// BUG: int seginfo_add_entry(int seg, int first, int last, const char* fname)
// BUG: find

int seginfo_add_asm(int seg, int first, int last, const char* fname)
{
    if (fname == NULL)
        return -1;
    if (seg > max_seg)
        return -1;
    if (seg == -1)
        seg = max_seg + 1;
    segments[seg].asm_files[first] = asm_file(fname, first, last - first + 1); 
    return 0;
}

int seginfo_add_linkage(int seg, int offset, int last, const char* name)
{
    if (name == NULL)
        return -1;
    if (seg > max_seg)
        return -1;
    if (seg == -1)
        seg = max_seg + 1;
    segments[seg].linkage[offset] = entry_point(name, offset, last);
    return 0;
}

