/*
 * Support for symbolic debugging -- information about segments
*/
/*
   Copyright (c) 2007-2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#ifndef _SEGINFO_H
#define _SEGINFO_H

#ifdef __cplusplus
extern "C" {
#endif

#if !defined(SIM_DEFS_H_)
// #include <stdint.h>
// typedef uint64_t t_uint64;
// SIMH insists on unsigned long long instead of uint64_t, so we should
// just pull in their defs for consistancy.
#include "sim_defs.h"
#endif

#ifndef _HW6180_H
// FIXME -- should probably require  h6180.h to be included...
// typedef enum { ABSOLUTE_mode, APPEND_mode, BAR_mode } addr_modes_t;
#endif

typedef struct {
    int offset;         // offset relative to stack pointer
    const char *name;
} automatic_t;
typedef struct {
    const char *file_name;
    const char *entry;
    int entry_offset;
    int entry_hi;
    int line_no;
    const char *line;
    int n_auto;
} where_t;

void seginfo_dump(void);
int seginfo_add_linkage(int seg, int offset, const char* name);
int seginfo_add_source_file(int seg, int first, int last, const char* fname);
int seginfo_find_all(int seg, int offset, where_t *wherep);
int seginfo_automatic_count(int segno, int offset);
int seginfo_automatic_list(int segno, int offset, int *count, automatic_t *list);
void seginfo_find_line(int segno, int offset, const char**line, int *lineno);

extern int fetch_acc(int (*fetch)(unsigned addr, t_uint64 *wordp), unsigned addr, char bufp[513]);

#ifdef __cplusplus
}
#endif


// ============================================================================

#ifdef __cplusplus

#include <iostream>
#include <string.h>
#include <vector>
#include <list>
#include <map>

// The seg_offset_t and seg_addr_t classes exist mainly to provide
// formatted output support.

class seg_offset_t {
public:
    seg_offset_t(int o) { offset = o; }
    seg_offset_t() { offset = -1; }
    operator int() const { return offset; }
    int offset; // FIXME: rename to val
    friend ostream& operator<<(ostream& out, const seg_offset_t& o);
};

class seg_addr_t {
public:
    seg_addr_t(int s, int o) { segno = s; offset = o; }
    int segno;
    seg_offset_t offset;
    int operator == (const seg_addr_t& x) const
        { return x.segno == segno && x.offset == offset; }
    int operator != (const seg_addr_t& x) const
        { return ! operator==(x); }
    friend ostream& operator<<(ostream& out, const seg_addr_t& sa);
    operator string(void) const;
};

#if 0
class addr_t {
    addr_modes_t _mode;
    unsigned _segno;
    seg_offset_t _offset;
public:
    addr_modes_t mode() { return _mode; }
    addr_t(int offset) { _mode = ABSOLUTE_mode; _offset = offset; }
    addr_t(unsigned segno, int offset) { _mode = APPEND_mode; _segno = segno; _offset = offset; }
    const seg_offset_t& offset() const { return _offset; }
    seg_offset_t& offset() { return _offset; }
    int segno() const { return (_mode == APPEND_mode) ? _segno : -1; }
};
#endif

class source_line {
    // Offsets are the un-relocated values given in the compiler listing
public:
    source_line() { offset = -1; line_no = -1; }
    source_line(int loc, int line_num, const string& ltext)
        { offset = loc; line_no = line_num; text = ltext; }
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const source_line& sl)
        { return sl.print(out,0); }
    seg_offset_t offset;            // segment offset of first of probably multiple instructions for the line
    int line_no;
    string text;
};


class entry_point;
class source_file;

class var_info {
public:
    enum vartype { unknown, fixedbin, bit, ptr, str, vchar };

    var_info() { type = unknown; size = 0; size2 = 0; };
    var_info(const char *nm, enum vartype v, unsigned sz, int sz2 = 0)
        { name = nm; type = v; size = sz; size2 = sz2; };

    enum vartype type;
    string name;
    unsigned size;
    int size2;
};

class stack_frame {
public:
    stack_frame() { owner = NULL; size = -1; }
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const stack_frame& sf)
        { return sf.print(out, 0); }
    entry_point *owner;
    int size;
    map <int, var_info> automatics; // Key is stack offset
    // map <int, string> automatics;    // Key is stack offset
};


// Entry point information as loaded from a source listing.   Offsets are
// those reported by the compiler; note that in-core offsets may differ
// because of relocation by the binder.
class entry_point {
public:
    entry_point() { offset = -1, last = -1; _stack = NULL; stack_owner = NULL; source = NULL; is_proc = 0;};
    entry_point(const string& nm, int off, int lst = -1)
        { name = nm; offset = off; last = lst; _stack  = NULL; stack_owner = NULL; source = NULL; is_proc = 0;}
    string name;
    seg_offset_t offset;            // Un-relocated offset reported by the compiler
    seg_offset_t last;              // negative if unknown
    stack_frame* stack() const { return _stack ? _stack : stack_owner ? stack_owner ->_stack : NULL; }
    stack_frame* _stack;
    const entry_point* stack_owner;
    const source_file* source;
    int is_proc;
    ostream& print(ostream& out, int indent) const;
    friend ostream& operator<<(ostream& out, const entry_point& ep)
        { return ep.print(out, 0); }
};

// Information found in either in-core linkage sections of segments
// or in listings files.
class linkage_info {
public:
    linkage_info() { offset = -1; entry = NULL; }
    linkage_info(const char *nm, int off)
        { name = nm; offset = off; entry = NULL; }
    string name;                // DOC FIXME: is name same as entry->name?  Is entry allowed to be NULL?
    int offset;                 // May or may not be relocated value, see owner
    entry_point* entry;
    int hi() const // unrelocated
        { return (entry == NULL) ? -1 : int(entry->last); }
    ostream& print(ostream& out, int indent) const;
};


#if 0
class automatic {
public:
    automatic() { offset = -1; value = 0; }
    automatic(string& nm, int off) { name = nm; offset = off; value = 0; }
    automatic(const char* nm, int off) { name = nm; offset = off; value = 0; }
    int offset;         // offset relative to stack pointer
    string name;
    t_uint64 value;
};
#endif

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
    seg_offset_t hi;
};
#endif

class source_file {
    // All offsets for source files are the unrelocated values given in the compiler listing
public:
    source_file(const char* name) { fname = name; reloc = -1; }
    string fname;
    string seg_name;
    seg_offset_t reloc;     // Compiled segment may be relocated by binder
    // Assembly sources are tracked only by file name and a range of offsets
    seg_offset_t _lo;
    seg_offset_t _hi;
    seg_offset_t lo() const { return (reloc < 0) ? -1 : _lo + reloc; }
    seg_offset_t hi() const { return (reloc < 0) ? -1 : _hi < 0 ? -1 : _hi + reloc; }
    map <int, source_line> lines;   // Key is source_line.offset
    map <int, entry_point> entries; // Key is entry_point.offset
    map <string,stack_frame> stack_frames;  // Key is stack frame name (which should match some entry_point)
    ostream& print(ostream& out, int indent) const;
#if 0
    friend ostream& operator<<(ostream& out, const source_file& pfile)
        { return pfile.print(out, 0); }
#endif
    entry_point* find_entry(const string& s);
    const source_line *get_line(int offset, int use_relocation = 1) const;
//private:
    map <string, entry_point*> entries_by_name; // Key is entry_point.name, pointers point into "entries" map
private:
    map<int,source_line>::const_iterator find_line(int offset, int use_relocation = 1) const;
};


class seginfo {
private:
    multimap <int, linkage_info> linkage;   // Key is linkage_info.offset (a relocated runtime value)
public:
    list <source_file> source_list;
    map <int, source_file*> source_map; // Key is source_file.lo(); sources with negative lo offsets are not entered
    bool empty() const {
        return linkage.empty() && source_map.empty() && source_list.empty();
    }
    map<int,linkage_info>::iterator linkage_insert(pair<int,linkage_info> p)
        { return linkage.insert(p); }
    map<int,linkage_info>::const_iterator linkage_end() const
        { return linkage.end(); };
    map<int,linkage_info>::const_iterator find_linkage(int offset) const
        { return linkage.find(offset); }
    map<int,linkage_info>::const_iterator find_entry(int offset) const;
        // search linkage for relocated entry_point corresponding to
        // offset
    ostream& print(ostream& out, int indent = 0) const;
};

// A class for representing details known to be associated with either
// absolute memory locations or one of the 512 segments.   An index of
// -1 represents absolute mode addressing; non-negative indices represent
// appending aka segmented mode addressing.
class segments_t {
    vector<seginfo> segments;
public:
    static const int min_segno = -1;
    static const int max_segno = 511;
    seginfo& operator()(int n) { return segments[n + 1]; }
    const seginfo& operator()(int n) const { return segments[n + 1]; }
    segments_t() { segments.resize(max_segno - min_segno + 1); }
};

extern segments_t segments;

// ============================================================================

// SIMH munges UNIX output and expects a carriage return for every line feed

static inline std::ostream& simh_nl(std::ostream &os) { return os << "\r\n"; }

static inline std::ostream& simh_endl(std::ostream &os) { return os << simh_nl << flush; }

// ============================================================================

extern source_file& seginfo_add_source_file(int segno, const char *fname, int offset = -1);

// ============================================================================

typedef enum { def_undef = -1, def_text = 0, def_symbol = 2, def_seg_name = 3} def_type;

// An entry in a segment's definitions table
class def_t {
private:
    string _name;
public:
    // TODO: make offset private & provide a function that returns
    // either _offset or link.offset as appropriate
    def_type type;
    linkage_info link;  // for type == def_text
    unsigned offset;    // for type == def_symbol or unknown
    const string& name() const
        { if (type == def_text) return link.name; else return _name; }
    def_t (linkage_info li)
        { type = def_text; offset = ~0; link = li; }
    def_t (const char* seg_name)
        { type = def_seg_name; offset = ~0; _name = seg_name; }
    def_t (unsigned off, const char* sym)
        { type = def_symbol; offset = off; _name = sym; }
    def_t (int def_class, unsigned off, const char* text)
        { type = (def_type) def_class; offset = off; _name = text; }
    def_t ()
        { type = def_undef; offset = ~ 0; }
};

// A segment's entire definitions table
class seg_defs {
private:
    // unsigned _hdrp;
    unsigned _first_defp;
    unsigned _firstp;
    //vector<string> _names;
    //map <int, linkage_info> _defs;  // Key is offset
    // void add(unsigned defp, unsigned offset, const char* s);
    void add_link(unsigned defp, linkage_info li)
        { defs[defp] = def_t(li); }
    void add_seg_name(unsigned defp, const char* seg_name)
        { defs[defp] = def_t(seg_name); }
    void add_sym(unsigned defp, unsigned offset, const char* sym)
        { defs[defp] = def_t(offset, sym); }
    void add_unknown(unsigned defp, int def_class, unsigned offset, const char* text)
        { defs[defp] = def_t(def_class, offset, text); }
public:
    unsigned defp() { return _first_defp; }
    unsigned firstp() { return _firstp; }
    int scan(uint hdrp, int (*fetch)(uint addr, t_uint64 *wordp));
    map <int, def_t> defs;  // Key is defp offset
        // FIXME: make private & provide iterators
    //FIXME: provide efficient lookups:
        // const string name(); 
        // const string* names();
        // begin() & end() ordered by offset
};

// ============================================================================

#endif //__cplusplus
#endif // _SEGINFO_H
