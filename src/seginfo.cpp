/*
    seginfo.cpp -- Symbol tables and related functions for representing information about segments.
    Only used for debug output, not for actual instruction execution.
*/

/*
    BUG: Need to implement algorithm for searching through an ordered list of (possibly nested) ranges.
    Currently, some ranges may have unknown ending point.

    BUG: relocation
        Scanning config_.list shows find: at offset 314; seg scanning shows find$find at 401.
        Difference is 65.  Hand estimate was 60.
*/

using namespace std;
#include "seginfo.h"
#include <iostream>
#include <iomanip>
// #include <string.h>
// #include <list>
// #include <map>
// #include <vector>

// ============================================================================

static seginfo segments[max_seg + 2];
static const int max_alm_per_line = 10;     // Used for estimating the location of the last machine instruction for the last source line of a file

// ============================================================================

// SIMH munges UNIX output and expects a carriage return for every line feed

static inline std::ostream& simh_nl(std::ostream &os )
{
    return os << "\r\n";
}

static inline std::ostream& simh_endl(std::ostream &os )
{
    return os << simh_nl << flush;
}

// ============================================================================

class range_t {
public:
    range_t(int seg, int off_lo, int off_hi)
        { segno = seg; lo = off_lo; hi = off_hi; }
private:
    int segno;
    offset_t lo, hi;
friend ostream& operator<<(ostream& out, const range_t& r);
};

// ============================================================================

ostream& operator<<(ostream& out, const offset_t& o)
{
    if (o.offset >= 0)
        out << setw(6) << setfill('0');
    out << oct << o.offset;

    return out;
}

// ============================================================================

ostream& operator<<(ostream& out, const seg_addr_t& sa)
{
    if (sa.segno != -1)
        out << oct << setw(3) << setfill('0') << sa.segno << "|";
    out << sa.offset;

    return out;
}

// ============================================================================

ostream& operator<<(ostream& out, const range_t& r)
{
    // Output an address range.
    // If seg is negative, outputs:     <offset> .. <offset>
    // If seg is non negative, outputs: <seg>|<offset> .. <seg>|<offset>

    if (r.segno < 0) {
        out << r.lo << " .. " << r.hi;
    } else {
        out << seg_addr_t(r.segno, r.lo);
        out << " .. ";
        out << seg_addr_t(r.segno, r.hi);
    }

    return out;
}

// ============================================================================

ostream& linkage_info::print(ostream& out, int indent) const
{
    out << string(indent, ' ');
    out << "Offset " << offset_t(offset) << ": entry point " << name << simh_nl;

    return out;
}


// ============================================================================

ostream& source_line::print(ostream& out, int indent) const
{
    out << string(indent, ' ');
    out << "Line " << dec << line_no << " at " << offset <<  ": " << text << simh_nl;

    return out;
}

// ============================================================================

ostream& entry_point::print(ostream& out, int indent) const
{
    out << string(indent, ' ');
    out << "Entry point " << name << " at offset " << offset;
    if (last > 0)
        out << " .. " << last;
    if (stack) {
        if (stack->owner == this)
            out << " with own stack frame";
        else if (stack->owner)
            if (stack->owner->name == name)
                out << " using own stack frame";
            else
                out << " using stack frame of " << stack->owner->name;
        // else no known owner...
    }
    out << simh_nl;

    return out;
}

// ============================================================================

ostream& stack_frame::print(ostream& out, int indent) const
{
    out << string(indent, ' ');
    out << "Stack frame " << ((owner) ? owner->name : "unknown") << ":" << simh_nl;
    indent += 2;
    out << string(indent, ' ') << "Stack size: " << dec << size << simh_nl;
    if (automatics.empty())
        out << string(indent, ' ') << "No Automatics" << simh_nl;
    else {
        out << string(indent, ' ') << "Automatics:" << simh_nl;
        indent += 2;
        for (map<int,automatic>::const_iterator it = automatics.begin(); it != automatics.end(); it++) {
            const automatic& a = (*it).second;
            out << string(indent, ' ') << offset_t(a.offset) << " " << a.name << simh_nl;
        }
    }

    return out;
}

// ============================================================================

ostream& source_file::print(ostream& out, int indent) const
{
    out << string(indent, ' ');

    if (lo() < 0)
        out << "File " << fname;
    else {
        if (reloc < 0)
            out << "Tentative ";
        if (hi() >= 0)
            out << "Range  " << range_t(-1, lo(), hi());
        else
            out << "Offset " << lo();
        out << ":  " << fname << " ";
        if (reloc < 0)
            out << "(Relocation information unavailable)";
        else if (reloc > 0)
            out << "(Relocated by " << reloc << ")";
        else
            out << "(non-relocated)" ;
    }
    if (entries.empty() && stack_frames.empty() && lines.empty()) {
        out << " -- No entry points, stack frames, or lines." << simh_nl;
        return out;
    }
    out << simh_nl;

    out << simh_nl;
    indent += 2;

    if (entries.empty())
        out << string(indent, ' ') << "No entry points." << simh_nl;
    else {
        out << string(indent, ' ') << "Entry Points:" << simh_nl;
        for (map<int,entry_point>::const_iterator it = entries.begin(); it != entries.end(); it++) {
            const entry_point& ep = (*it).second;
            ep.print(out, indent + 2);
        }
    }

    if (stack_frames.empty())
        out << string(indent, ' ') << "No stack frames." << simh_nl;
    else {
        out << string(indent, ' ') << "Stack Frames:" << simh_nl;
        for (map<string,stack_frame>::const_iterator it = stack_frames.begin(); it != stack_frames.end(); it++) {
            const stack_frame& sl = (*it).second;
            sl.print(out, indent + 2);
        }
    }
    
    
    if (lines.empty())
        out << string(indent, ' ') << "No lines." << simh_nl;
    else {
        out << string(indent, ' ') << "Lines:" << simh_nl;
        for (map<int,source_line>::const_iterator it = lines.begin(); it != lines.end(); it++) {
            const source_line& sl = (*it).second;
            sl.print(out, indent + 2);
        }
    }
    
    return out;
}


// ============================================================================
// ============================================================================
// ============================================================================
//
entry_point* source_file::find_entry(const string& s)
{
    map<string,entry_point*>::iterator it = entries_by_name.find(s);
    if (it == entries_by_name.end())
        return NULL;
    else
        return (*it).second;
}

template<typename Map> typename Map::const_iterator 
find_lower(Map const& m, typename Map::key_type const& k)
{
    typename Map::const_iterator it = m.upper_bound(k);
    if (it == m.begin())
        return m.end();
    return --it;
}


template<typename Map> typename Map::iterator 
find_lower(Map & m, typename Map::key_type const& k)
{
    typename Map::iterator it = m.upper_bound(k);
    if (it == m.begin())
        return m.end();
    return --it;
}

// ============================================================================

source_file& seginfo_add_source_file(int segno, const char *fname, int offset)
{
    int segidx = (segno == -1) ? max_seg + 1 : segno;
    segments[segidx].source_list.push_back(source_file(fname));
    source_file& src = segments[segidx].source_list.back();
    if (offset < 0)
        src._lo = -1;
    else {
        src._lo = 0;
        src._hi = -1;
        src.reloc = offset;
        if (segments[segidx].source_map[src.lo()] != NULL)
            cerr << "internal error: " << oct << segno << "|" << src.lo() << " already has a source file listed." << simh_nl;
        else
            segments[segidx].source_map[src.lo()] = &src;
    }
    return src;
}

// ============================================================================

int seginfo_add_source_file(int segno, int first, int last, const char* fname)
{
    source_file& src = seginfo_add_source_file(segno, fname, first);
    src._hi = last;
    src.reloc = 0;
    return 0;
}

// ============================================================================

void seginfo_dump(void)
{
    for (int segno = -1; segno <= max_seg; ++ segno) {
        int segidx = (segno == -1) ? max_seg + 1 : segno;
        seginfo& seg = segments[segidx];
        if (seg.empty())
            continue;
        if (segno == -1)
            cout << "Absolute Memory:" << simh_nl;
        else
            cout << "Segment " << oct << segno << ":" << simh_nl;
        if (! seg.source_map.empty()) {
            cout << "  Sources (with re-location info):" << simh_nl;
            for (map<int,source_file*>::iterator it = seg.source_map.begin(); it != seg.source_map.end(); it++) {
                source_file* src = (*it).second;
                src->print(cout, 4);
            }
        }
        if (seg.linkage.empty())
            cout << "  No Linkage." << simh_nl;
        else {
            cout << "  Linkage:" << simh_nl;
            for (map<int,linkage_info>::iterator it = seg.linkage.begin(); it != seg.linkage.end(); it++) {
                linkage_info li= (*it).second;
                li.print(cout, 4);
            }
        }
        if (seg.source_list.empty()) 
            cout << "  No sources that don't have location info." << simh_nl;
        else {
            bool hdr = 0;
            for (list<source_file>::iterator it = seg.source_list.begin(); it != seg.source_list.end(); it++) {
                source_file& src = *it;
                if (src.reloc < 0) {
                    // not displayed above
                    if (!hdr) {
                        hdr = 1;;
                        cout << "  Sources (without re-location info):" << simh_nl ;
                    }
                    src.print(cout, 4);
                }
            }
            if (!hdr)
                cout << "  All sources have re-location info." << simh_nl;
        }
    }
}

// ============================================================================


int seginfo_add_linkage(int segno, int offset, const char* name)
{
    if (name == NULL)
        return -1;
    if (segno > max_seg)
        return -1;
    int segidx = (segno == -1) ? max_seg + 1 : segno;

    seginfo& seg = segments[segidx];
    int ret = 0;
    if (seg.linkage.find(offset) != seg.linkage.end()) {
        const linkage_info& li = (*(seg.linkage.find(offset))).second;
        if (li.name == name && li.offset == offset)
            return 0;   // duplicate entry; see comments in _scan_seg() in misc.c
        cerr << "internal error: " << seg_addr_t(segno,offset) << " Adding linkage for '" << name << "', but we already had '" << li.name << "'." << simh_nl;
        ret = -1;
    } // else cerr << "note: " << seg_addr_t(segno,offset) << " Adding linkage for '" << name << "'." << simh_nl;

    seg.linkage[offset] = linkage_info(name, offset);

    // TODO: Move source_file to a single list instead of requiring user to specify segments
    for (list<source_file>::iterator src_it = seg.source_list.begin(); src_it != seg.source_list.end(); src_it++) {
        source_file& src = *src_it;
        for (map<int,entry_point>::iterator e_it = src.entries.begin(); e_it != src.entries.end(); e_it++) {
            entry_point& ep = (*e_it).second;
            if (ep.name == name) {
                int delta = offset - ep.offset;
                cout << "Linkage: " << name << " at " << seg_addr_t(segno,offset) << " matches: " << src.fname << " with offset " << ep.offset << ".  Delta is " << delta << simh_nl;
                if (src.reloc < 0) {
                    src.reloc = delta;
                    if (src._lo != -1)
                        cerr << "internal error: " << src.fname << " has _lo of " << src._lo << simh_nl;
                    src._lo = 0;
                    if (segments[segidx].source_map[src.lo()] != NULL)
                        cerr << "internal error: " << oct << segno << "|" << src.lo() << " already has a source file listed." << simh_nl;
                    else
                        segments[segidx].source_map[src.lo()] = &src;
                } else
                    if (src.reloc != delta)
                        cerr << "Linkage: Warning: Prior delta for this source file was " << src.reloc << simh_nl;
            }
        }
    }

    return ret;
}

// ============================================================================

int seginfo_find_all(int segno, int offset, where_t *wherep)
{
    if (wherep == NULL)
        return -1;

    wherep->file_name = NULL;
    wherep->entry = NULL;
    wherep->entry_offset = -1;
    wherep->entry_hi = -1;
    wherep->line_no = -1;
    wherep->line = NULL;

    int segidx = (segno == -1) ? max_seg + 1 : segno;
    const seginfo& seg = segments[segidx];
    if (seg.empty()) {
//if (segno == 0427) cerr << "DEBUG: Seg " << segno << "is empty." << simh_endl;
        return -1;
    }

    // BUG: we need to use relocation info... -- fixed?

    /*
        Find the correct source file
    */

    const source_file* srcp;
    if (seg.source_map.empty()) {
        srcp = NULL;
        // if (segno == 0431 && 0713 <= offset && offset <= 0720) cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": No source map." << simh_endl;
    } else {
        // Find highest entry that is less than given offset
        // Note that all keys in the source_map are the relocated offset
        map<int,source_file*>::const_iterator src_it = find_lower(seg.source_map, offset);
        if (src_it == seg.source_map.end()) {
            // All are larger -- nothing matches
            if (0 && segno == 031 && 0 <= offset && offset <= 02166) {
                cerr << "DEBUG: " << seg_addr_t(segno,offset) << " All sources have higher offsets." << simh_nl;
                src_it = seg.source_map.begin();
                cerr << "DEBUG: " << seg_addr_t(segno,offset) << " First source: " << (*src_it).second->fname << ": _lo = " << (*src_it).second->_lo << ", reloc = " << (*src_it).second->reloc << ", lo() = " << (*src_it).second->lo() << simh_endl;
                src_it = -- seg.source_map.end();
                cerr << "DEBUG: " << seg_addr_t(segno,offset) << " Last source: " << (*src_it).second->fname << ": _lo = " << (*src_it).second->_lo << ", reloc = " << (*src_it).second->reloc << ", lo() = " << (*src_it).second->lo() << simh_endl;
                src_it = seg.source_map.end();
            }
        } else if (src_it == -- seg.source_map.end()) {
            // Found last entry in list, so sanity check
            source_file* last_src = (*src_it).second;
            // if (segno == 031 && 0 <= offset && offset <= 02166)
            // cerr << "DEBUG: " << seg_addr_t(segno,offset) << " Last source file (" << last_src->fname << ") matches; will sanity check." << simh_nl;
            if (last_src->lo() != offset)
                // Given offset is higher than anything in the list
                if (last_src->hi() > 0) {
                    // User has specified the highest (last) known offset used by this source file
                    if (offset > last_src->hi())
                        ++ src_it;  // fail to match
                } else {
                    // Look at last source line to estimate highest offset related to this source file
                    if (last_src-> reloc < 0)
                        { cerr << "impossible at line " << __LINE__ << simh_endl; abort(); }
                    if (! last_src->lines.empty()) {
                        map<int,source_line>::const_iterator ln_it = -- last_src->lines.end();
                        int off = (*ln_it).second.offset + last_src->reloc;
                        if (offset > off + max_alm_per_line) {
                            ++ src_it;  // fail to match
                        }
                    }
                }
        } else {
            // Found one...
            // if (segno == 031 && 0 <= offset && offset <= 02166)
            // cerr << "DEBUG: " << seg_addr_t(segno,offset) << " Found a source file." << simh_nl;
        }
        if (src_it == seg.source_map.end()) {
            srcp = NULL;
        } else {
            srcp = (*src_it).second;
            if (srcp->lo() > offset) {
                cerr << "impossible at line " << dec << __LINE__ << simh_endl;
                srcp = NULL;
                abort();
            }  else
                if (srcp->hi() > 0 && offset > srcp->hi())
                    srcp = NULL;
        }
    }
    if (srcp != NULL) {
        if (srcp->fname.empty())
            // impossible
            cerr << "DEBUG: Found src for " << seg_addr_t(segno,offset) << " with empty name." << simh_nl;

        wherep->file_name = srcp->fname.c_str();
        // impossible
        if (wherep->file_name == NULL) cerr << "DEBUG: Null string on file name for " << seg_addr_t(segno,offset) << " with name '" << srcp->fname << "'.\r\n";
        // if (segno == 031 && 0 <= offset && offset <= 02166)
        // cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << " Found source file " << srcp->fname << simh_nl;
    }
    //else if (segno == 031 && 0 <= offset && offset <= 02166)
        //cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << " No source file found." << simh_nl;


    /*
        Find entry point
    */

    // Entry points (but not files) may nest; We'll look for the highest offset
    // that's lower than our offset, but that may not be quite right...
    // BUG: ignoring "hi", which could have helped pick correct overlapping entry point

    // Find entry in linkage table, see comments above
    // Looking for highest entry <= given offset
    if (! seg.linkage.empty()) {
        map<int,linkage_info>::const_iterator li_it = seg.linkage.upper_bound(offset);
        int need_sanity_check = 0;
        if (li_it == seg.linkage.end()) {
            // All are less than the given value, so use the last entry
            -- li_it;
            need_sanity_check = 1;          // May not be a close match, need sanity test
        } else if (li_it == seg.linkage.begin()) {
            // All are larger -- nothing matches
            li_it = seg.linkage.end();
        } else {
            // Found one that's larger, so back up one
            --li_it;
        }
        if (li_it != seg.linkage.end()) {
            // wherep->entry_hi = li.last;
            const linkage_info& li = (*li_it).second;
            int fail = 0;
            if (li.hi() == -1) {
                wherep->entry_hi = -1;
                if (need_sanity_check)
                    fail = offset - li.offset > max_alm_per_line;
            } else if (li.hi() >= offset) {
                wherep->entry_hi = li.entry->last;
                need_sanity_check = 0;
            } else {
                wherep->entry_hi = -1;
                fail = 1;
            }
            if (!fail) {
                wherep->entry = li.name.c_str();
                wherep->entry_offset = li.offset;
            }
        }
    }

    if (0 && segno == 031 && 0 <= offset && offset <= 02166) {
        if (wherep->entry)
            cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << " Found entry " << wherep->entry << simh_nl;
        else
            cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << " No entry found." << simh_nl;
    }


    /*
        Find Line
    */

    // BUG: use find_lower for consistency ...
    if (srcp && !srcp->lines.empty() && srcp->reloc < 0)
        cerr << "DEBUG(line " << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << " No relocation info." << simh_endl;
    if (srcp && !srcp->lines.empty() && srcp->reloc >= 0) {
        // Source lines contain and are keyed by their un-relocated offset
        int src_offset = offset - srcp->reloc;
        if (0 && segno == 031 && 0 <= offset && offset <= 02166) {
            cerr << "Source: _lo = " << srcp->_lo << ", reloc = " << srcp->reloc << ", lo() = " << srcp->lo();
            cerr << "; Line offsets range from ";
            map<int,source_line>::const_iterator ln_it = srcp->lines.begin();
            cerr << offset_t((*ln_it).first) << " (" << (*ln_it).second.offset << ") .. ";
            ln_it = -- srcp->lines.end();
            cerr << offset_t((*ln_it).first) << " (" << (*ln_it).second.offset << ")." << simh_endl;
        }
        map<int,source_line>::const_iterator ln_it = srcp->lines.upper_bound(src_offset);
        int need_sanity_check = 0;
        if (ln_it == srcp->lines.end()) {
            // All are less than the given value, so use the last entry
            -- ln_it;
            //if (segno == 031 && 0 <= offset && offset <= 02166) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": at end" << simh_endl;
            need_sanity_check = 1;          // May not be a close match, need sanity test
            // Sanity Check
            if (src_offset - (*ln_it).second.offset > max_alm_per_line) {
                ++ ln_it;   // fail to match
            }
        } else if (ln_it == srcp->lines.begin()) {
            // All are larger -- nothing matches
            //if (segno == 031 && 0 <= offset && offset <= 02166) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": all larger" << simh_endl;
            ln_it = srcp->lines.end();
        } else {
            // Found one that's larger, so back up one
            // if (segno == 031 && 0 <= offset && offset <= 02166) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": backing up" << simh_endl;
            --ln_it;
        }
        if (ln_it != srcp->lines.end()) {
            const source_line& sl = (*ln_it).second;
            //if (segno == 031 && 0 <= offset && offset <= 02166) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": found line " << dec << sl.line_no << simh_endl;
            if (sl.offset > src_offset)
                cerr << "crap, line " << dec << __LINE__ << ": impossible." << simh_endl;
            wherep->line_no = sl.line_no;
            wherep->line = sl.text.c_str();
        }
    }

    return wherep->file_name == NULL && wherep->entry == NULL && wherep->line_no == -1;
}
