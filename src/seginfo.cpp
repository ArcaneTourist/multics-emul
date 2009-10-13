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

// BUG: int seginfo_add_entry(int seg, int first, int last, const char* fname)

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

    if (lo < 0)
        out << "File " << fname;
    else {
        if (hi >= 0)
            out << "Range  " << range_t(-1, lo, hi) << ":  " << fname;
        else
            out << "Offset " << lo << ":  " << fname;
    }


    if (entries.empty() && stack_frames.empty() && lines.empty()) {
        out << " -- No entry points, stack frames, or lines." << simh_nl;
        return out;
    }

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
        for (list<stack_frame>::const_iterator it = stack_frames.begin(); it != stack_frames.end(); it++) {
            const stack_frame& sl = *it;
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
    src.lo = offset;
    if (src.lo >= 0) {
        if (segments[segidx].source_map[src.lo] != NULL)
            cerr << "internal error: " << oct << segno << "|" << src.lo << " already has a source file listed." << simh_nl;
        segments[segidx].source_map[src.lo] = &src;
    }
    return src;
}

// ============================================================================

int seginfo_add_source_file(int segno, int first, int last, const char* fname)
{
    source_file& src = seginfo_add_source_file(segno, fname, first);
    src.hi = last;
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
            cout << "  Sources (with location info):" << simh_nl;
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
                if (src.lo < 0) {
                    // not displayed above
                    if (!hdr) {
                        hdr = 1;;
                        cout << "  Sources (without location info):" << simh_nl ;
                    }
                    src.print(cout, 4);
                }
            }
            if (!hdr)
                cout << "  All sources have location info." << simh_nl;
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

    // BUG: TODO: compare linkages versus what parsing source yielded...
    // Following code is just a draft...
    // BUG: listing parser does not yet generate entry info...
    for (list<source_file>::iterator src_it = seg.source_list.begin(); src_it != seg.source_list.end(); src_it++) {
        source_file& src = *src_it;
        for (map<int,entry_point>::iterator e_it = src.entries.begin(); e_it != src.entries.end(); e_it++) {
            entry_point& ep = (*e_it).second;
            if (ep.name == name)
                // cout << "Linkage: " << name << " at " << seg_addr_t(segno,offset) << " matches: " << src.name << " with offset " << src.lo << "+" << ep.offset << "=" << (src.lo + ep.offset) << ".  Delta is " << (offset - ep.offset) << simh_nl;
                cout << "Linkage: " << name << " at " << seg_addr_t(segno,offset) << " matches: " << src.fname << " with offset " << ep.offset << ".  Delta is " << (offset - ep.offset) << simh_nl;
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
if (segno == 0427) cerr << "DEBUG: Seg " << segno << "is empty." << simh_endl;
        return -1;
    }

    // BUG: we need relocation info...

    /*
        Find the correct source file
    */

    const source_file* srcp;
    if (seg.source_map.empty()) {
        srcp = NULL;
        // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": No source map." << simh_endl;
    } else {
        // Find highest entry that is less than given offset
        // map<int,source_file*>::const_iterator src_it = seg.source_map.upper_bound(offset);
        map<int,source_file*>::const_iterator src_it = find_lower(seg.source_map, offset);
        if (src_it == seg.source_map.end()) {
            // All are larger -- nothing matches
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": no match, all are larger." << simh_endl;
        } else if (src_it == -- seg.source_map.end()) {
            // Found last entry in list, so sanity check
            if ((*src_it).second->lo != offset)
                if ((*src_it).second->hi > 0) {
                    if (offset > (*src_it).second->hi)
                        ++ src_it;
                } else {
                    // need a sanity check -- BUG: should look at source lines to estimate highest offset related to this src
                    if (!(*src_it).second->lines.empty()) {
                        map<int,source_line>::const_iterator ln_it = (*src_it).second->lines.end();
                        -- ln_it;
                        int off = (*ln_it).second.offset;
                        if (off >= 0 && offset > off + 10) {
                            ++ src_it;
                            // cerr << "crap, line " << dec << __LINE__ << ": source file doesn't pass sanity check." << simh_endl;
                        }
                    }
                }
        } else {
            // Found one...
        }
        if (src_it == seg.source_map.end()) {
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": no match found." << simh_endl;
            srcp = NULL;
        } else {
            srcp = (*src_it).second;
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": match found." << simh_endl;
            if (srcp->lo > offset) {
                cerr << "impossible at line " << dec << __LINE__ << simh_endl;
                srcp = NULL;
                abort();
            } else
                if (srcp->hi > 0 && offset > srcp->hi)
                    srcp = NULL;
        }
    }
    if (srcp != NULL) {
        // if (segno == 0427) cerr << "DEBUG: Found src for " << seg_addr_t(segno,offset) << " with name '" << srcp->fname << ".\r\n";
        if (srcp->fname.empty())
            // impossible
            cerr << "DEBUG: Found src for " << seg_addr_t(segno,offset) << " with empty name." << simh_nl;

        wherep->file_name = srcp->fname.c_str();
        // impossible
        if (wherep->file_name == NULL) cerr << "DEBUG: Null string on file name for " << seg_addr_t(segno,offset) << " with name '" << srcp->fname << "'.\r\n";
    }
//cerr << "crap, line " << dec << __LINE__ << ": done with srcp." << simh_endl;

#if 0
    if (segno == 0431 && 0713 << offset && offset <= 0720)
        if (srcp)
            cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": We have src info." << simh_endl;
        else
            cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": No src info!" << simh_endl;
#endif

    /*
        Find entry point
            BUG: source entry point may not be re-located
            BUG: linkage has good info, but won't list internal procs
            BUG: Old version called add_entry() while scanning segs (and also allowed cmd line injection)
            BUG: TODO: compare linkages versus what parsing source yielded...
    */

    // Entry points (but not files) may nest; We'll look for the highest offset
    // that's lower than our offset, but that may not be quite right...

    // BUG: ignoring "hi", which could have helped pick correct overlapping entry point

{
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
            if (li.offset > offset) { cerr << "impossible at line " << dec << __LINE__ << simh_endl; abort(); }
            int fail = 0;   // BUG: this is fucking bad code
            if (li.hi() == -1) {
                wherep->entry_hi = -1;
                if (need_sanity_check)
                    fail = offset - li.offset > 10;
                // if(fail) cerr << "crap, line " << dec << __LINE__ << ": entry " << li.name << " fails sanity check." << simh_endl;
            } else if (li.hi() >= offset) {
                wherep->entry_hi = li.entry->last;
                need_sanity_check = 0;
            } else {
                wherep->entry_hi = -1;
                fail = 1;
                // if(fail) cerr << "crap, line " << dec << __LINE__ << ": entry " << li.name << " fails bounds check -- " << "looking for " << offset << ", found " << range_t(-1,li.offset,li.hi()) << simh_endl;
            }
            if (!fail) {
                if (li.name.empty()) cerr << "DEBUG: Found linkage for " << seg_addr_t(segno,offset) << " with empty name.\r\n";
                wherep->entry = li.name.c_str();
                if (wherep->entry == NULL) cerr << "DEBUG: Null string on linkage for " << seg_addr_t(segno,offset) << " with name '" << li.name << "'.\r\n";
                wherep->entry_offset = li.offset;
            }
        }
    }
}

if (0) {
    // Find entry, see comments above
    // BUG: we need relocation info...
    // BUG: not if we use linkage_info instead of source file entry points.   But they only have externals...
    map<int,entry_point>::const_iterator e_it = srcp->entries.lower_bound(offset);
    if (e_it != srcp->entries.end()) {
        if (e_it != srcp->entries.begin() && (*e_it).second.offset != offset)
            --e_it;
        const entry_point& ep = (*e_it).second;
        if (ep.offset <= offset)
            if (ep.last < 0 || offset <= ep.last) {
if (ep.name.empty()) cerr << "DEBUG: Found entry for " << seg_addr_t(segno,offset) << " with empty name.\r\n";
                wherep->entry = ep.name.c_str();
if (wherep->entry == NULL) cerr << "DEBUG: Null string on entry for " << seg_addr_t(segno,offset) << " with name '" << ep.name << "'.\r\n";
                wherep->entry_offset = ep.offset;
                wherep->entry_hi = ep.last;
            }
    }
}

    /*
        Find Line
    */

#if 0
    if (segno == 0431 && 0713 << offset && offset <= 0720)
        if (!srcp)
            cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": no src info." << simh_endl;
        else if (srcp->lines.empty())
            cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": no lines for src" << simh_endl;
        else
            cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": src has lines..." << simh_endl;
#endif
    if (srcp && !srcp->lines.empty()) {
        map<int,source_line>::const_iterator ln_it = srcp->lines.upper_bound(offset);
        int need_sanity_check = 0;
        if (ln_it == srcp->lines.end()) {
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": at end" << simh_endl;
            // All are less than the given value, so use the last entry
            -- ln_it;
            need_sanity_check = 1;          // May not be a close match, need sanity test
            // Sanity Check
            if (offset - (*ln_it).second.offset > 10) {
                // cerr << "crap, line " << dec << __LINE__ << ": last line isn't close enough." << simh_endl;
                ++ ln_it;
            }
        } else if (ln_it == srcp->lines.begin()) {
            // All are larger -- nothing matches
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": all larger" << simh_endl;
            ln_it = srcp->lines.end();
        } else {
            // Found one that's larger, so back up one
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": backing up" << simh_endl;
            --ln_it;
        }
        if (ln_it != srcp->lines.end()) {
            const source_line& sl = (*ln_it).second;
            if (sl.offset > offset)
                cerr << "crap, line " << dec << __LINE__ << ": impossible." << simh_endl;
            wherep->line_no = sl.line_no;
            wherep->line = sl.text.c_str();
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": found" << dec << sl.line_no << simh_endl;
        }
        // else if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(line" << dec << __LINE__ << "): " << seg_addr_t(segno,offset) << ": no match found." << simh_endl;
    }

#if 0
    if (srcp && !srcp->lines.empty()) {
        map<int,source_line>::const_iterator l_it = srcp->lines.lower_bound(offset);
        if (l_it == srcp->lines.end()) {
            // l_it = srcp->lines.begin();
            -- l_it;
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(find line): " << seg_addr_t(segno,offset) << ": rewind" << simh_endl;
        } else
            if (l_it != srcp->lines.begin() && (*l_it).second.offset != offset) {
                // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(find line): " << seg_addr_t(segno,offset) << ": backup" << simh_endl;
                --l_it;
            } // else if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(find line): " << seg_addr_t(segno,offset) << ": keeping" << simh_endl;
            
        const source_line& sl = (*l_it).second;
        if (sl.offset <= offset) {
            wherep->line_no = sl.line_no;
            wherep->line = sl.text.c_str();
            // if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(find line): " << seg_addr_t(segno,offset) << ": returning: line "  << dec << wherep->line_no << simh_endl;
        } // else if (segno == 0431 && 0713 << offset && offset <= 0720) cerr << "DEBUG(find line): " << seg_addr_t(segno,offset) << ": *Not* returning: line " << dec << wherep->line_no << simh_endl;
    }
#endif

    return wherep->file_name == NULL && wherep->entry == NULL && wherep->line_no == -1;
}
