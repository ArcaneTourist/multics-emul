/*
    seginfo.cpp -- Symbol tables and related functions for representing
    information about segments.
    Only used for debug output, not for actual instruction execution.
*/

/*
    BUG: Need to implement algorithm for searching through an ordered list
    of (possibly nested) ranges.
    Currently, some ranges may have unknown ending point.

    BUG: relocation
        Scanning config_.list shows find: at offset 314; seg scanning
        shows find$find at 401.
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

extern "C" void log_msg(int, const char*, ...); // BUG

// ============================================================================

segments_t segments;
static const int max_alm_per_line = 10;     // Used for estimating the location of the last machine instruction for the last source line of a file

// ============================================================================
// ============================================================================
// ============================================================================

entry_point* source_file::find_entry(const string& s)
{
    map<string,entry_point*>::iterator it = entries_by_name.find(s);
    if (it == entries_by_name.end())
        return NULL;
    else
        return (*it).second;
}

// ============================================================================

template<typename Map> typename Map::const_iterator 

find_lower(Map const& m, typename Map::key_type const& k)
{
    typename Map::const_iterator it = m.upper_bound(k);
    if (it == m.begin())
        return m.end();
    return --it;
}

// ============================================================================

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
    seginfo& seg = segments(segno);
    seg.source_list.push_back(source_file(fname));
    source_file& src = segments(segno).source_list.back();
    if (offset < 0)
        src._lo = -1;
    else {
        src._lo = 0;
        src._hi = -1;
        src.reloc = offset;
        if (segments(segno).source_map[src.lo()] != NULL)
            cerr << "internal error: " << oct << segno << "|" << src.lo() << " already has a source file listed." << simh_nl;
        else
            segments(segno).source_map[src.lo()] = &src;
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
    for (int segno = -1; segno <= segments_t::max_segno; ++ segno) {
        const seginfo& seg = segments(segno);
        if (seg.empty())
            continue;
        if (segno == -1)
            cout << "Absolute Memory:" << simh_nl;
        else
            cout << "Segment " << oct << segno << ":" << simh_nl;
        seg.print(cout);
    }
}

// ============================================================================

#if 0

extern "C" int seginfo_add_name(int segno, int offset, const char *name);   // BUG
int seginfo_add_name(int segno, int offset, const char *name)
    // A segment may have multiple names or aliases.   A bound segment is built from
    // multiple individual segments.  So, while a bound segment will have multiple names,
    // the various aliases within the bound segment are not all equivalent; only
    // the alias equivalencies match that of the original segments.   Aliases that
    // all refer to the same originally unbound segment will all be located at the same
    // offset.
    //
    /*
        Example:

        bound_library_wired_ has the following map:

        Definition header points to first entry at offset 02.   This segment provides the following:

        Def entry at 20|34762 (offset 0002): class 03.  Segment Name is bound_library_wired_; first def is at offset 013.
        Def entry at 20|34773 (offset 0013): class 02.  Name is symbol_table; offset is 0 within symbol section
        Def entry at 20|35002 (offset 0022): class 02.  Name is bind_map; offset is 0 within symbol section
        Def entry at 20|43036 (offset 6056): class 03.  Segment Name is clock_; first def is at offset 041.
            Def entry at 20|35021 (offset 0041): class 0.  Text clock_: link 41|055
        Def entry at 20|43041 (offset 6061): class 03.  Segment Name is config_; first def is at offset 044.
        Def entry at 20|43044 (offset 6064): class 03.  Segment Name is config; first def is at offset 044.
        Def entry at 20|43047 (offset 6067): class 03.  Segment Name is find; first def is at offset 044.
        Def entry at 20|43052 (offset 6072): class 03.  Segment Name is find_2; first def is at offset 044.
        Def entry at 20|43055 (offset 6075): class 03.  Segment Name is find_periph; first def is at offset 044.
        Def entry at 20|43060 (offset 6100): class 03.  Segment Name is find_peripheral; first def is at offset 044.
        Def entry at 20|43063 (offset 6103): class 03.  Segment Name is find_parm; first def is at offset 044.
            Def entry at 20|35024 (offset 0044): class 0.  Text config_: link 41|0360
            Def entry at 20|35031 (offset 0051): class 0.  Text config: link 41|0367
            Def entry at 20|35036 (offset 0056): class 0.  Text find: link 41|0401

        This yields entrypoints:
            Segment clock_
                41|055 clock_$clock_ 
            Segment config_ (aka config, find, find_2, find_periph, find_periphral, find_parm)
                41|0401 config_$find
                    In theory, with names: config$find, find$find, find_2$find, find_periph$find, etc
    */
{
}

#endif

// ============================================================================

int seginfo_add_linkage(int segno, int offset, const char* name)
{
    if (name == NULL)
        return -1;
    if (segno > segments_t::max_segno)
        return -1;

    seginfo& seg = segments(segno);
    int ret = 0;
    if (seg.linkage.find(offset) != seg.linkage.end()) {
        const linkage_info& li = (*(seg.linkage.find(offset))).second;
        if (li.name == name && li.offset == offset)
            return 0;   // duplicate entry; see comments in _scan_seg() in misc.c
        // It seems that it's valid for multiple entries to be located at the same offset.
        // BUG: this is mostly a non issue, but *might* mean that the stack tracking code for
        // displaying automatic variables will refuse to handle these entry points.
        log_msg(/*BUG*/ 0, "SEGINFO", "Entry point %s is at %03o|%o, but we already have %s recorded as being at that address.\n", name, segno, offset, li.name.c_str());
        // cerr << "internal error: " << seg_addr_t(segno,offset) << " Adding linkage for '" << name << "', but we already had '" << li.name << "'." << simh_nl;
        ret = -1;   // last one wins
        // return -1;   // first one wins
    }

    seg.linkage[offset] = linkage_info(name, offset);

    // TODO: Move source_file to a single list instead of requiring user to specify segments
    for (list<source_file>::iterator src_it = seg.source_list.begin(); src_it != seg.source_list.end(); src_it++) {
        source_file& src = *src_it;
        for (map<int,entry_point>::iterator e_it = src.entries.begin(); e_it != src.entries.end(); e_it++) {
            entry_point& ep = (*e_it).second;
            if (ep.name == name) {
                int delta = offset - ep.offset;
                // cout << "Linkage: " << name << " at " << seg_addr_t(segno,offset) << " matches: " << src.fname << " with offset " << ep.offset << ".  Delta is " << delta << simh_nl;
                if (src.reloc < 0) {
                    src.reloc = delta;
                    if (src._lo != -1)
                        cerr << "internal error: " << src.fname << " has _lo of " << src._lo << simh_nl;
                    src._lo = 0;
                    if (segments(segno).source_map[src.lo()] != NULL)
                        cerr << "internal error: " << oct << segno << "|" << src.lo() << " already has a source file listed." << simh_nl;
                    else
                        segments(segno).source_map[src.lo()] = &src;
                } else
                    if (src.reloc != delta)
                        cerr << "Linkage: Warning: Prior delta for this source file was " << src.reloc << simh_nl;
                seg.linkage[offset].entry = &ep;
            }
        }
    }

    return ret;
}

// ============================================================================

/*
 * seginfo::find_entry
 *
 * Find entry in linkage table
 */

map<int,linkage_info>::const_iterator seginfo::find_entry(int offset) const
{
    // Entry points (but not files) may nest; We'll look for the highest offset
    // that's lower than the given offset, but that may not be quite right...
    // BUG: ignoring "hi", which could have helped pick correct overlapping entry point

    // Looking for highest entry <= given offset
    if (linkage.empty())
        return linkage.end();
    else {
        map<int,linkage_info>::const_iterator li_it = linkage.upper_bound(offset);
        int need_sanity_check = 0;
        if (li_it == linkage.end()) {
            // All are less than the given value, so use the last entry
            -- li_it;
            need_sanity_check = 1;          // May not be a close match, need sanity test
        } else if (li_it == linkage.begin()) {
            // All are larger -- nothing matches
            li_it = linkage.end();
        } else {
            // Found one that's larger, so back up one
            --li_it;
        }
        if (li_it != linkage.end()) {
            const linkage_info& li = (*li_it).second;
            if (li.hi() == -1) {
                if (need_sanity_check)
                    if (offset - li.offset > max_alm_per_line)
                        li_it == linkage.end(); // fail
            } else if (li.hi() >= offset) {
                // OK
            } else {
                li_it == linkage.end(); // fail
            }
        }
        return li_it;
    }
}

// ============================================================================

const source_line* source_file::get_line(int offset, int use_relocation) const
{
    map<int,source_line>::const_iterator lit = find_line(offset, use_relocation);
    if (lit == lines.end())
        return NULL;
    return &(*lit).second;
}

// ============================================================================

map<int,source_line>::const_iterator source_file::find_line(int offset, int use_relocation) const
{
    // BUG: use find_lower for consistency ...
    // TODO: Performance: Cache prior successful iterator and search fwd from there first

    if (lines.empty())
        return lines.end();
    if (reloc < 0 && use_relocation)
        return lines.end();

    // Source lines contain and are keyed by their un-relocated offset
    int src_offset = offset;
    if (use_relocation)
        src_offset -= reloc;
    map<int,source_line>::const_iterator ln_it = lines.upper_bound(src_offset);
    int need_sanity_check = 0;
    if (ln_it == lines.end()) {
        // All are less than the given value, so use the last entry
        -- ln_it;
        need_sanity_check = 1;          // May not be a close match, need sanity test
        // Sanity Check
        if (src_offset - (*ln_it).second.offset > max_alm_per_line) {
            ++ ln_it;   // fail to match
        }
    } else if (ln_it == lines.begin()) {
        // All are larger -- nothing matches
        ln_it = lines.end();
    } else {
        // Found one that's larger, so back up one
        --ln_it;
    }

    if (ln_it != lines.end()) {
        const source_line& sl = (*ln_it).second;
        if (sl.offset > src_offset)
            cerr << "line " << dec << __LINE__ << ": impossible." << simh_endl;
    }

    return ln_it;
}

// ============================================================================

class loc_t {
public:
    const linkage_info* linkage;
    const source_file* file;
    const source_line* line;
    // const stack_frame* frame;
};

// ============================================================================

int seginfo_find_all(int segno, int offset, loc_t& loc)
{
    loc.linkage = NULL;
    loc.file = NULL;
    loc.line = NULL;
    // loc.frame = NULL;

    const seginfo& seg = segments(segno);
    if (seg.empty())
        return -1;

    /*
        Find the correct source file
    */

    const source_file* srcp;
    if (seg.source_map.empty()) {
        srcp = NULL;
    } else {
        // Find highest entry that is less than given offset
        // Note that all keys in the source_map are the relocated offset
        map<int,source_file*>::const_iterator src_it = find_lower(seg.source_map, offset);
        if (src_it == seg.source_map.end()) {
            // All are larger -- nothing matches
        } else if (src_it == -- seg.source_map.end()) {
            // Found last entry in list, so sanity check
            source_file* last_src = (*src_it).second;
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


    /*
        Find entry point
    */

    // Find entry in linkage table, see comments above
    // Looking for highest entry <= given offset

    map<int,linkage_info>::const_iterator li_it = seg.find_entry(offset);
    if (li_it != seg.linkage.end()) {
        const linkage_info& li = (*li_it).second;
        loc.linkage = &li;
    }


    /*
        Find Line
    */

    if (srcp != NULL)
        loc.line = srcp->get_line(offset, 1);

    loc.file = srcp;
    return loc.linkage == NULL && loc.file == NULL && loc.line == NULL;
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
    wherep->n_auto = -1;

    loc_t loc;
    int ret = seginfo_find_all(segno, offset, loc);
    if (ret != 0)
        return ret;

    const source_file* srcp = loc.file;
    if (srcp != NULL)
        wherep->file_name = srcp->fname.c_str();

    if (loc.linkage != NULL) {
        const linkage_info& li = *loc.linkage;
        wherep->entry = li.name.c_str();
        wherep->entry_offset = li.offset;
        wherep->entry_hi = li.hi();
        if (li.entry != NULL && li.entry->stack() != NULL)
            wherep->n_auto = li.entry->stack()->automatics.size();
    }

    if (loc.line != NULL) {
        wherep->line_no = loc.line->line_no;
        wherep->line = loc.line->text.c_str();
    }

    return 0;
}

// ============================================================================

void seginfo_find_line(int segno, int offset, const char**line, int *lineno)
{
    loc_t loc;
    if (seginfo_find_all(segno, offset, loc) == 0 && loc.line != NULL) {
        *lineno = loc.line->line_no;
        *line = loc.line->text.c_str();
    } else  {
        *line = NULL;
        *lineno = -1;
    }
}

// ============================================================================

int seginfo_automatic_count(int segno, int offset)
{
    const seginfo& seg = segments(segno);
    if (seg.empty())
        return -1;

    map<int,linkage_info>::const_iterator li_it = seg.find_entry(offset);
    if (li_it == seg.linkage.end())
        return -1;
    const linkage_info& li = (*li_it).second;
    if (li.entry == NULL);
        return -1;
    stack_frame *sfp = li.entry->stack();
    if (sfp == NULL)
        return -1;
    return sfp->automatics.size();
}

// ============================================================================

int seginfo_automatic_list(int segno, int offset, int *count, automatic_t *list)
{
    return -1;
}

// ============================================================================
