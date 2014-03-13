/*
    seginfo_run.cpp -- Symbol tables and related functions for representing
    information about segments.
    Only used for debug output, not for actual instruction execution.
    See source file seginfo.cpp for most of the class member functions.
    This source file is mostly to integrate the class with the emulator.
*/
/*
   Copyright (c) 2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

using namespace std;
#include <iostream>
#include <iomanip>
// #include <string.h>
// #include <list>
// #include <map>
// #include <vector>
#include <stdlib.h>

//#include <stdio.h>
//#include <stdarg.h>

#include "hw6180.h"
extern "C" int cmd_load_alm(int32 arg, char *buf);

#include "seginfo.hpp"

static int _scan_seg(uint segno, int msgs);
int print_seg_defs(uint segno, uint hdrp, seg_defs& seg_defs, int name_only, int msgs);
static int scan_seg_defs(uint segno, AR_PR_t *defsp, int name_only, int msgs);

// =============================================================================

int scan_seg(uint segno, int msgs)
{
    // See descripton at _scan_seg().

    uint saved_seg = TPR.TSR;
    fault_gen_no_fault = 1;
    addr_modes_t amode = get_addr_mode();
    set_addr_mode(APPEND_mode);

    int saved_debug = opt_debug;
    opt_debug = 0;
    t_stat ret = _scan_seg(segno, msgs);
    if (ret > 1 && msgs)
        out_msg("xseginfo: Error processing request.\n");

    opt_debug = saved_debug;
    TPR.TSR = saved_seg;
    fault_gen_no_fault = 0;
    set_addr_mode(amode);
    return ret;
}

// ============================================================================

static int get_linkage(int msgs, uint segno, AR_PR_t *linkagep, uint *first_linkp, uint *last_linkp, AR_PR_t *defsp)
{
    // Dump the linkage info for an in-memory segment.

    t_uint64 word0, word1;
    int saved_seg = TPR.TSR;

    /* Read LOT */

    TPR.TSR = 015;
    int ret = fetch_word(segno, &word0);
    TPR.TSR = saved_seg;
    if (ret != 0) {
        if (msgs)
            out_msg("xseginfo: Error reading LOT entry 15|%o\n", segno);
        return 1;
    }
    if (word0 == 0) {
        if (msgs)
            out_msg("LOT entry for seg %#o is empty.\n", segno);
        return -1;
    }
    word2pr(word0, linkagep);
    if (msgs)
        out_msg("LOT entry for seg %#o is %o|%#o linkage pointer.\n", segno, linkagep->PR.snr, linkagep->wordno);

    /* Read definitions pointer from linkage section */

    memset(defsp, 0, sizeof(defsp));
    TPR.TSR = linkagep->PR.snr;
    ret = fetch_pair(linkagep->wordno, &word0, &word1);
    TPR.TSR = saved_seg;
    if (ret != 0)
        return 2;
    if (word0 == 0 && word1 == 0) {
        if (msgs)
            out_msg("Seg %#o has a linkage section at %o|%o with no ITS pointer to definitions.\n", segno, linkagep->PR.snr, linkagep->wordno);
        return 1;
    }
    words2its(word0, word1, defsp);
    if (defsp->PR.snr == 0 && defsp->wordno == 0) {
        if (msgs)
            out_msg("Seg %#o has a linkage section at %o|%o with no ITS pointer to definitions.\n", segno, linkagep->PR.snr, linkagep->wordno);
        return 1;
    }
    if (msgs)
        out_msg("Linkage section has ITS to defs at %o|%o\n", defsp->PR.snr, defsp->wordno);

#if 1
    /* Read link pair info from Linkage section */

    if (fetch_word(linkagep->wordno + 6, &word1) != 0)
        return 2;
    *first_linkp = word1 >> 18;
    *last_linkp = word1 & MASK18;
    if (msgs) {
        if (word1 == 0)
            out_msg("Linkage section has no references to other segments.\n");
        else {
            out_msg("Linkage section has %d references to other segments at entry #s %#o .. %#o.\n", (*last_linkp - *first_linkp) / 2 + 1, *first_linkp, *last_linkp);
        }
    }
#endif

    return 0;
}


static int _scan_seg(uint segno, int msgs)
{

    // Dump the linkage info for an in-memory segment.
    // Can be invoked by an interactive command.
    // Also invoked by the CPU to discover entrypoint names and locations so
    // that the CPU can display location information.
    // Any entrypoints found are passed to seginfo_add_linkage().
    //
    // Caller should preserve TPR.TSR -- only scan_seg() should call _scan_seg()

    t_uint64 word0, word1;

    if (opt_debug) log_msg(DEBUG_MSG, "scan-seg", "Starting for seg %#o\n", segno);

    /* Locate and report the last non-zero word of the segment */
    if (msgs) {
        /* Get last word of segment -- but in-memory copy may be larger than original, so search for last non-zero word */
        TPR.TSR = segno;
        if (fetch_word(0, &word0) != 0) {
            if (msgs)
                out_msg("xseginfo: Cannot read first word of segment %#o.\n", segno);
        }
        SDW_t *sdwp = get_sdw();        // Get SDW for TPR.TSR
        if (sdwp == 0) {
            out_msg("xseginfo: Cannot find SDW segment descriptor word for segment %#o.\n", segno);
        } else {
            out_msg("SDW segment descriptor word for %#o: %s\n", segno, sdw2text(sdwp));
            int bound = 16 * (sdwp->bound + 1);
            t_uint64 last = 0;
            for (bound -= 2; bound > 0; bound -= 2) {
                if (fetch_pair(bound, &word0, &word1) != 0) {
                    out_msg("xseginfo: Error reading %#o|%#o/2\n", segno, bound);
                    break;
                }
                // don't stop for 'b' 'k' 'p' 't'
                if (word0 != 0 && word0 != 0142153160164) {
                    last = word0;
                    break;
                }
                if (word1 != 0 && word1 != 0142153160164) {
                    last = word1;
                    ++ bound;
                    break;
                }
            }
            if (bound > 0) {
                out_msg("Last non-zero word might be at %#o|%#o: %012llo\n", segno, bound, last);
            }
        }
    }
    
    /* Read LOT, definitions pointer from linkage section, link pair info */
    AR_PR_t linkage;
    uint first_link;
    uint last_link;
    AR_PR_t defs;
    int ret = get_linkage(msgs, segno, &linkage, &first_link, &last_link, &defs);
    if (ret == -1)
        return 0;
    if (ret != 0)
        return ret;

    TPR.TSR = defs.PR.snr;

    /* Definitions */

    ret = scan_seg_defs(segno, &defs, 0, msgs);
    if (ret != 0)
        return ret;

    /* Linkage Section */

    if (msgs && first_link != 0 && last_link != 0) {
        out_msg("\n");
        out_msg("Linkage section:\n");
        for (int link = first_link; link <= last_link; link += 2) {
            TPR.TSR = linkage.PR.snr;
            if (fetch_pair(linkage.wordno + link, &word0, &word1) != 0)
                return 2;
            if ((word0 & 077) == 043) {
                // snapped link
                AR_PR_t pr;
                if (words2its(word0, word1, &pr) != 0)
                    out_msg("Link pair at %#o: %012llo %012llo: snapped link -- bad ptr\n", link, word0, word1);
                else
                    out_msg("Link pair at %#o: %012llo %012llo: snapped link %o|%#o\n", link, word0, word1, pr.PR.snr, pr.wordno);
            } else if ((word0 & 077) == 046) {
                // unsnapped link
                uint exp_ptr = word1 >> 18;
                // out_msg("Link pair at %#o: %012llo %012llo: unsnapped link with exp-ptr %#o.\n", link, word0, word1, exp_ptr);
                out_msg("Link pair at %#o: %#o|%#04o: unsnapped link ", link, TPR.TSR, linkage.wordno + link);
                t_uint64 word2;
                TPR.TSR = defs.PR.snr;
                if (fetch_word(defs.wordno + exp_ptr, &word2) != 0) {
                    out_msg("Cannot read expression word at %o|%o\n", TPR.TSR, defs.wordno + exp_ptr);
                    return 2;
                }
                uint offset = word2 >> 18;
                t_uint64 a, q;
                if (fetch_word(defs.wordno + offset, &a) != 0) {
                    out_msg("... exp-word at %o|%o: %012llo: offset %o|%o => <error>\n", linkage.PR.snr, exp_ptr, word2, TPR.TSR, offset);
                    return 2;
                }
                if (fetch_word(defs.wordno + offset + 1, &q) != 0) {
                    out_msg("... exp-word at %o|%o: %012llo: offset %o|%o => <error>\n", linkage.PR.snr, exp_ptr, word2, TPR.TSR, offset);
                    return 2;
                }
                // out_msg("\texp-word %012llo: offset %o => type-pair (%#llo,%#llo)\n", word2, offset, a, q);
                int exp_offset = word2 & MASK18;
                int typ = a >> 18;
                if (typ == 0 || typ > 6) {
                    out_msg("of unknown type %d.\n", typ);
                    continue;
                } else if (typ == 1 || typ == 2 || typ == 5) {
                    out_msg("of Type %d.\n", typ);
                    continue;
                } else {
                    // types 3, 4, and 6
                    char sname[513];
                    char ename[513];
                    int ql = q & MASK18;
                    int qu = q >> 18;
                    if (qu == 0)
                        *sname = 0;
                    else if (fetch_acc(fetch_word, defs.wordno + qu, sname) != 0)
                        return 2;
                    if (ql == 0)
                        *ename = 0;
                    else if (fetch_acc(fetch_word, defs.wordno + ql, ename) != 0)
                        return 2;
                    if (exp_offset == 0)
                        out_msg("of type %d to %s$%s\n", typ, sname, ename);
                    else
                        out_msg("of type %d to %s$%s%+d\n", typ, sname, ename, exp_offset);
                }
            } else {
                out_msg("Link pair at %#o: %012llo %012llo: unrecognized/ignorable.\n", link, word0, word1);
            }
        }
    }

    return 0;
}

// ============================================================================

static int _get_seg_name(uint segno); // FIXME

int get_seg_name(uint segno)
{
    uint saved_seg = TPR.TSR;
    flag_t fgen = fault_gen_no_fault;
    fault_gen_no_fault = 1;
    addr_modes_t amode = get_addr_mode();
    set_addr_mode(APPEND_mode);

    int msgs = 0;

    int saved_debug = opt_debug;
    opt_debug = 0;
    t_stat ret = _get_seg_name(segno);
    if (ret > 1 && msgs)
        out_msg("get_seg_name: Error processing request.\n");

    opt_debug = saved_debug;
    TPR.TSR = saved_seg;
    fault_gen_no_fault = fgen;
    set_addr_mode(amode);
    return ret;
}

static int _get_seg_name(uint segno)
{
    int msgs = 1;

    /* Read LOT, definitions pointer from linkage section, link pair info */
    AR_PR_t linkage;
    uint first_link;
    uint last_link;
    AR_PR_t defs;
    int ret = get_linkage(msgs, segno, &linkage, &first_link, &last_link, &defs);
    if (ret == -1)
        return 0;
    if (ret != 0)
        return ret;

    TPR.TSR = defs.PR.snr;

    /* Definitions */

    // FIXME: unfinished...
    // much gunk; "xseginfo" also provides defs
    //ret = scan_seg_defs(segno, &defs, 1, 0);

    return ret;
}


// ============================================================================

static int scan_seg_defs(uint segno, AR_PR_t *defsp, int name_only, int msgs)
{
    const char* moi = "scan_seg_defs";

    uint hdrp = defsp->wordno;
    seg_defs seg_defs;
    int ret = seg_defs.scan(hdrp, fetch_word);
    if (ret != 0)
        return ret;

    return print_seg_defs(segno, hdrp, seg_defs, name_only, msgs);
}

int print_seg_defs(uint segno, uint hdrp, seg_defs& seg_defs, int name_only, int msgs)
{
    // Moved out of _scan_seg()
    // Intent is to make usable for getting segment names

    const char* moi = "scan_seg_defs";

    if (seg_defs.defp() == 0) {
        if (msgs)
            out_msg("Definition header doesn't point to any entries\n");
        return 0;
    }

    if (msgs) {
        out_msg("\n");
        out_msg("Definition header points to first entry at offset %#o.   This segment provides the following:\n", seg_defs.defp());
    }

    map<int, def_t>::const_iterator def_it;
    const char* first_name = NULL;
    for (def_it = seg_defs.defs.begin(); def_it != seg_defs.defs.end(); ++ def_it) {
        unsigned defp = (*def_it).first;
        const def_t& def = (*def_it).second;
        if (msgs || (def.type == def_seg_name && name_only))
            out_msg("Def entry at %o|%04o (offset %04o): class %#o.  ", segno, defp, defp - hdrp, def.type);
        switch(def.type) {
            case def_text:
                if (msgs)
                    out_msg("Text %s: link %o|%#o\n", def.name().c_str(), segno, def.link.offset);
                if (seginfo_add_linkage(segno, def.link.offset, def.link.name.c_str()) != 0)
                    log_msg(INFO_MSG, moi, "call to seginfo_add_linkage failed\n");
                break;
            case def_seg_name:
                if (first_name == NULL) {
                    // Segments can have multiple names
                    first_name = def.name().c_str();
                    // seginfo_add_name(segno, thing_relp, first_name);    // call unneeded, only first name matters
                    if (msgs || name_only)
                        out_msg("Segment Name is %s; first def is at offset %#o.\n", first_name, seg_defs.firstp());
                } else {
                    if (msgs || name_only)
                        out_msg("Segment %s has alias %s\n", first_name, def.name().c_str());
                }
                break;
            case def_symbol:
                if (msgs)
                    out_msg("Name is %s; offset is %#o within %s section\n", def.name().c_str(), def.offset, "symbol");
#if 0
                if (strcmp(buf, "bind_map") == 0) {
                } else if (strcmp(buf, "symbol_table") == 0) {
                } else
                    ...
#endif
                break;
            default:
                if (msgs)
                    out_msg("Name is %s with offset thing_relp = %#o within the class %d section.\n", def.name().c_str(), defp, def.type);
        }
    }
    return 0;
}

// ============================================================================
