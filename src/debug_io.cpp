/*
    debug_io.cpp -- Output and formatting functions related to the classes used to track and
    display changes to multics internals.  Also includes general purpose C++ I/O code.
    Everything here is used for debug output, not for actual instruction execution.
*/

using namespace std;
#include "seginfo.h"
#include <iostream>
#include <iomanip>
#include <sstream>
// #include <string.h>
// #include <list>
// #include <map>
// #include <vector>

#include <fstream>
#if 0
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file_descriptor.hpp>

namespace bio = boost::iostreams;

class fd_sink : public bio::stream_buffer<bio::file_descriptor_sink> {
    // A derived class that will report what file descriptor it's tied to.
private:
    int _fd;
public:
    int fd() { return is_open() ? _fd : -1; }
    fd_sink() :  bio::stream_buffer<bio::file_descriptor_sink> () { _fd = -1; }
    fd_sink(int fdesc) :  bio::stream_buffer<bio::file_descriptor_sink> (fdesc) { _fd = fdesc; }
};

// bio::stream_buffer<bio::file_descriptor_sink> cdebug_buf;
//fd_sink cdebug_buf;
//ostream cdebug(&cdebug_buf);
ostream cdebug(NULL);
#endif

// ============================================================================

class range_t {
public:
    range_t(int seg, int off_lo, int off_hi)
        { segno = seg; lo = off_lo; hi = off_hi; }
private:
    int segno;
    seg_offset_t lo, hi;
friend ostream& operator<<(ostream& out, const range_t& r);
};

// ============================================================================

ostream& operator<<(ostream& out, const seg_offset_t& o)
{
    if (o.offset >= 0)
        out << setw(6) << setfill('0');
    out << oct << o.offset;

    return out;
}

// ============================================================================

seg_addr_t::operator string() const
{
    char buf[60];
    if (segno != -1)
        sprintf(buf, "%03o|%06o", segno, int(offset));
    else
        sprintf(buf, "%08o", int(offset));
    return string(buf);
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
    out << "Offset " << seg_offset_t(offset) << ": entry point " << name << simh_nl;

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
    if (stack()) {
        if (stack()->owner == this) {
            out << " with own stack frame";
            out << " (which has " << stack()->automatics.size() << " automatics)";
        }
        else if (stack()->owner) {
            if (stack()->owner->name == name)
                out << " using own stack frame";
            else
                out << " using stack frame of " << stack()->owner->name;
        }
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
        for (map<int,string>::const_iterator it = automatics.begin(); it != automatics.end(); it++) {
            int offset = (*it).first;
            const string& name = (*it).second;
            out << string(indent, ' ') << seg_offset_t(offset) << " " << name << simh_nl;
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

ostream& seginfo::print(ostream& out, int indent) const
{
    if (! source_map.empty()) {
        cout << "  Sources (with re-location info):" << simh_nl;
        for (map<int,source_file*>::const_iterator it = source_map.begin(); it != source_map.end(); it++) {
            source_file* src = (*it).second;
            src->print(cout, 4);
        }
    }

    if (linkage.empty())
        cout << "  No Linkage." << simh_nl;
    else {
        cout << "  Linkage:" << simh_nl;
        for (map<int,linkage_info>::const_iterator it = linkage.begin(); it != linkage.end(); it++) {
            linkage_info li= (*it).second;
            li.print(cout, 4);
        }
    }

    if (source_list.empty()) 
        cout << "  No sources that don't have location info." << simh_nl;
    else {
        bool hdr = 0;
        for (list<source_file>::const_iterator it = source_list.begin(); it != source_list.end(); it++) {
            const source_file& src = *it;
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

    return out;
}

// ============================================================================

#if 0
void setup_streams()
    // Setup the C++ cdebug and clog streams to flow to the same places as sim_deb and the SIMH console.
    // Flushing issues would mostly disappear if we had a class that took a FILE* rather than a descriptor.
{
    extern FILE *sim_deb;

    // Make cdebug point to same descriptor as sim_deb
    if (sim_deb == NULL) {
        fd_sink* bufp = (fd_sink*) cdebug.rdbuf();
        if (bufp != NULL && bufp->is_open()) {
            //fd_sink* newbufp = new fd_sink();
            //cdebug.rdbuf(newbufp);
            cdebug.rdbuf(NULL);
            delete bufp;
        }
    } else {
        fflush(sim_deb);
        int sim_fd = fileno(sim_deb);
        fd_sink* bufp = (fd_sink*) cdebug.rdbuf();
        if (bufp == NULL || bufp->fd() != sim_fd) {
            fd_sink* newbufp = new fd_sink(sim_fd);
            cdebug.rdbuf(newbufp);
            if (bufp != NULL)
                delete bufp;
        }
    }

    // Make clog point to the same descriptor as sim_log if sim_log is open
    extern FILE *sim_log;
    if (sim_log == NULL) {
        fd_sink* bufp = (fd_sink*) clog.rdbuf();
        if (bufp == NULL || bufp->fd() != 1) {
            fd_sink* newbufp = new fd_sink(1);
            clog.rdbuf(newbufp);
            if (bufp != NULL)
                delete bufp;
        }
    } else {
        fflush(sim_log);
        int sim_fd = fileno(sim_log);
        fd_sink* bufp = (fd_sink*) clog.rdbuf();
        if (bufp == NULL || bufp->fd() != sim_fd) {
            fd_sink* newbufp = new fd_sink(sim_fd);
            clog.rdbuf(newbufp);
            if (bufp != NULL)
                delete bufp;
        }
    }
}
#endif
