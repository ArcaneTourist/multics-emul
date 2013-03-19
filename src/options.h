/*
    Compile time options

    See also the settings for sys_opts in hw6180_sys.c

    The main difference between settings in sys_opts and the #defines here is
    that the #defines here are expected to have noticable performance impact.
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#ifndef _OPTIONS_H
#define _OPTIONS_H

// Per-instruction statistics
// Counts may be interesting, but wall-clock timings may be inaccurate
#define FEAT_INSTR_STATS 1
#define FEAT_INSTR_STATS_TIMING 0

// Memory is represented via 64 bit integers representing the 36bit words. If
// this #define is true, memory is initialized to all ones and every memory read
// checked.  If an uninitialized read is found, a warning is displayed, but a zero
// value is still returned to the emulator as if all memory had been initialized
// to zero.  Note that normal operation can only write values with the lower 36
// bits set on but not the remaining 26 of 64 bits.
#define FEAT_MEM_CHECK_UNINIT 0

#endif  // _OPTIONS_H
