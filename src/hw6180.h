#include "sim_defs.h"

typedef unsigned int uint;  // efficient unsigned int, at least 16 or 32 bits

/*
        For efficiency, we mostly use full ints instead of bit fields for
    the flags and other fields of most of the typedefs here.  When necessary,
    such as when an instruction saves a control register to memory, the more
    efficient variables are copied into appropriate bit fields.
        Also note that we only represent selected fields of some of the
    registers.  The simulator doesn't need some of the various scratches
    and the OS doesn't need them either.
        Descriptions of registers may be found in AN87 and AL39.
*/


/* Misc enum typdefs */

typedef enum { ABSOLUTE_mode, APPEND_mode, BAR_mode } addr_modes_t;
typedef enum { NORMAL_mode, PRIV_mode } instr_modes_t;
typedef enum {
    ABORT_cycle, FAULT_cycle, EXEC_cycle, INTERRUPT_cycle,
    FETCH_cycle,
// CA FETCH OPSTORE, DIVIDE_EXEC
} cycles_t;

enum faults {
    shutdown_fault = 0, timer_fault = 4, connect_fault = 8, illproc_fault = 10, startup_fault = 12, trouble_fault = 31,
    oob_fault=32    // out-of-band, simulator only
};

// Simulator stop codes (as returned by sim_instr)
//      Zero and values above 63 reserved by SIMH
enum sim_stops {
    STOP_MEMCLEAR = 1,  // executing empty memory; zero reserved by SIMH
    STOP_BUG,           // impossible conditions, coding error
    STOP_ODD_FETCH,
    STOP_IBKPT,         // breakpoint
};


/* Format of a 36 bit instruction word */
typedef struct {
    /* uint32 addr; /* 18 bits at 0..17 */
    uint offset;    // 18 bits at 0..17; 18 bit offset or seg/offset pair
    uint opcode;    /* 10 bits at 18..27 */
    uint inhibit;   /* 1 bit at 28 */
    uint pr_bit;    // 1 bit at 29; use offset[0..2] as pointer reg?
    uint tag;       /* 6 bits at 30..35 */
} instr_t;


/* Indicator register (14 bits) */
typedef struct {
    uint zero;              /* 1 bit at 18 */
    /* ... */
    uint not_bar_mode;      /* 1 bit at 28 */
    /* ... */
    uint mid_instr_intr_fault;/* 1 bit at 30 */
    uint abs_mode;          /* 1 bit at 31 */
} IR_t;

// more simulator state variables for the cpu
// these probably belong elsewhere..
typedef struct {
    t_bool ic_odd;  // executing odd pair?
    t_bool xfr;     // most recent instruction was a transfer instr?
} cpu_state_t;


/* Fault register (72 bits) */
typedef struct {
    // Multics never examines this (just the CPU) -- multicians.org glossary
    uint ill_op:1;      /* 1 bit at 0 */
    uint ill_mod:1;     /* 1 bit at 1 */
    uint ill_slv:1;     /* 1 bit at 2 */
    uint ill_proc:1;    /* 1 bit at 3 */
    /* ... */
} fault_reg_t;

// Simulator-only interrupt and fault info
// tentative
typedef struct {
    t_bool any;                 // true if any of the below are true
    t_bool int_pending;
    int low_group;          // Lowest group-number fault preset
    uint32 group7;          // bitmask for multiple group 7 faults
    int fault[6];           // only one fault in groups 1..6 can be pending
    t_bool interrupts[32];
} events_t;

// PPR Procedure Pointer Register (pseudo register)
//      Use: Holds infor relative to the location in main memory of the
//      procedure segment in execution and the location of the current
//      instruction within that segment
typedef struct {
    uint PRR;       /* Procedure ring register; 3 bits @ 0[0..2] */
    uint PSR;       /* Procedure segment register; 15 bits @ 0[3..17] */
    uint P;         /* Privileged bit; 1 bit @ 0[18] */
    uint IC;        /* Instruction counter, 18 bits */
} PPR_t;

// TPR Temporary Pointer Register (pseudo register)
//      Use: current virt addr used by the processor in performing addr
//      prep for operands, indirect words, and instructions.   At the
//      completion of addr prep, the contents of the TPR is presented
//      to the appending unit associative memories for translation
//      into the 24-bit absolute main memory address.
typedef struct {
    uint TRR;   // Current effective ring number, 3 bits
    uint TSR;   // Current effective segment number, 15 bits
    uint TBR;   // Current bit offset as calculated from ITS and ITP
    uint CA;    // Current computed addr relative to the segment in TPR.TSR
} TPR_t;


/* Control unit data (288 bits) */
typedef struct {
    /*      This is a collection of flags and registers from the
        appending unit and the control unit.  The scu and rcu
        instructions store and load these values to an 8 word
        memory block.
            The CU data may only be valid for use with the scu and
        rcu instructions.
            Comments indicate format as stored in 8 words by the scu
        instruction.
    */

    /* NOTE: PPR (procedure pointer register) is a combination of registers:
        From the Appending Unit
            PRR bits [0..2] of word 0
            PSR bits [3..17] of word 0
            P   bit 18 of word 0
        From the Control Unit
            IC  bits [0..17] of word 4
    */

#if 0
    /* word 0 */
    // PPR portions copied from Appending Unit
    uint PPR_PRR;       /* Procedure ring register; 3 bits @ 0[0..2] */
    uint PPR_PSR;       /* Procedure segment register; 15 bits @ 0[3..17] */
    uint PPR_P;         /* Privileged bit; 1 bit @ 0[18] */
    /* uint64 word0bits; /* Word 0, bits 18..32 (all for the APU) */
    uint FCT;           /* Fault counter; 3 bits at 0[33..35];

    /* word 1 */
    uint64 word1bits;   /* Word1, bits [0..19] and [35] */
    uint IA;        /* 4 bits @ 1[20..23] */
    uint IACHN;     /* 3 bits @ 1[24..26] */
    uint CNCHN;     /* 3 bits @ 1[27..29] */
    uint FIADDR     /* 5 bits @ 1[30..34] */

    /* word 2 */
    uint TPR_TRR;   // 3 bits @ 2[0..2];  temporary ring register
    uint TPR_TSR;   // 15 bits @ 2[3..17]; temporary segment register
    // unused: 10 bits at 2[18..27]
    // uint cpu_no; // 2 bits at 2[28..29]; from maint panel switches
    uint DELTA      // 6 bits at 2[30..35]; addr increment for repeats
    
    /* word 3 */

    /* word 4 */
    // IC belongs to CU
    uint IC;        // 18 bits at 4[0..17]; instruction counter aka ilc
    // copy of IR bits 14 bits at 4[18..31]
    // unused: 4 bits at 4[32..36];

    /* word 5 */
    uint CA;        // 18 bits at 5[0..17]; computed address value (offset) used in the last address preparation cycle
    // cu bits for repeats, execute double, restarts, etc
    uint CT_HOLD;   // 6 bits at 5[30..35]; contents of the "remember modifier" register
#endif

    /* word 6 */
    instr_t IR;     /* Working instr register; addr & tag are modified */

    /* word 7 */
    // instr_t IRODD;   /* Instr holding register; odd word of last pair fetched */
    t_uint64 IRODD; /* Instr holding register; odd word of last pair fetched */
    
} ctl_unit_data_t;


// Physical Switches
typedef struct {
    int FLT_BASE;   // normally 7 MSB of 12bit fault base addr
} switches_t;


/*
    Operations on 36-bit pseudo words

    Multics 36bit words are simulated with 64bit integers.  Multics
    documentation refers to the right-most or least significant bit
    as position 35.   Position zero is the leftmost bit.   This is
    unusual.  Usually, we refer to the LSB as position zero and the
    MSB (most significant bit) as the highest position number.  The latter
    convention matches up the value of a bit position with its
    twos-complement value, e.g. turning only bit #4 results is a value
    of 2 raised to the 4th power.  With the Multics notational convention,
    turning on only bit 35 results in the twos-complement value of one.

    The following macros support operating on 64bit words as though the
    right-most bit were bit 35.  This means that bit positon zero is
    in the 36th bit from the right -- in the middle of the 64 bit word.
*/

/*  Extract (i)th bit of a 36 bit word (held in a uint64). */
#define bitval36(word,i) ( ((word)>>(35-i)) & (uint64_t) 1 )
/*  Value of a 36bit word (held in a uint64) after setting or clearing the
    the (i)th bit. */
#define bitset36(word,i) ( (word) | ( (uint64_t) 1 << (35 - i)) )
#define bitclear36(word,i) ( (word) & ~ ( (uint64_t) 1 << (35 - i)) )
static t_uint64 getbits36(t_uint64 x, int i, int n) {
    // bit 35 is right end, bit zero is 36th from the left
    return (x >> (35-i+n-1)) & ~ (~0 << n);
}

// obsolete typedef -- hold named register sub-fields in their inefficient
// native format.
/* #define CU_PPR_P(CU) (bitval36(CU.word0bits, 18)) */

// ============================================================================

extern void debug_msg(const char* who, const char* format, ...);
extern void complain_msg(const char* who, const char* format, ...);
extern void execute_instr(void);
extern void fault_gen(enum faults);
extern int decode_addr(instr_t* ip, t_uint64* addrp);
extern int decode_ypair_addr(instr_t* ip, t_uint64* addrp);
extern int fetch_pair(uint IC, t_uint64* word0p, t_uint64* word1p);
int fetch_instr(uint IC, instr_t *ip);



#include "opcodes.h"
