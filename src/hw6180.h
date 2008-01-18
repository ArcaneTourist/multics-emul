#include "sim_defs.h"

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

typedef unsigned int uint;  // efficient unsigned int, at least 32 bits

// ============================================================================
// === Misc enum typdefs

typedef enum { ABSOLUTE_mode, APPEND_mode, BAR_mode } addr_modes_t;
typedef enum { NORMAL_mode, PRIV_mode } instr_modes_t;
typedef enum {
    ABORT_cycle, FAULT_cycle, EXEC_cycle, INTERRUPT_cycle,
    FETCH_cycle,
// CA FETCH OPSTORE, DIVIDE_EXEC
} cycles_t;

enum faults {
    shutdown_fault = 0, store_fault = 1, timer_fault = 4, connect_fault = 8,
    illproc_fault = 10, startup_fault = 12, overflow_fault = 13,
    div_fault = 14, trouble_fault = 31,
    //
    oob_fault=32    // out-of-band, simulator only
};

// Simulator stop codes (as returned by sim_instr)
//      Zero and values above 63 reserved by SIMH
enum sim_stops {
    STOP_MEMCLEAR = 1,  // executing empty memory; zero reserved by SIMH
    STOP_BUG,           // impossible conditions, coding error;
    STOP_WARN,          // something odd or interesting; further exec might possible
    STOP_ODD_FETCH,
    STOP_IBKPT,         // breakpoint
};


// ============================================================================
// === Misc constants

// Clocks
#define CLK_TR_HZ (512*1) /* should be 512 kHz, but we'll use 512 Hz for now */
#define TR_CLK 1 /* SIMH allows clock ids 0..7 */

// Memory
#define IOM_MBX_LOW 01200
#define IOM_MBX_LEN 02200
#define DN355_MBX_LOW 03400
#define DN355_MBX_LEN 03000


// ============================================================================
// === Struct typdefs

/* Format of a 36 bit instruction word */
typedef struct {
    /* uint32 addr; /* 18 bits at 0..17 */
    uint offset;    // 18 bits at 0..17; 18 bit offset or seg/offset pair
    uint opcode;    /* 10 bits at 18..27 */
    uint inhibit;   /* 1 bit at 28 */
    uint pr_bit;    // 1 bit at 29; use offset[0..2] as pointer reg?
    uint tag;       /* 6 bits at 30..35 */
    uint is_value;  // is offset a value or an address? (du or dl modifiers)
    //t_uint64 value;   // 36bit value from opcode constant via du/dl
} instr_t;


/* Indicator register (14 bits [only positions 18..32 have meaning]) */
typedef struct {
    uint zero;              // bit 18
    uint neg;               // bit 19
    uint carry;             // bit 20; see AL39, 3-6
    uint overflow;          // bit 21
    // exp_overflow;        // bit 22
    // exp_underflow;       // bit 23
    uint overflow_mask;     // bit 24
    uint tally_runout;      // bit 25
    // parity_error;        // bit 26
    // parity_mask;         // bit 27
    uint not_bar_mode;      // bit 28
    // truncation;          // bit 29
    uint mid_instr_intr_fault;  // bit 30
    uint abs_mode;          // bit 31
    uint hex_mode;          // bit 32
} IR_t;

// more simulator state variables for the cpu
// these probably belong elsewhere..
typedef struct {
    t_bool ic_odd;  // executing odd pair?
    t_bool xfr;     // most recent instruction was a transfer instr?
} cpu_state_t;


/* Fault register (72 bits) */
#if 0
typedef struct {
    // Multics never examines this (just the CPU) -- multicians.org glossary
    uint ill_op:1;      /* 1 bit at 0 */
    uint ill_mod:1;     /* 1 bit at 1 */
    uint ill_slv:1;     /* 1 bit at 2 */
    uint ill_proc:1;    /* 1 bit at 3 */
    /* ... */
} fault_reg_t;
#endif

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
//      Use: Holds info relative to the location in main memory of the
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
    t_uint64 CA;// Current computed addr relative to the segment in TPR.TSR; Normally 24? bits but sized to hold 36bit non-address operands
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


// PTWAM registers, 51 bits each
typedef struct {
    //uint addr;        // 18 bits; upper 18 bits of 24bit main memory addr of page
    //uint modified;    // flag
    //uint ptr;     // 15 bits; effective segment #
    //uint pageno;  // 12 bits; 12 high order bits of CA used to fetch this PTW from mem
    uint is_full;   // flag; PTW is valid
    uint use;       // counter, 4 bits
    uint enabled;   // internal flag, not part of the register
} PTWAM_t;

// SDWAM registers, 88 bits each
typedef struct {
    //uint addr;        // 24bit main memory addr -- page table or page segment
    //uint r1;          // 3 bits
    //uint r2;          // 3 bits
    //uint r3;          // 3 bits
    //uint bound;       // 14 bits; 14 high order bits of furtherest Y-block16
    //uint r;           // flag; read perm
    //uint e;           // flag; exec perm
    //uint w;           // flag; write perm
    //uint p;           // flag; priv
    //uint u;           // flag; unpaged
    //uint g;           // flag; gate control
    //uint c;           // flag; cache control
    //uint cl;          // 14 bits; (inbound) call limiter
    //uint modified;    // flag
    //uint ptr;         // 15 bits; effective segment #
    uint is_full;   // flag; PTW is valid
    uint use;       // counter, 4 bits
    uint enabled;   // internal flag, not part of the register
} SDWAM_t;



// Physical Switches
typedef struct {
    int FLT_BASE;   // normally 7 MSB of 12bit fault base addr
} switches_t;

typedef struct {
    uint ports[8];  // SCU connectivity; designated a..h
    int scu_port;   // What port num are we connected to (same for all SCUs)
} cpu_ports_t;

typedef struct {
    // int interrupts[32];
    // uint mem_base;   // zero on boot scu
    // mode reg: see AN87 2-2 -- 2 bit ID
#if 0
    struct {
        unsigned mode_a:3;  // online, test, or offline
        unsigned bdry_a:3;  // size of memory
        unsigned mode_a:3;
        unsigned bdry_b:3;
        unsigned interlace:1;
        unsigned lwr:1;     // controls whether A or B is low order memory
        unsigned addr_offset:2;
        unsigned port_no:4; // port from which rscr instr was received
        struct {
            unsigned flag:2;
        } port_enabled[8];
        struct {
            unsigned value:9;
        } pima[4];      // each bit indicates an assigned port (matches 4 rotary switches [initally?])
    } config_switches;
#endif
    t_uint64 masks[8];  // port assignements
    uint ports[8];  // CPU/IOM connectivity; designated 0..7; negative to disable
    int mask_assign[4]; //  Bit masks.  Which port(s) is each PIMA reg assigned to?
    
} scu_t;

// ============================================================================
// === Operations on 36-bit pseudo words
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
static inline t_uint64 getbits36(t_uint64 x, int i, int n) {
    // bit 35 is right end, bit zero is 36th from the left
    return (x >> (35-i-n+1)) & ~ (~0 << n);
}
static inline t_uint64 setbits36(t_uint64 x, int p, int n, t_uint64 val)
{
    // return x with n bits starting at p set to n lowest bits of val 
    // return (x & ((~0 << (p + 1)) | (~(~0 << (p + 1 - n))))) | ((val & ~(~0 << n)) << (p + 1 - n));

    t_uint64 mask = ~ (~0<<n);  // n low bits on
        // 1 => ...1
        // 2 => ..11
    mask <<= 36 - p - n;    // shift 1s to proper position; result 0*1{n}0*
            // 0,1 => 35 => 1000... (35 zeros)
            // 1,1 => 34 => 0100... (34 z)
            // 0,2 = 34 => 11000 (34 z)
            // 1,2 = 33 => 011000 (33 z)
            // 35,1 => 0 => 0001 (0 z)
    return (x & ~ mask) | (val << (36 - p - n));
}


// #define bit36_is_neg(x) ( ((x)>>35) == 1 )
#define bit36_is_neg(x) (((x) & (((t_uint64)1)<<35)) != 0)
#define bit27_is_neg(x) (((x) & (((t_uint64)1)<<26)) != 0)
#define bit18_is_neg(x) (((x) & (((t_uint64)1)<<17)) != 0)

// obsolete typedef -- hold named register sub-fields in their inefficient
// native format.
/* #define CU_PPR_P(CU) (bitval36(CU.word0bits, 18)) */

// ============================================================================
// === Variables

extern t_uint64 reg_A;      // Accumulator, 36 bits
extern t_uint64 reg_Q;      // Quotient, 36 bits
extern t_uint64 reg_E;      // Quotient, 36 bits
extern t_uint64 reg_X[8];   // Index Registers, 18 bits
extern IR_t IR;             // Indicator Register
extern t_uint64 reg_TR;     // Timer Reg, 27 bits -- only valid after calls to SIMH clock routines
extern PPR_t PPR;           // Procedure Pointer Reg, 37 bits, internal only
extern TPR_t TPR;           // Temporary Pointer Reg, 42 bits, internal only
extern PTWAM_t PTWAM[16];   // Page Table Word Associative Memory, 51 bits
extern SDWAM_t SDWAM[16];   // Segment Descriptor Word Associative Memory, 88 bits

extern ctl_unit_data_t cu;
extern cpu_state_t cpu;

// ============================================================================
// === Functions

extern void debug_msg(const char* who, const char* format, ...);
extern void complain_msg(const char* who, const char* format, ...);
extern void execute_instr(void);
extern void fault_gen(enum faults);
extern int decode_addr(instr_t* ip, t_uint64* addrp);
extern int decode_ypair_addr(instr_t* ip, t_uint64* addrp);
extern int fetch_instr(uint IC, instr_t *ip);
extern char *bin2text(t_uint64 word, int n);

int fetch_word(uint addr, t_uint64 *wordp);
int fetch_pair(uint addr, t_uint64* word0p, t_uint64* word1p);
int store_word(uint addr, t_uint64 word);

void set_addr_mode(addr_modes_t mode);
addr_modes_t get_addr_mode(void);

// ============================================================================

#include "opcodes.h"
