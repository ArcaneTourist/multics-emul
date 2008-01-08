#include "sim_defs.h"

/*
        For efficiency, we mostly use full ints instead of bit fields for
    the flags and other fields of most of the typedefs here.  When necessary,
    such as when an instruction saves a control register to memory, the more
    efficient variables are copied into appropriate bit fields.
        Also note that we only represent selected fields of some of the
    registers.  The simulator doesn't need some of the various scratches
    and the OS doesn't need them either.
*/


/* Misc enum typdefs */

typedef enum { ABSOLUTE_mode, APPEND_mode, BAR_mode } addr_modes_t;
typedef enum { NORMAL_mode, PRIV_mode } instr_modes_t;


/* Format of a 36 bit instruction word */
typedef struct {
    /* uint32 addr; /* 18 bits at 0..17 */
    uint32 offset;  /* 18 bits at 0..17 */
    uint32 opcode;  /* 10 bits at 18..27 */
    uint32 inhibit; /* 1 bit at 28 */
    uint32 pr_bit;  /* 1 bit at 29 */
    uint32 tag;     /* 6 bits at 30..35 */
} instr_t;


/* Indicator register */
typedef struct {
    uint64 zero;                /* 1 bit at 18 */
    /* ... */
    uint64 not_bar_mode;        /* 1 bit at 28 */
    /* ... */
    uint64 mid_instr_intr_fault;/* 1 bit at 30 */
    uint64 abs_mode;            /* 1 bit at 31 */
} IR_t;


/* Fault register (72 bits) */
typedef struct {
    // Multics never examines this (just the CPU) -- multicians.org glossary
    unsigned int ill_op:1;      /* 1 bit at 0 */
    unsigned int ill_mod:1;     /* 1 bit at 1 */
    unsigned int ill_slv:1;     /* 1 bit at 2 */
    unsigned int ill_proc:1;    /* 1 bit at 3 */
    /* ... */
} fault_reg_t;


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

    /* word 0 */
    uint32 PPR_PRR;     /* Procedure ring register; 3 bits @ 0[0..2] */
    uint32 PPR_PSR;     /* Procedure segment register; 15 bits @ 0[3..17] */
    uint32 PPR_P;       /* Privileged bit; 1 bit @ 0[18] */
    /* uint64 word0bits;    /* Word 0, bits 18..32 */
    uint32 FCT;         /* Fault counter; 3 bits at 0[33..35];

    /* word 1 */
    uint64 word1bits;   /* Word1, bits [0..19] and [35] */
    uint32 IA;          /* 4 bits @ 1[20..23] */
    uint32 IACHN;       /* 3 bits @ 1[24..26] */
    uint32 CNCHN;       /* 3 bits @ 1[27..29] */
    uint32 FIADDR       /* 5 bits @ 1[30..34] */

    /* word 2 */
    /* word 3 */
    /* word 4 */
    /* word 5 */

    /* word 6 */
    instr_t IR;     /* Working instr register; addr & tag are modified */

    /* word 7 */
    instr_t IRODD;  /* Instr holding register; odd word of last pair fetched */
    
} ctl_unit_data_t;


/*  Extract (i)th bit of a 36 bit word (held in a uint64).  Bit 35 is the
    rightmost bit. */
#define bitval36(word,i) ( ((word)>>(35-i)) & (uint64_t) 1 )
/*  Value of a 36bit word (held in a uint64) after setting or clearing the
    the (i)th bit. */
#define bitset36(word,i) ( (word) | ( (uint64_t) 1 << (35 - i)) )
#define bitclear36(word,i) ( (word) & ~ ( (uint64_t) 1 << (35 - i)) )

// obsolete typedef -- hold named register sub-fields in their inefficient
// native format.
/* #define CU_PPR_P(CU) (bitval36(CU.word0bits, 18)) */
