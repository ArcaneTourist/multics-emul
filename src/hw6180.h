#include "sim_defs.h"

/* For efficiency, we use full ints instead of bit fields for the flags
   and other field of most of the register typdefs
*/


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
    uint32 zero;                /* 1 bit at 18 */
    /* ... */
    uint32 not_bar_mode;        /* 1 bit at 28 */
    /* ... */
    uint32 mid_instr_intr_fault;/* 1 bit at 30 */
    uint32 abs_mode;            /* 1 bit at 31 */
} IR_t;


/* Fault register (72 bits) */
typedef struct {
    unsigned int ill_op:1;      /* 1 bit at 0 */
    unsigned int ill_mod:1;     /* 1 bit at 1 */
    unsigned int ill_slv:1;     /* 1 bit at 2 */
    unsigned int ill_proc:1;    /* 1 bit at 3 */
    /* ... */
} fault_reg_t;

/* Control unit data (288 bits) */
typedef struct {
    /*    This is a collection of flags and registers from the
       appending unit and the control unit.  The scu and rcu
       instructions store and load these values to an 8 word
       memory block.
          Ints are mostly used here.   Comments indicate format as
       stored in 8 words by the scu instruction.
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
#define bitval36(word,i) ( ((word)>>(35-i)) & 1 )

/* #define CU_PPR_P(CU) (bitval36(CU.word0bits, 18)) */
    rightmost bit. */
