#include "hw_6180.h"

extern DEVICE cpu_dev;
extern DEVICE dsk_dev;
extern UNIT cpu_unit;
extern REG cpu_reg[];
extern DEVICE sio_dev;
extern DEVICE ptr_dev;
extern DEVICE ptp_dev;
extern DEVICE lpt_dev;
extern t_uint64 M[];
extern int32 saved_PC;

/* SCP data structures

   sim_name             simulator name string
   sim_PC               pointer to saved PC register descriptor
   sim_emax             number of words needed for examine
   sim_devices          array of pointers to simulated devices
   sim_stop_messages    array of pointers to stop messages
   sim_load             binary loader
*/

char sim_name[] = "hw6180";

REG *sim_PC = &cpu_reg[0];

int32 sim_emax = 4;

/* Multics allows up to 7 CPUs... */

DEVICE *sim_devices[] = {
    &cpu_dev,
    &sio_dev,
    &ptr_dev,
    &ptp_dev,
    &dsk_dev,
    NULL
};

const char *sim_stop_messages[] = {
    "Unknown error",
    "Unknown I/O Instruction",
    "HALT instruction",
    "Breakpoint",
    "Invalid Opcode"
};

/* This is the binary loader.
   The load starts at the current value of the PC.
*/

t_stat sim_load (FILE *fileref, char *cptr, char *fnam, int flag)
{
    abort();
}

