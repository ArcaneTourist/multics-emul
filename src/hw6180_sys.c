#include "hw6180.h"

extern DEVICE cpu_dev;
extern DEVICE dsk_dev;
extern UNIT cpu_unit;
extern REG cpu_reg[];
extern DEVICE sio_dev;
extern DEVICE ptr_dev;
extern DEVICE ptp_dev;
extern DEVICE lpt_dev;
extern t_uint64 M[];

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
#if 0
    &sio_dev,
    &ptr_dev,
    &ptp_dev,
    &dsk_dev,
#endif
    NULL
};

const char *sim_stop_messages[] = {
    "Memory is empty",
    "BUG-STOP -- Internal error",
    "Fetch on Odd address",
    "Breakpoint",
    // "Invalid Opcode"
};


// One-time-only initialization for emulator
static void hw6180_init(void);
void (*sim_vm_init)(void) = hw6180_init;

/*
    Variables custom to HW6180 that aren't normally present in other
    SIMH emulators.
*/
int bootimage_loaded = 0;

/*  SIMH binary loader.
        The load normally starts at the current value of the PC.
    Args
        fileref -- file opened by SIMH
        cptr -- VM specific args (from cmd line?)
        fnam -- filename
        write_flag -- indicates whether to load or write
    
*/

t_stat sim_load (FILE *fileref, char *cptr, char *fnam, int32 write_flag)
{
    /*  Maybe:
            This emulator will load boot tape format files starting
            at location 30 as an emulation of the physical boot loaders.
        Or:
            Define a boot procedure so the user can use the "boot" command.
            However, the boot command may not allow specification of a
            command.
    */
    bootimage_loaded = 1;
    abort();
}


static void hw6180_init(void)
{
    debug_msg("SYS::init", "Once-only initialization running.\n");
    // todo: sim_brk_types = ...
    // todo: sim_brk_dflt = ...

}

extern t_stat fprint_sym (FILE *ofile, t_addr addr, t_value *val, UNIT *uptr, int32 sw)
{
    abort();
}

extern t_stat parse_sym (char *cptr, t_addr addr, UNIT *uptr, t_value *val, int32 sw)
{
    abort();
}
