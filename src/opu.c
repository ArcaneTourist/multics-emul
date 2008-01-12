#include "hw6180.h"

extern ctl_unit_data_t cu;
extern cpu_state_t cpu;

static int do_op(instr_t *ip);

void execute_instr(void)
{
    // execute whatever instruction is in the IR (not whatever the IC points at)
    do_op(&cu.IR);
}

static int do_op(instr_t *ip)
{
    // Returns non-zero on error or non-group-7  fault

    uint op = ip->opcode;
    char *opname = opcodes2text[op];

    int bit27 = op % 2;
    op >>= 1;
    if (opname == NULL) {
        debug_msg("OPU", "Illegal opcode 0%0o(%d)\n", op, bit27);
        fault_gen(illproc_fault);
        return 1;
    }
    
    if (bit27 == 0) {
        switch (op) {
            case opcode_xed: {
                // todo: re-implement via setting flags and return to control_unit()
                // todo: fault if xed invokes xed
                // todo: handle rpd repeats
                t_uint64 word0;
                t_uint64 word1;
                t_uint64 addr;
                instr_t IR;
                if (decode_ypair_addr(ip, &addr)) {
                    debug_msg("OPU::opcode::xed", "decode addr: error or fault\n");
                    return 1;   // faulted
                }
                // -----------
                if (fetch_instr(addr, &IR) != 0) {
                    debug_msg("OPU::opcode::xed", "fetch even: error or fault\n");
                    return 1;   // faulted
                }
                debug_msg("OPU::opcode::xed", "executing even instr at %ld\n", (long) addr);
                if (do_op(&IR) != 0) {
                    debug_msg("OPU::opcode::xed", "fault or error executing even instr\n");
                    return 1;
                }
                // -----------
                if (cpu.xfr) {
                    debug_msg("OPU::opcode::xed", "transfer instr executed, not doing odd instr\n");
                } else {
                    ++ addr;
                    if (fetch_instr(addr, &IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fetch odd: error or fault\n");
                        return 1;   // faulted
                    }
                    debug_msg("OPU::opcode::xed", "executing odd instr at %ld\n", (long) addr);
                    if (do_op(&IR) != 0) {
                        debug_msg("OPU::opcode::xed", "fault or error executing odd instr\n");
                        return 1;
                    }
                }
                debug_msg("OPU::opcode::xed", "finished\n");
                break;
            }
            default:
                debug_msg("OPU", "Unimplemented opcode 0%0o(0)\n", op);
                fault_gen(oob_fault);   // todo: mechanism to bomb back to simh
        }
    } else {
        switch (op) {
            default:
                debug_msg("OPU", "Unimplemented opcode 0%0o(1)\n", op);
                fault_gen(oob_fault);   // todo: better mechanism to bomb back to simh
        }
    }
}
