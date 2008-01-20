#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "hw6180.h"
#include "bits.h"
DEVICE cpu_dev; // hack

void anal36 (const char* tag, t_uint64 word);
char *bin(t_uint64 word, int n);
#include <ctype.h>

int bootimage_loaded;
void tape_block(unsigned char *p, uint32 len);
void decode_instr(instr_t *ip, t_uint64 word);

static t_uint64 gbits36(t_uint64 x, int i, int n) {
    // bit 35 is right end, bit zero is 36th from the left
    printf("getbits(%012Lo,%d,%d): ", x, i, n);
    t_uint64 shifted = (x >> (35-i-n+1));
    t_uint64 mask = ~ (~0<<n);
    t_uint64 result = shifted & mask;
    printf("%012Lo & %012Lo = %12Lo\n", shifted, mask, result);
    // return (x >> (35-i+n-1)) & ~ (~0 << n);
    return result;
}


int main()
{
    t_uint64 word;
    if (sizeof(word) != 8) {
        fprintf(stderr, "t_uint64 is not 8 bytes\n");
    }

#if 0
    word = 061062056065;
    anal36 ("TEST", word);
    printf("\n");
    tbit(word, "offset", 0, 18);
    tbit(word, "opcode", 18, 10);
    tbit(word, "i", 28, 1);
    tbit(word, "pr", 29, 1);
    tbit(word, "tag", 30, 6);
exit(1);
#endif

    doit();
    return 0;
}

int tbit(t_uint64 word, const char *tag, int pos, int nbit)
{
    t_uint64 result = gbits36(word, pos, nbit);
    printf("%s: bits %d..%d: %Lo\n",
        tag, pos, pos+nbit-1, result);
}

int doit()
{
    // Boot -- Copy bootstrap loader into memory & set PC (actually send startup fault)
    // Issue: have to specify boot tape or file out-of-band

    // quick & dirty

    // from the web site:
    //      tape image format:
    //      <32 bit little-endian blksiz> <data> <32bit little-endian blksiz>
    //      a single 32 bit word of zero represents a file mark
    // question
    //      What type of data does blksiz measure?  Count of 32bit words?  Count of 9 bit "bytes" ?
    
    char *fname = "boot.tape";
    printf("Loading file %s\n", fname);
    int fd;
    if ((fd = open(fname, O_RDONLY)) == -1) {
        perror(fname);  // BUG
        return STOP_BUG;    // todo: better code
    }
    uint addr = 0;
    uint nblocks = 0;
    int read_unit = 8;  // 8, 9, 18, 32, 36, 288, 9216, etc
    int invert = 0;
    uint totread = 0;
    for(;;) {
        ssize_t nread;
        unsigned char word[4];
        uint32 n;
        uint32 nalt;
        // stats
        // printf ("%u bytes read (incl %d 32bit counts)\n", totread, nblocks);
        // read 32bit count
        int i;
        if (1) {
            if ((nread = read(fd, word, 4)) != 4) {
                if (nread == 0) {
                    printf("EOF at index %d\n", i);
                    close(fd);
                    return STOP_BUG;    // BUG: bogus, but prevents "go"
                    return 0;
                } else {
                    perror("begin count read"); // BUG
                    close(fd);
                    return STOP_BUG;
                }
            }
            n = (word[0]) | (word[1] << 8) | (word[2] << 16) | (word[3] << 24);
            nalt = (word[3]) | (word[2] << 8) | (word[1] << 16) | (word[0] << 24);
        } else {
            if ((nread = read(fd, &n, 4)) != 4) {
                perror("count read");   // BUG
                close(fd);
                return STOP_BUG;
            }
        }
        ++ nblocks;
        totread += 4;
        if (n == 0) {
            printf("record mark found\n");
            continue;
        }
        // convert count
#if 0
        if (read_unit == 36) {
            if ((n * 36) % 32 == 0) {
                printf("Block size %d is a multiple of 36/32\n", n);
                n = n * 36 / 32;
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 36/8\n", (unsigned long) n, (unsigned long) n);
                close(fd);
                return STOP_BUG;
            }
        }
#endif
        if (invert) {
            if ((n * 8) % read_unit  == 0) {
                uint32 nold = n;
                n = n * 8 / read_unit;
                printf("Block size %u is a multiple of 8/%d ==> %u\n", nold, read_unit, n);
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 8/%d\n", (unsigned long) n, (unsigned long) n, read_unit);
                close(fd);
                return STOP_BUG;
            }
        } else {
            if ((n * read_unit) % 8  == 0) {
                uint32 nold = n;
                n = n * read_unit / 8;
                // printf("Block size %u is a multiple of %d/8 ==> %u\n", nold, read_unit, n);
            } else {
                printf("Block size %lu (oct %lo) is *not* a multiple of 8\n", (unsigned long) n, (unsigned long) n);
                close(fd);
                return STOP_BUG;
            }
        }
        // read block
        printf("Start read block of %lu bytes for %d-bit units\n", (unsigned long) n, read_unit);
        if(1) {
            unsigned char *bufp = malloc(n);
            if (bufp == NULL) {
                perror("malloc");
                close(fd);
                return STOP_BUG;
            }
            if ((nread = read(fd, bufp, n)) != n) {
                if (nread < 0) {
                    perror("block read");
                } else if (nread == 0) {
                    printf("Unexpected EOF\n");
                } else {
                    printf("Short read of %u bytes (expecting %u)\n", nread, n);
                }
                close(fd);
                return STOP_BUG;
            }
            // use bytes
            tape_block(bufp, n);
            bootimage_loaded = 1;
            // Only read one record
            // return STOP_BUG;
        } else {
        for (i = 0; i < n; ++i) {
            ++addr;
            unsigned char byte;
            uint32 word;
            int err;
            err = (nread = read(fd, &byte, 1)) != 1;
            if (!err) ++ totread;
            if (err) {
                if (nread == 0) {
                    printf("EOF at index %d\n", i);
                } else {
                    perror("byte read");    // BUG
                    printf("error at byte %d; return was %d\n", i, nread);
                }
                close(fd);
                return STOP_BUG;
            }
        }
        }
        if ((nread = read(fd, word, 4)) != 4) {
            perror("end-of-block count read");  // BUG
            close(fd);
            return STOP_BUG;
        }
        uint32 n2 = (word[0]) | (word[1] << 8) | (word[2] << 16) | (word[3] << 24);
        if (n == n2) {
            // printf("End block count %d matches begin block count\n", n2);
        } else {
            printf("End block count %d does not matche begin block count %u.\n", n2, n);
            close(fd);
            return STOP_BUG;
        }
    }
    return 0;
}

//=============================================================================


#define force030 1

void tape_block(unsigned char *p, uint32 len)
{
    size_t hack = 0;
    bitstream_t *bp = bitstm_new(p, len);
    if ((len * 8) % 36 != 0) {
        complain_msg("CPU::boot", "Length %u bytes is not a multiple of 36 bits.\n");
    }
    printf("=============================================================\n");
    printf("Tape block: %u bytes, %u 36-bit words\n", len, len*8/36);
    if (force030)
        hack = 030;
    else
        hack = 0;   // per block
    uint32 nbits = len * 8;
    while (nbits >= 36) {
        t_uint64 word;
        bitstm_get(bp, 36, &word);
        char msg[80];
        sprintf(msg, "WORD %lo (%ld dec)", hack, hack);
        ++hack;
        anal36(msg, word);
        // bitstm_get(bp, 36, &M[hack++]);
        nbits -= 36;
    }
    printf("\n");
    if (nbits != 0) {
        complain_msg("CPU::boot", "Internal error getting bits from tape\n");
    }
}

void anal36 (const char* tag, t_uint64 word)
{
    t_uint64 magic;
    magic = (unsigned long long) 0670314355245;
    unsigned char nines[4];
    nines[3] = word & 0777;
    nines[2] = (word >> 9) & 0777;
    nines[1] = (word >> 18) & 0777;
    nines[0] = (word >> 27) & 0777;
    // printf("%s: %012Lo octal, %Lu decimal\n", tag, word, word);
    printf("%s: %012Lo octal", tag, word);
    if (word == magic) {
        printf(" <MAGIC>");
    }
    printf("\n");
    instr_t instr;
    decode_instr(&instr, word);
    const char *opname = opcodes2text[instr.opcode];
    if (opname == NULL)
        printf("instr: <none>\n");
    else  {
        char o[80];
        if (instr.offset == 0)
            sprintf(o, "zero");
        else
            sprintf(o, "%d decimal", instr.offset);
        printf("instr: %s, offset %016o (%s); inhibit %d, pr %d, tag %02u\n",
            opname, (unsigned) instr.offset, o, instr.inhibit, instr.pr_bit, instr.tag);
    }
    //printf("bin64: %s\n", bin(word, 64));
    //printf("bin36: %s\n", bin(word, 36));
    //printf("9bits(oct): %03o %03o %03o %03o\n", nines[0], nines[1], nines[2], nines[3]);
    printf("9bits(ascii):");
    int i;
    for (i = 0; i < 4; ++ i) {
        if (isprint(nines[i])) {
            printf(" '%c'", nines[i]);
        } else {
            printf(" \\%03o", nines[i]);
        }
    }
    printf("\n");
}

char *bin(t_uint64 word, int n)
{
    static char str[65];
    str[n] = 0;
    int i;
    for (i = 0; i < n; ++ i) {
        str[n-i-1] = ((word % 2) == 1) ? '1' : '0';
        word >>= 1;
    }
    return str;
}

void decode_instr(instr_t *ip, t_uint64 word)
{
    ip->offset = getbits36(word, 0, 18);
    ip->opcode = getbits36(word, 18, 10);
    ip->inhibit = getbits36(word, 28, 1);
    ip->pr_bit = getbits36(word, 29, 1);
    ip->tag = getbits36(word, 30, 6);
}

