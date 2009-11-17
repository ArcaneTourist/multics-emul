/*
    Provides a library for reading chunks of an arbitrary number
    of bits from a stream.

    NOTE: It was later discovered that the emulator doesn't need
    this generality.   All known "tape" files are multiples of
    72 bits, so the emulator could simply read nine 8-bit bytes at
    a time which would yield two 36-bit "words".

    WARNING: uses mmap() and mumap() which may not be available on
    non POSIX systems.  We only read the input data one byte at a time,
    so switching to ordinary file I/O would be trivial.

*/

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>

#include "sim_defs.h"
#include "bitstream.h"

//=============================================================================

/*
    bitstm_create() -- allocate and initialize an empty bitstream_t object.
*/

static bitstream_t* bitstm_create()
{

    bitstream_t* bp;
    if ((bp = malloc(sizeof(*bp))) == NULL) {
        perror("malloc");
        return NULL;
    } 
    memset(bp, 0, sizeof(*bp));

    bp->fd = -1;
    bp->used = 8;   // e.g., none left
    return bp;
}

//=============================================================================

/*
    bitstm_new() -- Create a bitstream_t object tied to the given source buffer.
*/

bitstream_t* bitstm_new(const unsigned char* addr, uint32 len)
{

    bitstream_t* bp;
    if ((bp = bitstm_create()) == NULL)
        return NULL;
    
    bp->len = len;
    bp->p = addr;
    bp->head = addr;
    return bp;
}


//=============================================================================


/*
    bitstm_open() -- Create a bitstream_t object and tie a file to it.
*/

bitstream_t* bitstm_open(const char* fname)
{
    bitstream_t *bp;

    if ((bp = bitstm_create()) == NULL)
        return NULL;
    if ((bp->fname = strdup(fname)) == NULL) {
        perror("strdup");
        return NULL;
    }
    if ((bp->fd = open(fname, O_RDONLY)) == -1) {
        perror(fname);
        return NULL;
    }
    struct stat sbuf;
    if (stat(fname, &sbuf) != 0) {
        perror(fname);
        return NULL;
    }
    bp->len = sbuf.st_size;

    void* addr = mmap(0, bp->len, PROT_READ, MAP_SHARED|MAP_NORESERVE, bp->fd, 0);
    if (addr == NULL) {
        perror("mmap");
        return NULL;
    }
    bp->p = addr;
    bp->head = addr;
    return bp;
}

//=============================================================================

/*
    bitstm_get() -- Extract len bits from a stream

    Returns non-zero if unable to provide all of the
    requested bits.  Note that this model doesn't
    allow the caller to distinguish EOF from partial
    reads or other error conditions.
*/

int bitstm_get(bitstream_t *bp, size_t len, t_uint64 *word)
{

    *word = 0;
    size_t orig_len = len;
    int used = bp->used;
    int left = 8 - used;
    if (left != 0 && len < left) {
        // We have enough bits left in the currently buffered byte.
        uint val = bp->byte >> (8-len); // Consume bits from left of byte
        *word = val;
        //printf("b-debug: used %d leading bits of %d-bit curr byte to fufill small request.\n", len, left);
        bp->byte = bp->byte << len;
        bp->used += len;
        goto b_end;
    }

    t_uint64 wtmp;

    // Consume remainder of curr byte (but it's not enough)
    if (left != 0) {
        wtmp = bp->byte >> (8-left);
        len -= left;
        //printf("b-debug: using remaing %d bits of curr byte\n", left);
        bp->used = 8;
        bp->byte = 0;
    } else
        wtmp = 0;

    // Consume zero or more full bytes
    int i;
    for (i = 0; i < len / 8; ++ i) {
        //printf("b-debug: consuming next byte %03o\n", *bp->p);
        if (bp->p == bp->head + bp->len) {
            fflush(stdout); fflush(stderr);
            fprintf(stderr, "bits.c: bit stream exhausted\n");  // BUG: remove text msg
            return 1;
        }
        // left shift in next byte
        wtmp = (wtmp << 8) | (*bp->p);
        ++ bp->p;
    }

    // Consume one partial byte if needed (buffer the leftover bits)
    int extra = len % 8;
    if (extra != 0) {
        //printf("b-debug: consuming %d bits of next byte %03o\n", *bp->p);
        if (bp->p == bp->head + bp->len) {
            fflush(stdout); fflush(stderr);
            fprintf(stderr, "bits.c: bit stream exhausted\n");  // BUG: remove text msg
            return 1;
        }
        bp->byte = *bp->p;
        ++ bp->p;
        uint val = bp->byte >> (8-extra);
        wtmp = (wtmp << extra) | val;
        //printf("b-debug: used %d leading bits of curr byte to finish request.\n", extra);
        bp->byte = bp->byte << extra;
        bp->used = extra;
    }
    *word = wtmp;

b_end:
    //printf("b-debug: after %u req: offset=%u, %d curr bits used, return = %lu\n",
    //  orig_len, bp->p - bp->head, bp->used, (unsigned long) *word);
    return 0;
}

//=============================================================================

/*
    Free all memory associated with given bitstream.
    Returns non-zero on error.
*/

int bitstm_destroy(bitstream_t *bp)
{
    if (bp == NULL)
        return -1;

    int err = 0;
    if (bp->fd >= 0) {
        if (bp->p != NULL) {
            err |= munmap((void*)bp->p, bp->len);   // WARNING: casting away const
        }
        err |= close(bp->fd);
    }
    free((void*) bp->fname);
    free(bp);
    return err;
}
