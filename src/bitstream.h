/*
    Provides a library for reading chunks of an arbitrary number
    of bits from a stream.
*/
/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

typedef struct {
    const unsigned char *p;
    const unsigned char *head;
    const char *fname;
    int used;
    size_t len;
    int fd;
    unsigned char byte;
} bitstream_t;

extern bitstream_t* bitstm_open(const char* fname);
extern bitstream_t* bitstm_new(const unsigned char* addr, uint32 len);
extern int bitstm_get(bitstream_t *bp, size_t len, t_uint64 *word);
extern int bitstm_destroy(bitstream_t *bp);
