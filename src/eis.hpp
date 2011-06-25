#ifdef __cplusplus
extern "C" {
#endif

/*
 * "C" interfaces to C++ objects
 *
 * This hack should be removed...
 */

// TODO: Does OPU need to force modulo 64 for register derived lengths (which might otherwise be over 6 bits) ?
// Might be better to do via parse_eis_*_desc()

// We probably need to get rid of this and use desc_t's derived types (FIXME)
typedef struct {
    void *dummyp;
    void *objp;
    int n;
    int nbits;
#if 1
    // The following is also in num_desc_t...
    struct {
        int s;      // sign and type: 00b floating with leading sign; 01b-11b scaled fixed point, 01 leading sign, 10 trailing, 11 unsigned
        int scaling_factor;
    } num;
#endif
} eis_desc_t;

extern int decode_eis_alphanum_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd);
extern int decode_eis_bit_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd);
extern int decode_eis_num_desc(eis_desc_t* descp, const eis_mf_t* mfp, t_uint64 word, int is_read, int is_fwd);
extern const char* eis_desc_to_text(const eis_desc_t* descp);

extern void eis_desc_mod64(eis_desc_t *descp);

extern int eis_desc_get(eis_desc_t* descp, unsigned* valp);
extern int eis_desc_val(eis_desc_t* descp, unsigned* valp);
extern int eis_desc_put(eis_desc_t* descp, unsigned val);
extern int eis_desc_flush(eis_desc_t*);

#ifdef __cplusplus
}
#endif



#ifdef __cplusplus

/*
 * A pointer into main memory.  Pointers used in EIS descriptors are 18-bit
 * values that are either an 18-bit offset against the current segment or
 * are a 3-bit "PR" register number and a 15-bit offset against that PR.
 * Which of the two interpretations should be used is controlled by the "ar"
 * flag.
 */

class ptr_t {
private:
    // For compability with prior debug messages we defer real initialization
    struct mf_y_addr_t {
        bool ar;
        int reg;
        int width;
        unsigned y;
        // mf_y_addr_t() { ar = 0; reg = 0; width = 0; y = 0; }
        // mf_y_addr_t() {}
    } initial;
    // FIXME: rid of scheme with two seperate offsets but a
    // single merged bitno.  Let caller copy out the base and/or
    // have two objects.
    struct pr_info_t {
        int init(bool ar, unsigned reg, int width, unsigned y);
        bool _init;
        uint ringno;
        uint segno;
        // FIXME: we should make caller track original offset and bitno
        uint offset;
        uint bitno;
        pr_info_t() { _init = 0; }
    } base;
    struct page_t {
        bool _valid;
        unsigned addr;  // 24-bit main memory address
        unsigned lo;
        unsigned hi;
        int _offset;    // offset used to generate addr member
        int valid() const { return _valid && addr >= lo && addr <= hi; }
        // int valid(int offset) const;
        page_t() { _valid = 0; _offset = 0; }
    } page;
    void _bit_advance(int nbits, bool quiet);
public:
    ptr_t() {}
    ptr_t(bool ar, unsigned reg, int nbits, unsigned y) {
        set(ar, reg, nbits, y);
    }
    void set(bool ar, unsigned reg, int nbits, unsigned y);
    int init();
    int valid() const { return page.valid(); }
    void bit_advance(int nbits) { _bit_advance(nbits,0); }
    void char_advance(int nchars);
    void word_advance(int nwords);  // positions to bit zero of requested word
    int addr() const { return page.valid() ? (int) page.addr : -1; }
    int min() const { return page.valid() ? (int) page.lo : -1; }
    int max() const { return page.valid() ? (int) page.hi : -1; }
    int _addr() const { return page.addr; }
    int _min() const { return page.lo; }
    int _max() const { return page.hi; }
    unsigned _bitnum;
    int bitno() const { return page.valid() ? (int) _bitnum : -1; }
    int _bitno() const { return _bitnum; }
    int char_charno() const { return page.valid() ? (int) _bitnum / 9 : -1; }
    int char_bitno() const { return page.valid() ? (int) _bitnum % 9 : -1; }
    int get();
    // int get(unsigned* addrp, unsigned* bitnop, unsigned* minp, unsigned* maxp);
};

/*
 * class desc_t
 *
 * Abstract base class
 *
 */

class desc_t {
protected:
    eis_mf_t _mf;
    int _addr;          // original raw address
    int _width;         // "character" size in bits
    unsigned _count;    // initial number of characters (pre MF fixup)
    unsigned _n;                // current character count
    struct {
        int cn; 
        int bitno;
    } first_char;
    int ta() const {
        // not valid for width 1
        return (_width == 9) ? 0 : (_width == 6) ? 1 : (_width == 4) ? 2 : 7;
    }
    void len_to_text(char *bufp) const;
    void yaddr_to_text(char *bufp) const;
private:
    void *dummyp;
    int ptr_init;
    // t_uint64 buf[8]; // EIS instructions use a buffer
    // ptr_t _base;
    ptr_t _curr;
    // bool _is_read;
    bool _is_fwd;
    //bool _mod64;
    struct {
        // we read/write a word at a time
        t_uint64 word;
        bool is_loaded;
        int lo_write;
        int hi_write;
    } buf;
    int _get(unsigned* valp, bool want_advance);
    int _put(unsigned val, bool want_advance);
public:
    // desc_t(); -- no constructor for abstract base class; use init()
    void init(const eis_mf_t& mf, int y_addr, int width, int cn, int bit_offset, int nchar, int is_fwd);
    void mod64() {      // adjust length to be modulo 64
        _n %= 64; /* _mod64 = 1 */; }
    int n() const { return _n; }
    int width() const { return _width; }
    int put(unsigned val)       // write, advance forward or backwards
        { return _put(val, 1); }
    int set(unsigned val)       // write, no advance
        { return _put(val, 0); }
    int get(unsigned* valp)     // read, advance forward or backwards
        { return _get(valp, 1); }
    int val(unsigned* valp)     // read, no advance
        { return _get(valp, 0); }
    int flush(int verbose = 1);         // write buffer to main memory
    virtual char* to_text(char *buf) const = 0;
    int init_ptr();
    int valid() const { return _curr.valid(); }
};

class alpha_desc_t : public desc_t {
public:
    alpha_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd);
    char* to_text(char *buf) const;
};

class bit_desc_t : public desc_t {
public:
    bit_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd);
    char* to_text(char *buf) const;
};

class num_desc_t : public desc_t {
private:
    int _s;     // sign and type: 00b floating with leading sign; 01b-11b scaled fixed point, 01 leading sign, 10 trailing, 11 unsigned
    int _sf;    // scaling factor
public:
    num_desc_t(const eis_mf_t& mf, t_uint64 word, int is_fwd);
    // num_desc_t(const eis_mf_t& mf, int addr, int width, int cn, int bit_offset, int nchar, int is_read, int is_fwd, int stype, int sf);

    char* to_text(char *buf) const;
    int sf() const { return _sf; }
    int s() const { return _s; }
    int ndigits() const {
        // Returns number of coefficient aka "integer" digits
        int ncoe = n();
        if (s() != 3) ncoe -= 1;    // sign byte 
        if (s() == 0) ncoe -= (width() == 4) ? 2 : 1;  // exp byte(s)
        return ncoe;
    }
};

#endif // __cplusplus
