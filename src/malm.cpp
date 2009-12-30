#include <string>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <sstream>
#include <errno.h>
using namespace std;

#include "hw6180.h"
#include "malm.h"

extern "C" {
extern FILE* yyin;
// #include "opcodes.h"
extern int yyparse(void);
extern void init_opcodes();
extern void yyerror(const char* msg);
}

struct code_t {
    string operand;
    instr_t instr;
};
int any_parse_errors;
static const char* cmd_name;
static map<string,int> label_addrs;
static multimap<string,code_t*> label_refs;
static vector<code_t> code;
static int fix_labels(void);
static void generate(char *src);
static int write72(FILE* fp, const string& path, t_uint64 word0, t_uint64 word1);

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(*(a)))
#endif

int main(int argc, char* argv[])
{
    cmd_name = strrchr(argv[0], '/');
    if (cmd_name == NULL)
        cmd_name = strrchr(argv[0], '\\');
    if (cmd_name == NULL)
        cmd_name = argv[0];
    else
        ++cmd_name;

    if (argc != 2) {
        fprintf(stderr, "USAGE: %s <filename.alm>\n", cmd_name);
        exit(1);
    }
    char* src = argv[1];
    char* suffix = strrchr(src, '.');
    if (suffix == NULL || strcmp(suffix, ".alm") != 0) {
        fprintf(stderr, "%s: Filename must end in '.alm'\n", cmd_name);
        exit(1);
    }
    yyin = fopen(src, "r");
    if (! yyin) {
        fprintf(stderr, "%s: Cannot read %s: %s\n", cmd_name, src, strerror(errno));
        exit(1);
    }
    any_parse_errors = 0;
    init_opcodes();
    int ret = yyparse();
    if (ret != 0 || any_parse_errors) {
        fprintf(stderr, "%s: translation aborted.\n", cmd_name);
        exit(1);
    }
    if (fclose(yyin) != 0) {
        fprintf(stderr, "%s: Error reading %s: %s\n", cmd_name, src, strerror(errno));
        exit(1);
    }
    if (fix_labels() != 0) {
        exit(1);
    }
    generate(src);
}

// ============================================================================

// Truly horrible version
// BUG: Doesn't handle special *x cases such as awdx (awd with bit 29 on)
int lookup_op(const char* s)
{
    if (s == NULL)
        return -1;
    for (unsigned op = 0; op < ARRAY_SIZE(opcodes2text); ++op) {
        const char *txt = opcodes2text[op];
        if (txt != NULL && strcmp(txt, s) == 0) {
            // printf("op %s is %#o => %03o(%d)\n", s, op, op>>1, op&1);
            return op;
        }
    }
    return -1;
}

// ============================================================================

int add_label(const char* l)
{
    string label(l);
    int addr = code.size();
    if (label_addrs.find(label) != label_addrs.end()) {
        stringstream ss;
        ss << "label '" << label << "' already defined at " << label_addrs[label];
        yyerror(ss.str().c_str());
        return 1;
    }
    label_addrs[l] = addr;
    return 0;
}

// ============================================================================

int add_instr(const instr_t* ip, const char* operand)
{
    code_t c;
    c.instr = *ip;
    if (operand != NULL)
        c.operand = operand;
    code.push_back(c);
    if (operand != NULL) {
        label_refs.insert(pair<string,code_t*>(c.operand,  &*--code.end()));
    }
    return 0;
}

// ============================================================================

static int fix_labels(void)
{
    multimap<string,code_t*>::iterator it;
    map<string,int>::iterator addr_it = label_addrs.begin();
    int errors = 0;
    for (it = label_refs.begin(); it != label_refs.end(); ++it) {
        const string& label = (*it).first;
        code_t* codep = (*it).second;
        // addr_it = find(addr_it, label_addrs.end(), label);
        for (; addr_it != label_addrs.end(); ++ addr_it)
            if ((*addr_it).first == label)
                break;
        if (addr_it == label_addrs.end()) {
            fprintf(stderr, "%s: Label '%s' not defined.\n", cmd_name, label.c_str());
            ++ errors;
            addr_it = label_addrs.begin();
        } else {
            int addr = (*addr_it).second;
            if (codep->instr.mods.single.pr_bit)
                codep->instr.addr |= MASKBITS(15) & (unsigned) addr;
            else
                codep->instr.addr = MASKBITS(18) & (unsigned) addr;
        }
    }
    return errors != 0;
}

// ============================================================================

static void generate(char *src)
{
    string path(src);
    path.erase(path.rfind('.'));
    unsigned base_pos = path.rfind('/');
    if (base_pos == string::npos)
        base_pos = path.rfind('\\');
    if (base_pos == string::npos)
        base_pos = 0;
    else
        ++base_pos;

    string bin_path(path);
    bin_path.insert(base_pos, 1, '.');
    bin_path += ".bin.tmp";

    string lst_path(path);
    lst_path.insert(base_pos, 1, '.');
    lst_path += ".lst.tmp";

    FILE* binf = fopen(bin_path.c_str(), "w");
    if (! binf) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Cannot write %s: %s\n",
            cmd_name, bin_path.c_str(), msg);
        return;
    }
    FILE* lstf = fopen(lst_path.c_str(), "w");
    if (! lstf) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Cannot write %s: %s\n",
            cmd_name, lst_path.c_str(), msg);
        (void) unlink(bin_path.c_str());
        return;
    }

    vector<code_t>::iterator it;
    int addr = 0;
    t_uint64 word0;
    for (it = code.begin(); it != code.end(); ++it, ++addr) {
        instr_t& instr = (*it).instr;
        t_uint64 word1;
        t_uint64& word = (addr % 2 == 0) ? word0 : word1;
        encode_instr(&instr, &word);
        if (addr % 2 == 1)
            if (write72(binf, bin_path, word0, word1) != 0)
                break;
        // TODO: print labels
        fprintf(lstf, "%06o %012llo %s", addr, word, instr2text(&instr));
        if (! (*it).operand.empty())
            fprintf(lstf, "\t\"%s", (*it).operand.c_str());
        fprintf(lstf, "\n");
    }
    if (ferror(lstf) || fclose(lstf) != 0) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Error writing %s: %s\n",
            cmd_name, lst_path.c_str(), msg);
        (void) unlink(lst_path.c_str());
        (void) unlink(bin_path.c_str());
        return;
    }
    if (!ferror(binf) && addr % 2 == 1)
        (void) write72(binf, bin_path, word0, 0);
    if (ferror(binf) || fclose(binf) != 0) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Error writing %s: %s\n",
            cmd_name, bin_path.c_str(), msg);
        (void) unlink(lst_path.c_str());
        (void) unlink(bin_path.c_str());
        return;
    }

    string to_name(path);
    to_name += ".bin";
    (void) unlink(to_name.c_str());
    if (rename(bin_path.c_str(), to_name.c_str()) != 0) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Error renaming %s to %s: %s\n", cmd_name, bin_path.c_str(), to_name.c_str(), msg);
        (void) unlink(lst_path.c_str());
        (void) unlink(bin_path.c_str());
        return;
    }
    bin_path = to_name;

    to_name = path;
    to_name += ".lst";
    (void) unlink(to_name.c_str());
    if (rename(lst_path.c_str(), to_name.c_str()) != 0) {
        const char* msg = strerror(errno);
        fprintf(stderr, "%s: Error renaming %s to %s: %s\n", cmd_name, lst_path.c_str(), to_name.c_str(), msg);
        (void) unlink(lst_path.c_str());
        (void) unlink(bin_path.c_str());
        return;
    }
    printf("%s: List file is %s\n", cmd_name, to_name.c_str());
    printf("%s: Bin file is %s\n", cmd_name, bin_path.c_str());
}

// ============================================================================

static int write72(FILE* fp, const string& path, t_uint64 word0, t_uint64 word1)
{
    for (int i = 0; i < 4; ++i) {
        unsigned char c;
        c = (word0 >> (36 - 8 - i * 8)) & 0xff;
        if (fputc(c, fp) == EOF)
            return 1;
    }
    word1 |= (word0 & 0xf) << 36;
    for (int i = 0; i < 5; ++i) {
        unsigned char c;
        c = (word1 >> ((4-i) * 8)) & 0xff;
        if (fputc(c, fp) == EOF)
            return 1;
    }
    return 0;
}

//=============================================================================
