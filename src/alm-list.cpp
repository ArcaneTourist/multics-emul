#include <string>
#include <algorithm>
#include <list>
#include <vector>
#include <deque>
#include <map>
#include <iostream>
#include <fstream>
#include <sstream>
#include <errno.h>
#include <string.h>
using namespace std;

//#include "hw6180.h"
#include "alm-list.tab.h"
#include "alm-list.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(a) (sizeof(a)/sizeof(*(a)))
#endif

const char* cmd_name;

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
        cerr << "USAGE: " << cmd_name << " <filename.list>" << endl;
        exit(1);
    }
    if (yyalm_list_in != 0) {
        cerr << "internal error: yyalm_list_in" << endl;
        exit(1);
    }
    const char* filename = argv[1];
    const char* suffixp = strrchr(filename, '.');
    if (suffixp == NULL || strcmp(suffixp, ".list") != 0) {
        cerr << cmd_name << ": Filename must end in '.list'" << endl;
        exit(1);
    }
    if ((yyalm_list_in = fopen(filename, "r")) == NULL) {
        cerr << cmd_name << ": Cannot read " << filename << ": " << strerror(errno) << endl;
        exit(1);
    }

    //init_opcodes();
    int ret = yyalm_list_parse();
    if (ret != 0) {
        cerr << cmd_name << ": translation aborted." << endl;
        exit(1);
    }
    if (fclose(yyalm_list_in) != 0) {
        cerr << cmd_name << ": Error reading " << filename << ": " << strerror(errno) << endl;
        exit(1);
    }
}

// =============================================================================

void yyalm_list_error(const char* msg)
{
    cerr << "Error on line " << yyalm_list_lineno << ": " << msg << endl;
}
