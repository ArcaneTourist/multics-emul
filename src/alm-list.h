/*
   Copyright (c) 2014 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#ifdef __cplusplus
extern "C" {
#endif

typedef enum { lex_undef, lex_col, lex_ws } lex_modes;

//extern int yyalm_list_lex (void);
extern int yyalm_list_parse(void);
extern void yyalm_list_error(const char* msg);

extern int yyalm_list_lineno;
#ifdef _STDIO_H
extern FILE* yyalm_list_in;
#endif
//extern char* yytext;

//extern lex_modes lex_mode;
extern void set_lex_xref_state(int on);

extern int lookup_int(int tok);
extern int lookup_hdr(void);

#ifdef __cplusplus
}
#endif
