/*
   Copyright (c) 2007-2013 Michael Mondy

   This software is made available under the terms of the
   ICU License -- ICU 1.8.1 and later.     
   See the LICENSE file at the top-level directory of this distribution and
   at http://example.org/project/LICENSE.
*/

#ifdef __cplusplus
extern "C" {
#endif

extern int any_parse_errors;    // error recovery allows parser to return zero

extern int lookup_op(const char* s);
extern int add_label(const char* l);
extern int add_instr(const instr_t* ip, const char* operand);

#ifdef __cplusplus
}
#endif

