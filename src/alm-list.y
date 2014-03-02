%{

extern int yylineno;
extern void yyerror(const char* msg);
extern int yylex (void);
extern char* yytext;

//#include "hw6180.h"

%}

%error-verbose
%locations

%union {
	int val;
	unsigned uval;
	int p;
	const char* str;
};

%token <uval> TOK_OCTAL
%token <uval> TOK_DECIMAL
%token <str> TOK_col1text
%token <str> TOK_text
%token <val> TOK_lineno
%token <val> TOK_offset10
//%token <val> TOK_constant30
//%token <val> TOK_constant23
%token <str> TOK_code_reloc
%token <p> TOK_code_p_addr
%token <val> TOK_code_addr
%token <val> TOK_code_isn
%token <val> TOK_code_tag
%token <val> TOK_constant_hi
%token <val> TOK_constant_low

%%

input: /* empty */
	| alm_file
	;

alm_file :
	headers
	listing
//	literals
//	name_defs
//	defs_hash
//	ext_names
//	traps
//	type_pairs
//	internal_exp
//	linkage
//	sym_hdr
//	xref 
//	err_info
//	history
	;

headers:
	TOK_col1text |
	TOK_col1text headers
	;

listing :
	/* empty */ |
	listing source
	;

// literals: col1text source
//	;


source :
	TOK_text
	| TOK_lineno
	| TOK_lineno TOK_text
	// | macro
	| constant TOK_lineno TOK_text
//	| loc reloc_info word lineno text
//	| reloc_info constant
	;

//loc : TOK_offset10
//	;

constant:
	TOK_constant_low
	| TOK_constant_hi TOK_constant_low
	;
