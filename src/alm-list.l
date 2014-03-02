%{
#include "alm-list.tab.h"
static int lookup(int tok);
static int lookup_hdr(void);
%}


%{
/* handle locations */
static int yycolumn = 1;

#define YY_USER_ACTION \
	yyalm_list_lloc.first_line = \
		yyalm_list_lloc.last_line = yyalm_list_lineno; \
    yyalm_list_lloc.first_column = yycolumn; \
	yyalm_list_lloc.last_column = yycolumn + yyalm_list_leng - 1; \
    yycolumn += yyalm_list_leng; \
	if (yycolumn>=40) { BEGIN(col40);}
%}

%option bison-bridge bison-locations
%option yylineno

%x col40

DEC_DIGIT   [0-9]
OCT_DIGIT   [0-7]
%%

<*>
"\n"        { yycolumn = 1; BEGIN(0); fprintf(stderr, "<EOL %d>\n", yylineno); return *yytext; } 

{OCT_DIGIT}+ {
	unsigned x;
	(void) sscanf(yytext, "%o", &x);
	yyalm_list_lval.val = x;
	return lookup(TOK_OCTAL); }


^([^ \t]|.[^ \t]|..[^ \t])[^\n]* {
	// non white-space in first three columns
	yyalm_list_lval.str = strdup(yytext); return lookup_hdr(); }

<col40>
.*$			{ yyalm_list_lval.str = strdup(yytext); return TOK_text; }


[ \t]+      { }

.           { fprintf(stderr, "Unrecognized char '%s'\n", yytext); return -1; }


%%

/* -------------------------------------------------------------------------- */

int yywrap()
{
	return 1;   /* no more files */
}

static int lookup(int tok)
{
	int lineno = yyalm_list_lloc.first_line;
	int first_col = yyalm_list_lloc.first_column;
	int last_col = yyalm_list_lloc.last_column;
	int len = last_col - first_col + 1;
	const char *tdesc;
	char buf[80];
	if (tok == TOK_OCTAL)
		tdesc = "TOK_OCTAL";
	else {
		sprintf(buf, "token # %d", tok);
		tdesc = buf;
	}
	fprintf(stderr, "Found %s at line %d, col %d ..%d (%d chars)\n",
		tdesc,
		lineno,
		first_col,
		last_col,
		len);
	if (tok != TOK_OCTAL) {
		return -1;
	}
	return -1;
}

static int lookup_hdr()
{
	int lineno = yyalm_list_lloc.first_line;
	fprintf(stderr, "Found col1text at line %d: %s\n", lineno,
		yyalm_list_lval.str);
	return(TOK_col1text);
}
