%{
#include "y.tab.h"
%}

%option yylineno

ID          [a-zA-Z_][a-zA-Z_0-9]*
DEC_DIGIT   [0-9]
OCT_DIGIT   [0-7]

%%

{ID}        { yylval.str = strdup(yytext); return TOK_ID; }
'o'{OCT_DIGIT}+ {
	unsigned x; (void) sscanf(yytext, "%o", &x); yylval.val = x;
	return TOK_INT; }
{DEC_DIGIT}+    {
	(void) sscanf(yytext, "%d", &yylval.val);
	return TOK_INT; }
"-"{DEC_DIGIT}+ {
	(void) sscanf(yytext, "%d", &yylval.val);
	return TOK_INT; }
[ \t]+      { }
"\n"        { return *yytext; } 
[;=,:*]     { return *yytext; }
"\"".*      { }
.           { fprintf(stderr, "Unrecognized char '%s'\n", yytext); }

%%
int yywrap()
{
	return 1;   /* no more files */
}
