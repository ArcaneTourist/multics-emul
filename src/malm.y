%{
#include <string.h>
#include <stdio.h>

extern int yylineno;
extern void yyerror(const char* msg);
extern int yylex (void);
extern char* yytext;

#include "hw6180.h"
#include "malm.h"

static instr_t instr;
static char* operand;
static int is_eis[1024];	// hack
static int lookup_mod(const char *s, int *tm, int *td);
%}

%error-verbose

%union {
	int val;
	char* str;
};


%token <val> TOK_INT
%token <str> TOK_ID 

%%

input: /* empty */
	| input line
	;

line : labels stmt
	;
labels : /* empty */
	| labels label
	;
label : TOK_ID ':' {
		if (add_label($1) != 0) {
			YYERROR;
		}
	}
	;
stmt : eol
	| instr operand eol {
#if 0
		printf("Got a stmt on line %d: op %s %03o(%d)\n\n",
			yylineno - 1,
			opcodes2text[instr.opcode],
			instr.opcode % 512, instr.opcode / 512);
#endif
#if 0
		t_uint64 word;
		encode_instr(&instr, &word);	
		printf("Got a stmt on line %d: %012llo  %s\n\n",
			yylineno - 1,
			word,
			instr2text(&instr));
#endif
		if (add_instr(&instr, operand) != 0) {
			YYERROR;
		}
		if (operand != NULL) {
			free(operand);
			operand = NULL;
		}
	}
	| error '\n' { any_parse_errors = 1;}
	;;
eol : '\n'
	| ';'
	;
instr : TOK_ID {
		int op = lookup_op($1);
		if (op < 0) {
			fprintf(stderr, "Error on line %d: Unknown opcode '%s'\n",
				yylineno, $1);
			YYERROR;
		} else {
			memset(&instr, 0, sizeof(instr));
			operand = NULL;
			instr.opcode = op;
		}
		free($1);
	}
	;
operand : simple_operand
	| simple_operand ',' modifier
	| ptr_reg '|' simple_operand
	| ptr_reg '|' simple_operand ',' modifier
	| '=' simple_operand {
			// BUG: allocate a temp or use du/dl
			yyerror("constants with '=' no supported");
			YYERROR;
	}
	| '=' simple_operand ',' modifier {
			// BUG: allocate a temp or use du/dl
			yyerror("constants with '=' no supported");
			YYERROR;
	}
	| external
	| external ',' modifier
	;
ptr_reg : TOK_ID {
		if (strncmp($1, "pr", 2) != 0) {
			yyerror("Unknown pr register");
			YYERROR;
		} else if (strlen($1) > 3 || $1[2] < 0 || $1[2] > '7') { // WARNING: not portable
			yyerror("Unknown pr register");
			YYERROR;
		} else {
			uint pr = $1[2] - '0';
			instr.mods.single.pr_bit = 1;
			instr.addr = pr << 15;
		}
	}
	;
external : TOK_ID '$' TOK_ID {
		yyerror("External references not supported");
		YYERROR;
	}
	;
simple_operand : TOK_INT {
		// WARNING: no check for 15-bit or 18-bit overflow
		if (instr.mods.single.pr_bit)
			instr.addr |= MASKBITS(15) & (unsigned) $1;
		else
			instr.addr = MASKBITS(18) & (unsigned) $1;
	}
	| TOK_ID {
			operand = $1;
	}
	;
modifier : TOK_ID {
		int tm, td;
		if (lookup_mod($1, &tm, &td)) {
			YYERROR;
		} else
			instr.mods.single.tag = (tm << 4) | td;
	}
	| '*' TOK_ID {
		int tm, td;
		if (lookup_mod($2, &tm, &td)) {
			YYERROR;
		} else if (tm == 2) {
			yyerror("Modifiers may not be *IT");
			YYERROR;
		} else {
			tm = 1;
			instr.mods.single.tag = (tm << 4) | td;
		}
	}
	| TOK_ID '*' {
		int tm, td;
		if (lookup_mod($1, &tm, &td)) {
			YYERROR;
		} else if (tm == 2) {
			yyerror("Modifiers may not be IT*");
			YYERROR;
		} else {
			tm = 3;
			instr.mods.single.tag = (tm << 4) | td;
		}
	}
	;
%%

// ===========================================================================

void yyerror(const char* msg)
{
	fprintf(stderr, "Error on line %d: %s near token '%s'\n", yylineno, msg, yytext);
}

#if 0
int main(int argc, char* argv[])
{
	return yyparse();
}
#endif

static int lookup_mod(const char *s, int *tm, int *td)
{
	*tm = -1;
	*td = -1;
	if (strcmp(s, "n") == 0) {
		*tm = 0; *td = 0;
	} else if (strcmp(s, "au") == 0) {
		*tm = 0; *td = 1;
	} else if (strcmp(s, "qu") == 0) {
		*tm = 0; *td = 2;
	} else if (strcmp(s, "du") == 0) {
		*tm = 0; *td = 3;
	} else if (strcmp(s, "ic") == 0) {
		*tm = 0; *td = 4;
	} else if (strcmp(s, "al") == 0) {
		*tm = 0; *td = 5;
	} else if (strcmp(s, "ql") == 0) {
		*tm = 0; *td = 6;
	} else if (strcmp(s, "dl") == 0) {
		*tm = 0; *td = 7;
	} else if (*s == 'x' && s[2] == 0 && s[1] >= '0' && s[1] <= '7') {
		// WARNING: not portable
		unsigned reg = s[1] - '0';
		*tm = 0; *td = reg + 010;
	} else if (strcmp(s, "f1") == 0) {
		*tm = 2; *td = 0;
	} else if (strcmp(s, "sd") == 0) {
		*tm = 2; *td = 4;
	} else if (strcmp(s, "scr") == 0) {
		*tm = 2; *td = 5;
	} else if (strcmp(s, "f2") == 0) {
		*tm = 2; *td = 6;
	} else if (strcmp(s, "f3") == 0) {
		*tm = 2; *td = 7;
	} else if (strcmp(s, "ci") == 0) {
		*tm = 2; *td = 010;
	} else if (strcmp(s, "i") == 0) {
		*tm = 2; *td = 011;
	} else if (strcmp(s, "sc") == 0) {
		*tm = 2; *td = 012;
	} else if (strcmp(s, "ad") == 0) {
		*tm = 2; *td = 013;
	} else if (strcmp(s, "di") == 0) {
		*tm = 2; *td = 014;
	} else if (strcmp(s, "dic") == 0) {
		*tm = 2; *td = 015;
	} else if (strcmp(s, "id") == 0) {
		*tm = 2; *td = 016;
	} else if (strcmp(s, "idc") == 0) {
		*tm = 2; *td = 017;
	}

	if (*tm == -1) {
		fprintf(stderr, "Error on line %d: Unknown modifier '%s'\n",
			yylineno, s);
	}

	return (*tm < 0);
}

//=============================================================================

/*
 * reg2text()
 *
 * Format a "register modification" value for display.
 *
 */

void reg2text(char* buf, uint r)
{
	switch(r) {
		case 0: *buf = 0; return;
		case 1: strcpy(buf, "au"); return;
		case 2:	strcpy(buf, "qu"); return;
		case 3:	strcpy(buf, "du"); return;
		case 4:	strcpy(buf, "ic"); return;
		case 5: strcpy(buf, "al"); return;
		case 6: strcpy(buf, "ql"); return;
		case 7:	strcpy(buf, "dl"); return;
		case 010: strcpy(buf, "x0"); return;
		case 011: strcpy(buf, "x1"); return;
		case 012: strcpy(buf, "x2"); return;
		case 013: strcpy(buf, "x3"); return;
		case 014: strcpy(buf, "x4"); return;
		case 015: strcpy(buf, "x5"); return;
		case 016: strcpy(buf, "x6"); return;
		case 017: strcpy(buf, "x7"); return;
		default:
			sprintf(buf, "<illegal reg mod td=%#o>", r);
	}
}

//=============================================================================

/*
 * mod2text()
 *
 * Format an instruction address modifer for display.
 *
 */

void mod2text(char *buf, uint tm, uint td)
{
	if (buf == NULL)
		return;
	switch(tm) {
		case 0: // R
			if (td == 0)
				*buf = 0;
			else {
				*buf = ',';
				reg2text(buf+1, td);
			}
			return;
		case 1: // RI
			*buf = ',';
			if (td == 3 || td == 7)
				sprintf(buf, "<illegal RI mod td=%#o>", td);
			else {
				reg2text(buf+1, td);
				strcpy(buf+strlen(buf), "*");
			}
			return;
		case 2:
			switch(td) {
				case 0: strcpy(buf, ",fl"); return;
				case 1: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
				case 2: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
				case 3: sprintf(buf, "<illegal IT mod td=%#o>", td); return;
				case 4: strcpy(buf, ",sd"); return;
				case 5: strcpy(buf, ",scr"); return;
				case 6: strcpy(buf, ",f2"); return;
				case 7: strcpy(buf, ",f3"); return;
				case 010: strcpy(buf, ",ci"); return;
				case 011: strcpy(buf, ",i"); return;
				case 012: strcpy(buf, ",sc"); return;
				case 013: strcpy(buf, ",ad"); return;
				case 014: strcpy(buf, ",di"); return;
				case 015: strcpy(buf, ",dic"); return;
				case 016: strcpy(buf, ",id"); return;
				case 017: strcpy(buf, ",idc"); return;
				default: sprintf(buf, "<illegal IT tag td=%#o>", td);
			}
			return;
		case 3: // IR
			buf[0] = ',';
			buf[1] = '*';
			reg2text(buf+2, td);
			return;
		default:
			sprintf(buf, "<illegal tag tm=%#o td=%#o>", tm, td);
			return;
	}
}

//=============================================================================

/*
 * instr2text()
 *
 * Format a (decoded) instruction for display.
 *
 * WARNING
 *     Returned string is a static buffer and is only valid until the next call.
 *
 * BUG -- stolen from apu.c
 */

char* instr2text(const instr_t* ip)
{
	static char buf[100];
	uint op = ip->opcode;
	char *opname = opcodes2text[op];
	if (opname == NULL) {
		strcpy(buf, "<illegal instr>");
	} else if (ip->is_eis_multiword) {
		uint32 offset = ip->addr;
		int32 soffset = sign18(ip->addr);
		sprintf(buf, "%s, variable 0%06o, inhibit %u, mf1={ar=%d, rl=%d, id=%d, reg=0%o}",
			opname, 
			offset, ip->inhibit,
			ip->mods.mf1.ar, ip->mods.mf1.rl, ip->mods.mf1.id, ip->mods.mf1.reg);
	} else {
		char mod[40];
		if (ip->mods.single.tag == 0)
			strcpy(mod, "\"\"");
		else
			mod2text(mod, ip->mods.single.tag >> 4, ip->mods.single.tag & 017);
		if (ip->mods.single.pr_bit == 0) {
			uint32 offset = ip->addr;
			int32 soffset = sign18(ip->addr);
			sprintf(buf, "%s, offset 0%06o(%+d), inhibit %u, tag %s",
				opname, 
				offset, soffset, 
				ip->inhibit, mod);
		} else {
			uint pr = ip->addr >> 15;
			int32 offset = ip->addr & MASKBITS(15);
			int32 soffset = sign15(offset);
			sprintf(buf, "%s, PR %d, offset 0%06o(%+d), inhibit %u, tag %s",
				opname, 
				pr, offset, soffset, 
				ip->inhibit, mod);
		}
	}
	return buf;
}

//=============================================================================

/*
 * encode_instr()
 *
 * Convert an instr_t struct into a  36-bit word.
 * 
 */

void encode_instr(const instr_t *ip, t_uint64 *wordp)
{
		*wordp = setbits36(0, 0, 18, ip->addr);
#if 1
		*wordp = setbits36(*wordp, 18, 10, ip->opcode);
#else
		*wordp = setbits36(*wordp, 18, 9, ip->opcode & 0777);
		*wordp = setbits36(*wordp, 27, 1, ip->opcode >> 9);
#endif
		*wordp = setbits36(*wordp, 28, 1, ip->inhibit);
		if (! is_eis[ip->opcode&MASKBITS(10)]) {
			*wordp = setbits36(*wordp, 29, 1, ip->mods.single.pr_bit);
			*wordp = setbits36(*wordp, 30, 6, ip->mods.single.tag);
		} else {
			*wordp = setbits36(*wordp, 29, 1, ip->mods.mf1.ar);
			*wordp = setbits36(*wordp, 30, 1, ip->mods.mf1.rl);
			*wordp = setbits36(*wordp, 31, 1, ip->mods.mf1.id);
			*wordp = setbits36(*wordp, 32, 4, ip->mods.mf1.reg);
		}
}

//=============================================================================

/*
 * init_opcodes()
 *
 * This initializes the is_eis[] array which we use to detect whether or
 * not an instruction is an EIS instruction. 
 *
 * TODO: Change the array values to show how many operand words are
 * used.  This would allow for better symbolic disassembly.
 *
 * BUG: unimplemented instructions may not be represented
 */

void init_opcodes()
{
	memset(is_eis, 0, sizeof(is_eis));

	is_eis[(opcode1_cmpc<<1)|1] = 1;
	is_eis[(opcode1_scd<<1)|1] = 1;
	is_eis[(opcode1_scdr<<1)|1] = 1;
	is_eis[(opcode1_scm<<1)|1] = 1;
	is_eis[(opcode1_scmr<<1)|1] = 1;
	is_eis[(opcode1_tct<<1)|1] = 1;
	is_eis[(opcode1_tctr<<1)|1] = 1;
	is_eis[(opcode1_mlr<<1)|1] = 1;
	is_eis[(opcode1_mrl<<1)|1] = 1;
	is_eis[(opcode1_mve<<1)|1] = 1;
	is_eis[(opcode1_mvt<<1)|1] = 1;
	is_eis[(opcode1_cmpn<<1)|1] = 1;
	is_eis[(opcode1_mvn<<1)|1] = 1;
	is_eis[(opcode1_mvne<<1)|1] = 1;
	is_eis[(opcode1_csl<<1)|1] = 1;
	is_eis[(opcode1_csr<<1)|1] = 1;
	is_eis[(opcode1_cmpb<<1)|1] = 1;
	is_eis[(opcode1_sztl<<1)|1] = 1;
	is_eis[(opcode1_sztr<<1)|1] = 1;
	is_eis[(opcode1_btd<<1)|1] = 1;
	is_eis[(opcode1_mvne<<1)|1] = 1;
}
