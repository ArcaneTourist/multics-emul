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

