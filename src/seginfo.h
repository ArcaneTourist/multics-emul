#ifdef __cplusplus
extern "C" {
#endif

void seginfo_dump(void);
int seginfo_add_asm(int seg, int first, int last, const char* fname);
int seginfo_add_linkage(int seg, int offset, int last, const char* name);

#ifdef __cplusplus
}
#endif
